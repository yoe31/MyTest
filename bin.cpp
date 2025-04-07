// convert_geojson_to_bin.cpp
#include "map_data.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <zlib.h>

using json = nlohmann::json;

void write_string(std::ostream& out, const std::string& str) {
    size_t len = str.size();
    out.write(reinterpret_cast<const char*>(&len), sizeof(size_t));
    out.write(str.data(), len);
}

void convert(const std::string& geojson_file, const std::string& bin_file) {
    std::ifstream in(geojson_file);
    if (!in) {
        std::cerr << "Cannot open " << geojson_file << std::endl;
        return;
    }

    json j;
    in >> j;
    std::vector<Feature> features;

    for (const auto& feature_json : j["features"]) {
        Feature f;
        const auto& geometry = feature_json["geometry"];
        std::string type = geometry["type"];

        if (type == "Point") {
            f.type = GeometryType::Point;
            auto coord = geometry["coordinates"];
            f.geometry.push_back({ { coord[0], coord[1] } });

        } else if (type == "LineString") {
            f.type = GeometryType::LineString;
            std::vector<Point> line;
            for (auto& coord : geometry["coordinates"]) {
                line.push_back({ coord[0], coord[1] });
            }
            f.geometry.push_back(line);

        } else if (type == "Polygon") {
            f.type = GeometryType::Polygon;
            for (auto& ring : geometry["coordinates"]) {
                std::vector<Point> ring_points;
                for (auto& coord : ring) {
                    ring_points.push_back({ coord[0], coord[1] });
                }
                f.geometry.push_back(ring_points);
            }

        } else if (type == "MultiLineString") {
            f.type = GeometryType::MultiLineString;
            for (auto& line : geometry["coordinates"]) {
                std::vector<Point> line_points;
                for (auto& coord : line) {
                    line_points.push_back({ coord[0], coord[1] });
                }
                f.geometry.push_back(line_points);
            }

        } else if (type == "MultiPolygon") {
            f.type = GeometryType::MultiPolygon;
            for (auto& polygon : geometry["coordinates"]) {
                for (auto& ring : polygon) {
                    std::vector<Point> ring_points;
                    for (auto& coord : ring) {
                        ring_points.push_back({ coord[0], coord[1] });
                    }
                    f.geometry.push_back(ring_points);
                }
            }

        } else {
            std::cerr << "Unsupported geometry: " << type << "\n";
            continue;
        }

        // 모든 properties 저장
        for (auto& [key, val] : feature_json["properties"].items()) {
            f.properties[key] = val.dump(); // 문자열로 저장 (숫자도 포함 가능)
        }

        features.push_back(f);
    }

    // 원본 바이너리 버퍼 생성
    std::stringstream buffer(std::ios::binary | std::ios::in | std::ios::out);
    size_t feature_count = features.size();
    buffer.write(reinterpret_cast<const char*>(&feature_count), sizeof(size_t));

    for (const auto& f : features) {
        uint8_t type = static_cast<uint8_t>(f.type);
        buffer.write(reinterpret_cast<const char*>(&type), sizeof(uint8_t));

        // geometry
        size_t outer = f.geometry.size();
        buffer.write(reinterpret_cast<const char*>(&outer), sizeof(size_t));
        for (const auto& line : f.geometry) {
            size_t count = line.size();
            buffer.write(reinterpret_cast<const char*>(&count), sizeof(size_t));
            buffer.write(reinterpret_cast<const char*>(line.data()), sizeof(Point) * count);
        }

        // properties
        size_t prop_count = f.properties.size();
        buffer.write(reinterpret_cast<const char*>(&prop_count), sizeof(size_t));
        for (const auto& [k, v] : f.properties) {
            write_string(buffer, k);
            write_string(buffer, v);
        }
    }

    std::string raw = buffer.str();

    // zlib 압축
    uLongf compressed_size = compressBound(raw.size());
    std::vector<uint8_t> compressed_data(compressed_size);
    if (compress(compressed_data.data(), &compressed_size,
                 reinterpret_cast<const Bytef*>(raw.data()), raw.size()) != Z_OK) {
        std::cerr << "Compression failed\n";
        return;
    }

    std::ofstream out(bin_file, std::ios::binary);
    out.write(reinterpret_cast<const char*>(&raw.size()), sizeof(size_t)); // 압축 전 크기
    out.write(reinterpret_cast<const char*>(&compressed_size), sizeof(size_t));
    out.write(reinterpret_cast<const char*>(compressed_data.data()), compressed_size);
    std::cout << "Saved compressed binary: " << bin_file << "\n";
}