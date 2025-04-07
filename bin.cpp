#include "map_data.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <zlib.h>

using json = nlohmann::json;

// vector<char>에 바이너리 데이터 쓰기
void write_to_buffer(std::vector<char>& buffer, const void* data, size_t size) {
    const char* cdata = reinterpret_cast<const char*>(data);
    buffer.insert(buffer.end(), cdata, cdata + size);
}

// 문자열 저장 (길이 + 내용)
void write_string(std::vector<char>& buffer, const std::string& str) {
    size_t len = str.size();
    write_to_buffer(buffer, &len, sizeof(size_t));
    write_to_buffer(buffer, str.data(), len);
}

void convert_geojson_to_bin(const std::string& geojson_file, const std::string& bin_file) {
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
        const std::string& type = geometry["type"];

        if (type == "Point") {
            f.type = GeometryType::Point;
            auto coord = geometry["coordinates"];
            f.geometry.push_back({ { coord[0], coord[1] } });

        } else if (type == "LineString") {
            f.type = GeometryType::LineString;
            std::vector<Point> line;
            for (const auto& coord : geometry["coordinates"]) {
                line.push_back({ coord[0], coord[1] });
            }
            f.geometry.push_back(line);

        } else if (type == "Polygon") {
            f.type = GeometryType::Polygon;
            for (const auto& ring : geometry["coordinates"]) {
                std::vector<Point> ring_points;
                for (const auto& coord : ring) {
                    ring_points.push_back({ coord[0], coord[1] });
                }
                f.geometry.push_back(ring_points);
            }

        } else if (type == "MultiLineString") {
            f.type = GeometryType::MultiLineString;
            for (const auto& line : geometry["coordinates"]) {
                std::vector<Point> line_points;
                for (const auto& coord : line) {
                    line_points.push_back({ coord[0], coord[1] });
                }
                f.geometry.push_back(line_points);
            }

        } else if (type == "MultiPolygon") {
            f.type = GeometryType::MultiPolygon;
            for (const auto& polygon : geometry["coordinates"]) {
                for (const auto& ring : polygon) {
                    std::vector<Point> ring_points;
                    for (const auto& coord : ring) {
                        ring_points.push_back({ coord[0], coord[1] });
                    }
                    f.geometry.push_back(ring_points);
                }
            }

        } else {
            std::cerr << "Unsupported geometry type: " << type << std::endl;
            continue;
        }

        // properties 저장 (C++11 호환)
        auto props = feature_json["properties"];
        for (auto it = props.items().begin(); it != props.items().end(); ++it) {
            std::string key = it.key();
            std::string val = it.value().dump(); // JSON 형태로 저장
            f.properties[key] = val;
        }

        features.push_back(f);
    }

    // 바이너리 버퍼 생성
    std::vector<char> buffer;
    size_t feature_count = features.size();
    write_to_buffer(buffer, &feature_count, sizeof(size_t));

    for (const auto& f : features) {
        uint8_t type = static_cast<uint8_t>(f.type);
        write_to_buffer(buffer, &type, sizeof(uint8_t));

        // geometry
        size_t outer = f.geometry.size();
        write_to_buffer(buffer, &outer, sizeof(size_t));
        for (const auto& line : f.geometry) {
            size_t count = line.size();
            write_to_buffer(buffer, &count, sizeof(size_t));
            write_to_buffer(buffer, line.data(), sizeof(Point) * count);
        }

        // properties
        size_t prop_count = f.properties.size();
        write_to_buffer(buffer, &prop_count, sizeof(size_t));
        for (const auto& kv : f.properties) {
            write_string(buffer, kv.first);
            write_string(buffer, kv.second);
        }
    }

    // 압축
    uLongf compressed_size = compressBound(buffer.size());
    std::vector<uint8_t> compressed_data(compressed_size);

    if (compress(compressed_data.data(), &compressed_size,
                 reinterpret_cast<const Bytef*>(buffer.data()), buffer.size()) != Z_OK) {
        std::cerr << "Compression failed\n";
        return;
    }

    // 압축된 데이터 저장
    std::ofstream out(bin_file, std::ios::binary);
    size_t uncompressed_size = buffer.size();
    out.write(reinterpret_cast<const char*>(&uncompressed_size), sizeof(size_t));
    out.write(reinterpret_cast<const char*>(&compressed_size), sizeof(size_t));
    out.write(reinterpret_cast<const char*>(compressed_data.data()), compressed_size);
    out.close();

    std::cout << "Saved compressed binary: " << bin_file << " (" << compressed_size << " bytes)\n";
}