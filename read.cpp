// read_bin.cpp
#include "map_data.hpp"
#include <fstream>
#include <iostream>
#include <zlib.h>

void read_string(std::istream& in, std::string& str) {
    size_t len;
    in.read(reinterpret_cast<char*>(&len), sizeof(size_t));
    str.resize(len);
    in.read(str.data(), len);
}

std::vector<Feature> load_binary(const std::string& bin_file) {
    std::ifstream in(bin_file, std::ios::binary);
    if (!in) {
        std::cerr << "Cannot open file\n";
        return {};
    }

    size_t original_size, compressed_size;
    in.read(reinterpret_cast<char*>(&original_size), sizeof(size_t));
    in.read(reinterpret_cast<char*>(&compressed_size), sizeof(size_t));

    std::vector<uint8_t> compressed_data(compressed_size);
    in.read(reinterpret_cast<char*>(compressed_data.data()), compressed_size);

    std::vector<uint8_t> decompressed_data(original_size);
    if (uncompress(decompressed_data.data(), &original_size,
                   compressed_data.data(), compressed_size) != Z_OK) {
        std::cerr << "Decompression failed\n";
        return {};
    }

    std::stringstream buffer(std::string(decompressed_data.begin(), decompressed_data.end()), std::ios::binary);

    std::vector<Feature> features;
    size_t feature_count;
    buffer.read(reinterpret_cast<char*>(&feature_count), sizeof(size_t));

    for (size_t i = 0; i < feature_count; ++i) {
        Feature f;
        uint8_t type;
        buffer.read(reinterpret_cast<char*>(&type), sizeof(uint8_t));
        f.type = static_cast<GeometryType>(type);

        size_t outer;
        buffer.read(reinterpret_cast<char*>(&outer), sizeof(size_t));
        f.geometry.resize(outer);
        for (size_t j = 0; j < outer; ++j) {
            size_t count;
            buffer.read(reinterpret_cast<char*>(&count), sizeof(size_t));
            f.geometry[j].resize(count);
            buffer.read(reinterpret_cast<char*>(f.geometry[j].data()), sizeof(Point) * count);
        }

        size_t prop_count;
        buffer.read(reinterpret_cast<char*>(&prop_count), sizeof(size_t));
        for (size_t j = 0; j < prop_count; ++j) {
            std::string key, val;
            read_string(buffer, key);
            read_string(buffer, val);
            f.properties[key] = val;
        }

        features.push_back(f);
    }

    return features;
}

int main() {
    auto features = load_binary("1234.bin");
    std::cout << "Loaded " << features.size() << " features\n";
    for (const auto& f : features) {
        std::cout << "- Geometry type: " << static_cast<int>(f.type)
                  << ", properties: " << f.properties.size() << "\n";
    }
}