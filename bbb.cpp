// read_bin.cpp
#include "map_data.hpp"
#include <fstream>
#include <iostream>

std::vector<Feature> load_binary(const std::string& bin_file) {
    std::ifstream in(bin_file, std::ios::binary);
    if (!in) {
        std::cerr << "Cannot open " << bin_file << std::endl;
        return {};
    }

    size_t feature_count;
    in.read(reinterpret_cast<char*>(&feature_count), sizeof(size_t));

    std::vector<Feature> features;

    for (size_t i = 0; i < feature_count; ++i) {
        Feature f;
        uint8_t type;
        in.read(reinterpret_cast<char*>(&type), sizeof(uint8_t));
        f.type = static_cast<GeometryType>(type);

        size_t id_len, name_len;
        in.read(reinterpret_cast<char*>(&id_len), sizeof(size_t));
        f.id.resize(id_len);
        in.read(f.id.data(), id_len);

        in.read(reinterpret_cast<char*>(&name_len), sizeof(size_t));
        f.name.resize(name_len);
        in.read(f.name.data(), name_len);

        size_t outer_count;
        in.read(reinterpret_cast<char*>(&outer_count), sizeof(size_t));
        f.geometry.resize(outer_count);

        for (size_t j = 0; j < outer_count; ++j) {
            size_t point_count;
            in.read(reinterpret_cast<char*>(&point_count), sizeof(size_t));
            f.geometry[j].resize(point_count);
            in.read(reinterpret_cast<char*>(f.geometry[j].data()), sizeof(Point) * point_count);
        }

        features.push_back(f);
    }

    return features;
}

int main() {
    auto features = load_binary("1234.bin");
    std::cout << "Loaded " << features.size() << " features\n";
    for (const auto& f : features) {
        std::cout << " - ID: " << f.id << ", Name: " << f.name
                  << ", Type: " << static_cast<int>(f.type)
                  << ", Geometry count: " << f.geometry.size() << "\n";
    }
}