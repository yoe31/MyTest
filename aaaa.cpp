#include <filesystem>
#include <iostream>
#include "convert_geojson_to_bin.hpp" // 앞서 만든 convert 함수 포함된 헤더

namespace fs = std::filesystem;

void process_directory(const std::string& input_dir, const std::string& output_dir) {
    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (!entry.is_regular_file()) continue;

        const auto& path = entry.path();
        if (path.extension() != ".geojson") continue;

        fs::path input_path = path;
        fs::path filename = input_path.filename();
        fs::path output_path = fs::path(output_dir) / filename.replace_extension(".bin");

        std::cout << "Converting: " << input_path << " -> " << output_path << "\n";
        convert_geojson_to_bin(input_path.string(), output_path.string());
    }
}

int main() {
    process_directory("origin/a", "new/a");
    process_directory("origin/b", "new/b");

    return 0;
}