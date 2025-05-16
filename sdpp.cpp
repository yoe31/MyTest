#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <set>

constexpr double EARTH_RADIUS_M = 6371000.0; // Earth radius in meters
constexpr double TILE_SIZE_M = 200.0;        // 200m x 200m tiles
constexpr double SEARCH_RADIUS_M = 2000.0;   // 2km search radius

// ------------------------- Structures -------------------------

struct LatLon {
    double lat;
    double lon;
};

struct Link {
    int id;
    std::vector<LatLon> shape;
};

struct TileIndex {
    int x;
    int y;

    bool operator==(const TileIndex& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const TileIndex& other) const {
        return (x == other.x) ? (y < other.y) : (x < other.x);
    }
};

// ------------------------- Hash for unordered_map -------------------------

namespace std {
template<>
struct hash<TileIndex> {
    std::size_t operator()(const TileIndex& t) const {
        return std::hash<int>()(t.x) ^ (std::hash<int>()(t.y) << 1);
    }
};
}

// ------------------------- Utility Functions -------------------------

// Convert LatLon to local X/Y coordinates (in meters) relative to origin
void latlon_to_local_xy(const LatLon& origin, const LatLon& point, double& x, double& y) {
    double lat_rad = origin.lat * M_PI / 180.0;
    double d_lat = (point.lat - origin.lat) * M_PI / 180.0;
    double d_lon = (point.lon - origin.lon) * M_PI / 180.0;

    x = EARTH_RADIUS_M * d_lon * cos(lat_rad);
    y = EARTH_RADIUS_M * d_lat;
}

// Convert LatLon to TileIndex based on origin
TileIndex get_tile_index(const LatLon& origin, const LatLon& point) {
    double x, y;
    latlon_to_local_xy(origin, point, x, y);
    int tile_x = static_cast<int>(std::floor(x / TILE_SIZE_M));
    int tile_y = static_cast<int>(std::floor(y / TILE_SIZE_M));
    return { tile_x, tile_y };
}

// Assign each link to tiles it touches
void assign_links_to_tiles(const std::vector<Link>& links, const LatLon& origin,
                           std::unordered_map<TileIndex, std::vector<const Link*>>& tile_map) {
    for (const auto& link : links) {
        std::set<TileIndex> occupied_tiles;
        for (const auto& pt : link.shape) {
            TileIndex idx = get_tile_index(origin, pt);
            occupied_tiles.insert(idx);
        }
        for (const auto& idx : occupied_tiles) {
            tile_map[idx].push_back(&link);
        }
    }
}

// Get all tile indices within 2km radius of gps_point
std::vector<TileIndex> get_nearby_tile_indices(const LatLon& origin, const LatLon& gps_point) {
    double x, y;
    latlon_to_local_xy(origin, gps_point, x, y);

    int center_x = static_cast<int>(std::floor(x / TILE_SIZE_M));
    int center_y = static_cast<int>(std::floor(y / TILE_SIZE_M));

    int range = static_cast<int>(std::ceil(SEARCH_RADIUS_M / TILE_SIZE_M));
    std::vector<TileIndex> result;
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            int tx = center_x + dx;
            int ty = center_y + dy;

            double tile_center_x = (tx + 0.5) * TILE_SIZE_M;
            double tile_center_y = (ty + 0.5) * TILE_SIZE_M;

            double dist_sq = (tile_center_x - x) * (tile_center_x - x) +
                             (tile_center_y - y) * (tile_center_y - y);
            if (dist_sq <= SEARCH_RADIUS_M * SEARCH_RADIUS_M) {
                result.push_back({ tx, ty });
            }
        }
    }
    return result;
}

// ------------------------- Main -------------------------

int main() {
    // 기준점 (서울 시청 근처)
    LatLon origin = {37.5665, 126.9780};

    // 예시 Link 데이터
    std::vector<Link> links = {
        {1, {{37.5665, 126.9780}, {37.5667, 126.9782}}},
        {2, {{37.5670, 126.9785}, {37.5672, 126.9788}}},
        {3, {{37.5700, 126.9800}, {37.5702, 126.9802}}},
        {4, {{37.5600, 126.9700}, {37.5602, 126.9702}}}
    };

    // 타일 맵 생성
    std::unordered_map<TileIndex, std::vector<const Link*>> tile_map;
    assign_links_to_tiles(links, origin, tile_map);

    // GPS 수신 위치
    LatLon gps = {37.5668, 126.9784};

    // 반경 2km 내 타일들 추출
    auto nearby_tiles = get_nearby_tile_indices(origin, gps);

    // 타일 내 Link 추출
    std::set<int> link_ids;
    for (const auto& tile : nearby_tiles) {
        auto it = tile_map.find(tile);
        if (it != tile_map.end()) {
            for (const auto* link_ptr : it->second) {
                link_ids.insert(link_ptr->id);
            }
        }
    }

    // 결과 출력
    std::cout << "Nearby Links within 2km of GPS position:" << std::endl;
    for (int id : link_ids) {
        std::cout << "Link ID: " << id << std::endl;
    }

    return 0;
}