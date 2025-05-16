#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <set>

constexpr double EARTH_RADIUS_M = 6371000.0; // Earth radius in meters
constexpr double TILE_SIZE_M = 200.0;        // 200m x 200m
constexpr double SEARCH_RADIUS_M = 2000.0;   // 2km 검색 반경

// -------------------- 구조체 --------------------

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
};

// -------------------- 유틸 함수 --------------------

// 위경도 → 기준점 기준 XY 좌표 (meter)
void latlon_to_local_xy(const LatLon& origin, const LatLon& point, double& x, double& y) {
    double lat_rad = origin.lat * M_PI / 180.0;
    double d_lat = (point.lat - origin.lat) * M_PI / 180.0;
    double d_lon = (point.lon - origin.lon) * M_PI / 180.0;

    x = EARTH_RADIUS_M * d_lon * cos(lat_rad);
    y = EARTH_RADIUS_M * d_lat;
}

// XY 좌표 → 타일 인덱스
TileIndex get_tile_index(const LatLon& origin, const LatLon& point) {
    double x, y;
    latlon_to_local_xy(origin, point, x, y);
    int tile_x = static_cast<int>(std::floor(x / TILE_SIZE_M));
    int tile_y = static_cast<int>(std::floor(y / TILE_SIZE_M));
    return { tile_x, tile_y };
}

// 타일 인덱스를 32비트 정수 ID로 변환
int32_t tile_index_to_id(const TileIndex& idx) {
    // 16비트로 음수 표현을 위해 & 0xFFFF
    return (idx.x << 16) ^ (idx.y & 0xFFFF);
}

// -------------------- 타일 등록 --------------------

void assign_links_to_tiles(const std::vector<Link>& links, const LatLon& origin,
                           std::unordered_map<int32_t, std::vector<const Link*>>& tile_map) {
    for (const auto& link : links) {
        std::set<int32_t> occupied_tile_ids;
        for (const auto& pt : link.shape) {
            TileIndex idx = get_tile_index(origin, pt);
            int32_t tile_id = tile_index_to_id(idx);
            occupied_tile_ids.insert(tile_id);
        }
        for (int32_t tile_id : occupied_tile_ids) {
            tile_map[tile_id].push_back(&link);
        }
    }
}

// -------------------- 반경 내 타일 ID 검색 --------------------

std::vector<int32_t> get_nearby_tile_ids(const LatLon& origin, const LatLon& gps_point) {
    double x, y;
    latlon_to_local_xy(origin, gps_point, x, y);

    int center_x = static_cast<int>(std::floor(x / TILE_SIZE_M));
    int center_y = static_cast<int>(std::floor(y / TILE_SIZE_M));

    int range = static_cast<int>(std::ceil(SEARCH_RADIUS_M / TILE_SIZE_M));
    std::vector<int32_t> result;

    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            int tx = center_x + dx;
            int ty = center_y + dy;

            double tile_center_x = (tx + 0.5) * TILE_SIZE_M;
            double tile_center_y = (ty + 0.5) * TILE_SIZE_M;

            double dist_sq = (tile_center_x - x) * (tile_center_x - x) +
                             (tile_center_y - y) * (tile_center_y - y);
            if (dist_sq <= SEARCH_RADIUS_M * SEARCH_RADIUS_M) {
                result.push_back(tile_index_to_id({ tx, ty }));
            }
        }
    }
    return result;
}

// -------------------- 메인 함수 --------------------

int main() {
    // 기준 위경도 (예: 서울 시청)
    LatLon origin = {37.5665, 126.9780};

    // 예제 링크 데이터
    std::vector<Link> links = {
        {1, {{37.5665, 126.9780}, {37.5667, 126.9782}}},
        {2, {{37.5670, 126.9785}, {37.5672, 126.9788}}},
        {3, {{37.5700, 126.9800}, {37.5702, 126.9802}}},
        {4, {{37.5600, 126.9700}, {37.5602, 126.9702}}}
    };

    // 타일 맵
    std::unordered_map<int32_t, std::vector<const Link*>> tile_map;
    assign_links_to_tiles(links, origin, tile_map);

    // GPS 입력값
    LatLon gps = {37.5668, 126.9784};

    // 반경 2km 내 타일 ID 검색
    auto nearby_tile_ids = get_nearby_tile_ids(origin, gps);

    // 해당 타일에 속한 링크 찾기
    std::set<int> link_ids;
    for (int32_t tile_id : nearby_tile_ids) {
        auto it = tile_map.find(tile_id);
        if (it != tile_map.end()) {
            for (const auto* link_ptr : it->second) {
                link_ids.insert(link_ptr->id);
            }
        }
    }

    // 결과 출력
    std::cout << "Links within 2km of GPS:" << std::endl;
    for (int id : link_ids) {
        std::cout << "Link ID: " << id << std::endl;
    }

    return 0;
}