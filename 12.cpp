#include <stdio.h>
#include <math.h>

// WGS84 기준
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define EARTH_RADIUS 6378137.0          // [m]
#define FLATTENING (1.0 / 298.257223563)
#define ECC_SQ (2 * FLATTENING - FLATTENING * FLATTENING)

// 후륜 기준 LLH → 전방범퍼 LLH로 보정
void adjustLLHToFrontBumper(double lat_deg, double lon_deg, double h, double heading_deg,
                            double dx, double dy,
                            double* out_lat_deg, double* out_lon_deg, double* out_h) {
    // 입력값: heading은 도 단위 (0 = 북쪽), 반시계 방향 양수
    double lat_rad = lat_deg * DEG_TO_RAD;
    double lon_rad = lon_deg * DEG_TO_RAD;
    double heading_rad = heading_deg * DEG_TO_RAD;

    // N: 위도에 따른 곡률 반경
    double sin_lat = sin(lat_rad);
    double N = EARTH_RADIUS / sqrt(1 - ECC_SQ * sin_lat * sin_lat);

    // 차체 좌표계에서 전방범퍼 위치 (dx, dy)를 heading 방향으로 회전
    double d_north = dx * cos(heading_rad) - dy * sin(heading_rad);
    double d_east  = dx * sin(heading_rad) + dy * cos(heading_rad);

    // 위도 변화: 북쪽 이동
    double d_lat = d_north / (N + h);
    // 경도 변화: 동쪽 이동
    double d_lon = d_east / ((N + h) * cos(lat_rad));

    // 출력
    *out_lat_deg = lat_deg + d_lat * RAD_TO_DEG;
    *out_lon_deg = lon_deg + d_lon * RAD_TO_DEG;
    *out_h = h;
}

int main() {
    // 후륜 중심 LLH (예시)
    double rear_lat = 37.5665;   // 위도
    double rear_lon = 126.9780;  // 경도
    double rear_alt = 35.0;      // 고도 [m]
    double heading = 0.0;        // 북쪽 기준, 반시계 + (0도면 북쪽, 90도면 서쪽)

    // 오프셋 (전방범퍼 위치, 차량기준)
    double dx = 3.976; // [m] 앞쪽
    double dy = 0.0;   // [m] 왼쪽

    double front_lat, front_lon, front_alt;
    adjustLLHToFrontBumper(rear_lat, rear_lon, rear_alt, heading, dx, dy,
                           &front_lat, &front_lon, &front_alt);

    printf("전방범퍼 LLH:\n위도: %.10f\n경도: %.10f\n고도: %.2f\n", front_lat, front_lon, front_alt);
    return 0;
}