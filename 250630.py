#include <stdio.h>
#include <math.h>

// 점 구조체
typedef struct {
    double x;
    double y;
} Point;

// 두 점 사이 거리 계산 함수
double distance(Point a, Point b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

// 50m 지점의 전체 비율 계산 함수
double get_ratio_at_50m(Point *points, int N) {
    if (N < 2) return 0.0;

    double total_length = 0.0;
    for (int i = 1; i < N; i++) {
        total_length += distance(points[i - 1], points[i]);
    }

    if (total_length == 0.0) return 0.0;

    double target_length = 50.0;
    double accumulated = 0.0;

    for (int i = 1; i < N; i++) {
        double seg_length = distance(points[i - 1], points[i]);
        if (accumulated + seg_length >= target_length) {
            double remaining = target_length - accumulated;
            double partial_ratio = remaining / seg_length;
            double ratio_up_to_prev = accumulated / total_length;
            double ratio_in_seg = (seg_length == 0.0) ? 0.0 : remaining / total_length;
            return ratio_up_to_prev + ratio_in_seg;
        }
        accumulated += seg_length;
    }

    // 50m를 넘지 못한 경우 전체 비율은 1.0 (즉, 전체 끝까지 가도 50m 안 됨)
    return 1.0;
}