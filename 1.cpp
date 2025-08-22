#include <stdio.h>
#include <math.h>

typedef struct {
    float x;
    float y;
} Point;

// 두 점 사이 거리
float distance(Point a, Point b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

// 벡터 정규화
void normalize(float* dx, float* dy) {
    float len = sqrtf((*dx) * (*dx) + (*dy) * (*dy));
    if (len > 1e-6f) {
        *dx /= len;
        *dy /= len;
    }
}

// 평행 이동 수행
void shift_points_parallel(Point* points, int N, float start_ratio, float end_ratio, float shift_distance) {
    if (N < 2 || start_ratio < 0.0f || end_ratio > 1.0f || start_ratio >= end_ratio) return;

    float segment_lengths[N];
    float total_length = 0.0f;

    segment_lengths[0] = 0.0f;
    for (int i = 1; i < N; ++i) {
        segment_lengths[i] = distance(points[i - 1], points[i]);
        total_length += segment_lengths[i];
    }

    float accumulated_length = 0.0f;
    for (int i = 0; i < N; ++i) {
        if (i > 0)
            accumulated_length += segment_lengths[i];

        float ratio = accumulated_length / total_length;
        if (ratio >= start_ratio && ratio < end_ratio) {
            // 인접한 선분의 방향 벡터 (앞 또는 뒤)
            float dx, dy;

            if (i < N - 1) {
                dx = points[i + 1].x - points[i].x;
                dy = points[i + 1].y - points[i].y;
            } else {
                dx = points[i].x - points[i - 1].x;
                dy = points[i].y - points[i - 1].y;
            }

            normalize(&dx, &dy);  // 방향 벡터 단위화

            // 수직 방향 벡터 (좌측 방향: 시계 반대 방향 90도 회전)
            float perp_x = -dy;
            float perp_y = dx;

            // 평행이동
            points[i].x += shift_distance * perp_x;
            points[i].y += shift_distance * perp_y;
        }
    }
}