#include <stdio.h>
#include <math.h>

typedef struct {
    float x;
    float y;
} Point;

float distance(Point a, Point b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

void normalize(float* dx, float* dy) {
    float len = sqrtf((*dx) * (*dx) + (*dy) * (*dy));
    if (len > 1e-6f) {
        *dx /= len;
        *dy /= len;
    }
}

// 개선된: 선분 기준 비율로 이동
int extract_shifted_points_from_segments(const Point* points, int N,
                                         float start_ratio, float end_ratio, float shift_distance,
                                         Point* shifted_out, int max_output_count) {
    if (N < 2 || start_ratio < 0.0f || end_ratio > 1.0f || start_ratio >= end_ratio)
        return 0;

    // 1. 전체 길이 계산
    float total_length = 0.0f;
    float segment_lengths[N - 1];
    for (int i = 0; i < N - 1; ++i) {
        segment_lengths[i] = distance(points[i], points[i + 1]);
        total_length += segment_lengths[i];
    }

    // 2. 선분 기준으로 비율 판단 및 평행이동
    float accumulated_length = 0.0f;
    int out_count = 0;

    for (int i = 0; i < N - 1; ++i) {
        float seg_len = segment_lengths[i];
        float start_ratio_seg = accumulated_length / total_length;
        float end_ratio_seg = (accumulated_length + seg_len) / total_length;

        // 만약 선분이 원하는 비율 범위와 겹친다면, 이 선분을 기준으로 shift
        if (end_ratio_seg > start_ratio && start_ratio_seg < end_ratio) {
            if (out_count >= max_output_count) break;

            // 시작점 기준으로 이동
            float dx = points[i + 1].x - points[i].x;
            float dy = points[i + 1].y - points[i].y;
            normalize(&dx, &dy);

            float perp_x = -dy;
            float perp_y = dx;

            Point shifted;
            shifted.x = points[i].x + shift_distance * perp_x;
            shifted.y = points[i].y + shift_distance * perp_y;

            shifted_out[out_count++] = shifted;
        }

        accumulated_length += seg_len;
    }

    return out_count;
}