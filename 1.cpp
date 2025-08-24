#include <stdio.h>
#include <math.h>

typedef struct {
    double x;
    double y;
} Point;

double distance(Point a, Point b) {
    return hypot(b.x - a.x, b.y - a.y);
}

Point interpolate(Point a, Point b, double t) {
    Point p;
    p.x = a.x + (b.x - a.x) * t;
    p.y = a.y + (b.y - a.y) * t;
    return p;
}

Point get_normal(Point a, Point b) {
    Point dir = {b.x - a.x, b.y - a.y};
    double len = distance(a, b);
    Point n = {-dir.y / len, dir.x / len};  // 반시계 방향 90도 회전
    return n;
}

int extract_shifted_segment(
    Point* input, int input_len,
    double start_ratio, double end_ratio,
    double offset,
    Point* output, int* output_len
) {
    if (input_len < 2 || start_ratio >= end_ratio) return 0;

    // 1. 전체 길이 계산
    double total_len = 0;
    double seg_len[input_len - 1];
    for (int i = 0; i < input_len - 1; i++) {
        seg_len[i] = distance(input[i], input[i+1]);
        total_len += seg_len[i];
    }

    double start_dist = total_len * start_ratio;
    double end_dist   = total_len * end_ratio;

    // 2. 자를 구간 구성 (보간 포함)
    Point temp[1024];  // 충분히 큰 임시 버퍼
    int temp_count = 0;

    double acc_len = 0;
    for (int i = 0; i < input_len - 1; i++) {
        double next_len = acc_len + seg_len[i];

        // start point 보간 위치
        if (acc_len <= start_dist && start_dist < next_len) {
            double t = (start_dist - acc_len) / seg_len[i];
            temp[temp_count++] = interpolate(input[i], input[i+1], t);
        }

        // 내부 점 추가
        if (next_len > start_dist && next_len < end_dist) {
            temp[temp_count++] = input[i+1];
        }

        // end point 보간 위치
        if (acc_len < end_dist && end_dist <= next_len) {
            double t = (end_dist - acc_len) / seg_len[i];
            temp[temp_count++] = interpolate(input[i], input[i+1], t);
            break;
        }

        acc_len = next_len;
    }

    // 3. 평행이동
    for (int i = 0; i < temp_count; i++) {
        Point normal;
        if (i < temp_count - 1) {
            normal = get_normal(temp[i], temp[i+1]);
        } else {
            normal = get_normal(temp[i-1], temp[i]);
        }
        output[i].x = temp[i].x + offset * normal.x;
        output[i].y = temp[i].y + offset * normal.y;
    }

    *output_len = temp_count;
    return 1;
}