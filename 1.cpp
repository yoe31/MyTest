#include <stdio.h>
#include <math.h>

#define MAX_POINTS 1000
#define INTERVAL 1.0   // 샘플링 간격 (m)
#define SHIFT_DIST 3.5 // 평행이동 거리 (m)
#define TARGET_LEN 80.0

typedef struct {
    double x;
    double y;
} Point;

double distance(Point a, Point b) {
    return hypot(b.x - a.x, b.y - a.y);
}

void generate_shifted_segment(Point* input, int num_points, Point* output, int* output_count) {
    // 누적 거리 계산
    double* cum_dist = (double*)malloc(sizeof(double) * num_points);
    cum_dist[0] = 0.0;
    for (int i = 1; i < num_points; ++i) {
        cum_dist[i] = cum_dist[i - 1] + distance(input[i - 1], input[i]);
    }

    // 전체 길이
    double total_len = cum_dist[num_points - 1];
    if (total_len < TARGET_LEN) {
        printf("입력 선분의 길이가 80m보다 짧습니다.\n");
        *output_count = 0;
        free(cum_dist);
        return;
    }

    // 80m만큼 뒤에서부터 탐색 시작 (끝점 기준)
    double start_dist = total_len - TARGET_LEN;
    int idx = 0;
    while (idx < num_points - 1 && cum_dist[idx + 1] < start_dist) {
        idx++;
    }

    // 초기 위치 보간
    Point shifted[MAX_POINTS];
    int shifted_count = 0;

    double remain = start_dist - cum_dist[idx];
    Point p1 = input[idx];
    Point p2 = input[idx + 1];
    double seg_len = distance(p1, p2);
    double ratio = remain / seg_len;
    Point pos;
    pos.x = p1.x + (p2.x - p1.x) * ratio;
    pos.y = p1.y + (p2.y - p1.y) * ratio;

    // 시작점을 법선 방향으로 이동
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double norm = hypot(dx, dy);
    double nx = -dy / norm;
    double ny = dx / norm;
    pos.x += SHIFT_DIST * nx;
    pos.y += SHIFT_DIST * ny;
    shifted[shifted_count++] = pos;

    double accumulated = 0.0;
    double step = INTERVAL;
    while (accumulated < TARGET_LEN && idx < num_points - 1) {
        double seg_remain = distance(input[idx], input[idx + 1]) * (1.0 - ratio);
        if (seg_remain + accumulated < step) {
            accumulated += seg_remain;
            idx++;
            ratio = 0.0;
            continue;
        }

        double d = step - accumulated;
        ratio += d / distance(input[idx], input[idx + 1]);
        if (ratio > 1.0) {
            ratio = 1.0;
        }

        Point p;
        p.x = input[idx].x + (input[idx + 1].x - input[idx].x) * ratio;
        p.y = input[idx].y + (input[idx + 1].y - input[idx].y) * ratio;

        // 평행이동
        dx = input[idx + 1].x - input[idx].x;
        dy = input[idx + 1].y - input[idx].y;
        norm = hypot(dx, dy);
        nx = -dy / norm;
        ny = dx / norm;
        p.x += SHIFT_DIST * nx;
        p.y += SHIFT_DIST * ny;

        shifted[shifted_count++] = p;
        accumulated = 0.0;
    }

    // 출력 복사
    for (int i = 0; i < shifted_count; ++i) {
        output[i] = shifted[i];
    }
    *output_count = shifted_count;
    free(cum_dist);
}

int main() {
    Point input[] = {
        {0, 0}, {0, 20}, {0, 40}, {0, 60}, {0, 90}, {0, 120}
    };
    int num_input = sizeof(input) / sizeof(input[0]);

    Point output[MAX_POINTS];
    int output_count = 0;

    generate_shifted_segment(input, num_input, output, &output_count);

    printf("생성된 점 개수: %d\n", output_count);
    for (int i = 0; i < output_count; ++i) {
        printf("점 %d: (%.3f, %.3f)\n", i, output[i].x, output[i].y);
    }

    return 0;
}