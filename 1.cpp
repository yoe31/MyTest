#include <stdio.h>
#include <math.h>

#define MAX_POINTS 1000
#define OFFSET 3.5
#define TARGET_LENGTH 80.0

typedef struct {
    double x, y;
} Point;

double distance(Point a, Point b) {
    return hypot(b.x - a.x, b.y - a.y);
}

// 선분 a→b를 오른쪽으로 OFFSET 평행 이동한 점 반환
void offset_segment(Point a, Point b, double offset, Point* a_out, Point* b_out) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double length = hypot(dx, dy);
    double nx = -dy / length;
    double ny = dx / length;

    a_out->x = a.x + nx * offset;
    a_out->y = a.y + ny * offset;
    b_out->x = b.x + nx * offset;
    b_out->y = b.y + ny * offset;
}

int make_offset_path(Point* input, int input_len, Point* output, int* output_len) {
    double total_len = 0.0;
    int out_idx = 0;

    for (int i = 0; i < input_len - 1; ++i) {
        Point a = input[i];
        Point b = input[i + 1];
        double seg_len = distance(a, b);

        if (total_len + seg_len > TARGET_LENGTH) {
            double remain = TARGET_LENGTH - total_len;
            double ratio = remain / seg_len;

            Point mid = {
                a.x + (b.x - a.x) * ratio,
                a.y + (b.y - a.y) * ratio
            };

            Point oa, ob;
            offset_segment(a, mid, OFFSET, &oa, &ob);

            if (out_idx == 0) output[out_idx++] = oa;
            output[out_idx++] = ob;

            *output_len = out_idx;
            return 0;
        } else {
            Point oa, ob;
            offset_segment(a, b, OFFSET, &oa, &ob);
            if (out_idx == 0) output[out_idx++] = oa;
            output[out_idx++] = ob;

            total_len += seg_len;
        }
    }

    *output_len = out_idx;
    return 0;
}

void print_points(Point* pts, int len) {
    for (int i = 0; i < len; ++i) {
        printf("%.3f, %.3f\n", pts[i].x, pts[i].y);
    }
}

int main() {
    Point input[] = {
        {0, 0}, {40, 0}, {80, 20}, {120, 20}
    };
    int input_len = sizeof(input) / sizeof(Point);

    Point output[MAX_POINTS];
    int output_len = 0;

    make_offset_path(input, input_len, output, &output_len);
    print_points(output, output_len);

    return 0;
}