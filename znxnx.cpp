#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define MAX_POINTS 1000
#define MAX_ITER 1000
#define THRESHOLD 1.0

typedef struct {
    double x;
    double y;
} Point;

typedef struct {
    double a, b, c; // x = a*y^2 + b*y + c
} QuadModel;

// 3개의 점으로부터 2차 방정식 피팅
int fit_quadratic_model(Point p1, Point p2, Point p3, QuadModel* model) {
    double A[3][4] = {
        {p1.y * p1.y, p1.y, 1.0, p1.x},
        {p2.y * p2.y, p2.y, 1.0, p2.x},
        {p3.y * p3.y, p3.y, 1.0, p3.x}
    };

    // 가우스 소거법으로 a, b, c 구하기
    for (int i = 0; i < 3; i++) {
        // 피벗이 너무 작으면 실패
        if (fabs(A[i][i]) < 1e-6) return 0;

        for (int j = i + 1; j < 3; j++) {
            double r = A[j][i] / A[i][i];
            for (int k = i; k < 4; k++) {
                A[j][k] -= r * A[i][k];
            }
        }
    }

    // 역연산
    double x[3];
    for (int i = 2; i >= 0; i--) {
        x[i] = A[i][3];
        for (int j = i + 1; j < 3; j++) {
            x[i] -= A[i][j] * x[j];
        }
        x[i] /= A[i][i];
    }

    model->a = x[0];
    model->b = x[1];
    model->c = x[2];
    return 1;
}

// 예측 x값 계산
double predict_x(QuadModel model, double y) {
    return model.a * y * y + model.b * y + model.c;
}

// RANSAC 핵심 로직
void ransac_quadratic(Point* points, int num_points, QuadModel* best_model, int* best_inliers, int* best_inlier_count) {
    int max_inliers = 0;
    srand(time(NULL));

    for (int iter = 0; iter < MAX_ITER; iter++) {
        // 서로 다른 3개 점 선택
        int i1 = rand() % num_points;
        int i2, i3;
        do { i2 = rand() % num_points; } while (i2 == i1);
        do { i3 = rand() % num_points; } while (i3 == i1 || i3 == i2);

        QuadModel model;
        if (!fit_quadratic_model(points[i1], points[i2], points[i3], &model)) continue;

        int inliers[MAX_POINTS];
        int inlier_count = 0;

        for (int i = 0; i < num_points; i++) {
            double x_pred = predict_x(model, points[i].y);
            double error = fabs(x_pred - points[i].x);
            if (error < THRESHOLD) {
                inliers[inlier_count++] = i;
            }
        }

        if (inlier_count > max_inliers) {
            *best_model = model;
            *best_inlier_count = inlier_count;
            for (int k = 0; k < inlier_count; k++) {
                best_inliers[k] = inliers[k];
            }
            max_inliers = inlier_count;
        }
    }
}

// 예제 main
int main() {
    Point points[MAX_POINTS] = {
        {0, 0}, {1, 1}, {4, 2}, {9, 3}, {16, 4},
        {25, 5}, {36, 6}, {49, 7}, {64, 8}, {81, 9}, // 대략 x = y^2
        {50, -2}, {60, -3} // outlier
    };
    int num_points = 12;

    QuadModel best_model;
    int best_inliers[MAX_POINTS];
    int best_inlier_count = 0;

    ransac_quadratic(points, num_points, &best_model, best_inliers, &best_inlier_count);

    printf("Best model: x = %.3f * y^2 + %.3f * y + %.3f\n", best_model.a, best_model.b, best_model.c);
    printf("Inliers (%d): ", best_inlier_count);
    for (int i = 0; i < best_inlier_count; i++) {
        printf("%d ", best_inliers[i]);
    }
    printf("\n");

    return 0;
}