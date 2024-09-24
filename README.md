# MyTest


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define DEGREE 3                 // B-Spline 곡선의 차수 (자유롭게 조정 가능)
#define MAX_CONTROL_POINTS 100   // 제어점의 최대 개수
#define NUM_RESULT_POINTS 200    // 결과 곡선의 점 개수

// Point 구조체 정의
typedef struct {
    double x;
    double y;
} Point;

// Knot 벡터를 생성하는 함수
void generateKnots(double* knots, int numControlPoints, int degree) {
    int numKnots = numControlPoints + degree + 1;
    for (int i = 0; i < numKnots; i++) {
        if (i <= degree) knots[i] = 0.0;
        else if (i >= numKnots - degree - 1) knots[i] = 1.0;
        else knots[i] = (double)(i - degree) / (numKnots - 2 * degree - 1);
    }
}

// De Boor 알고리즘을 반복적으로 사용하여 B-Spline 곡선을 계산하는 함수
Point deBoorIterative(int degree, int i, double t, double* knots, Point* controlPoints) {
    Point d[DEGREE + 1]; // degree + 1 크기의 임시 배열
    for (int j = 0; j <= degree; j++) {
        d[j] = controlPoints[i - degree + j];
    }

    for (int r = 1; r <= degree; r++) {
        for (int j = degree; j >= r; j--) {
            double alpha = (t - knots[i - degree + j]) / (knots[i + 1 + j - r] - knots[i - degree + j]);
            d[j].x = (1.0 - alpha) * d[j - 1].x + alpha * d[j].x;
            d[j].y = (1.0 - alpha) * d[j - 1].y + alpha * d[j].y;
        }
    }
    return d[degree];
}

// B-Spline Curve를 계산하여 결과를 저장하는 함수
void calculateBSpline(Point* controlPoints, int numControlPoints, Point* resultPoints, int numResultPoints, int degree) {
    int numKnots = numControlPoints + degree + 1;
    double knots[numKnots];
    generateKnots(knots, numControlPoints, degree);

    double step = 1.0 / (numResultPoints - 1);
    for (int i = 0; i < numResultPoints; i++) {
        double t = i * step;
        if (t == 1.0) {  // 마지막 값을 처리하기 위한 보정
            t -= 1e-10;
        }

        // Knot 벡터에서 t가 포함될 수 있는 segment를 찾음
        int startIdx = degree;
        while (startIdx < numControlPoints && t > knots[startIdx + 1]) {
            startIdx++;
        }

        resultPoints[i] = deBoorIterative(degree, startIdx, t, knots, controlPoints);
    }
}

// 점 배열을 출력하는 함수
void printPoints(Point* points, int count) {
    for (int i = 0; i < count; i++) {
        printf("Point %d: (%.2f, %.2f)\n", i, points[i].x, points[i].y);
    }
}

int main() {
    // 샘플 제어점 (실제 데이터에서는 필터링된 제어점을 입력)
    Point controlPoints[MAX_CONTROL_POINTS] = {
        {112.55, 0.89}, {111.20, 0.57}, {109.73, -0.06}, {108.43, -0.98},
        {107.18, -2.72}, {106.51, -5.02}, {106.39, -7.42}, {106.05, -5.01},
        {105.64, -7.36}, {104.5, -8.0}, {103.0, -9.0}  // 필요한 만큼 점 추가
    };
    int numControlPoints = 11;  // 실제 사용되는 제어점의 개수

    Point resultPoints[NUM_RESULT_POINTS];

    // B-Spline 계산
    calculateBSpline(controlPoints, numControlPoints, resultPoints, NUM_RESULT_POINTS, DEGREE);

    // 결과 출력
    printPoints(resultPoints, NUM_RESULT_POINTS);

    return 0;
}
