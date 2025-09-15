#include <stdio.h>
#include <math.h>

typedef struct {
    double x;
    double y;
} Point;

enum ShiftResult {
    SHIFT_OK = 0,
    SHIFT_DEGENERATE_A = 1,
    SHIFT_NOT_PARALLEL = 2
};

// 두 직선(A: pA1-pA2, B: pB1-pB2)을 검사하여
// A를 B와 일치하도록 평행이동한 새로운 A의 점을 npA1,npA2에 저장.
// 반환값: 0(성공), 1(A가 점으로 퇴화), 2(평행 아님)
int shift_lineA_to_lineB(Point pA1, Point pA2, Point pB1, Point pB2,
                         Point *npA1, Point *npA2, double eps) {
    // A의 방향 벡터
    double dxA = pA2.x - pA1.x;
    double dyA = pA2.y - pA1.y;
    // B의 방향 벡터
    double dxB = pB2.x - pB1.x;
    double dyB = pB2.y - pB1.y;

    // A가 점으로 퇴화했는지 검사
    if (fabs(dxA) < eps && fabs(dyA) < eps) {
        return SHIFT_DEGENERATE_A;
    }

    // 평행성 검사: 방향 벡터 외적 == 0 이면 평행
    double cross = dxA * dyB - dyA * dxB;
    if (fabs(cross) > eps) {
        return SHIFT_NOT_PARALLEL;
    }

    // A 직선의 계수 (a*x + b*y + c = 0)
    double a = pA1.y - pA2.y;
    double b = pA2.x - pA1.x;
    double c = pA1.x * pA2.y - pA2.x * pA1.y;

    double norm = sqrt(a*a + b*b);
    if (norm < eps) { // 보호 (사실 위에서 검사했지만 안전하게)
        return SHIFT_DEGENERATE_A;
    }

    // B의 한 점을 A 식에 넣어 signed distance 계산
    double signed_dist = (a * pB1.x + b * pB1.y + c) / norm;

    // 이동벡터 t = signed_dist * (a,b)/norm
    double tx = signed_dist * (a / norm);
    double ty = signed_dist * (b / norm);

    // A의 두 점 이동
    npA1->x = pA1.x + tx;
    npA1->y = pA1.y + ty;
    npA2->x = pA2.x + tx;
    npA2->y = pA2.y + ty;

    return SHIFT_OK;
}

int main(void) {
    // 예시: A: (0,0)-(4,0)  (y=0)
    //       B: (0,2)-(1,2)  (y=2)
    Point A1 = {0.0, 0.0};
    Point A2 = {4.0, 0.0};
    Point B1 = {0.0, 2.0};
    Point B2 = {1.0, 2.0};

    Point nA1, nA2;
    int r = shift_lineA_to_lineB(A1, A2, B1, B2, &nA1, &nA2, 1e-12);

    if (r == SHIFT_OK) {
        printf("평행이동 성공.\n");
        printf("이동된 A의 점: (%.10f, %.10f) ~ (%.10f, %.10f)\n",
               nA1.x, nA1.y, nA2.x, nA2.y);
    } else if (r == SHIFT_DEGENERATE_A) {
        printf("오류: 직선 A가 두 동일한 점으로 퇴화했습니다.\n");
    } else if (r == SHIFT_NOT_PARALLEL) {
        printf("오류: 직선 A와 B는 평행이 아닙니다. (평행여부를 확인하세요)\n");
    }

    return 0;
}