#include <stdio.h>
#include <math.h>

/* =========================
 *  기본 정의
 * ========================= */

#define EPS 1e-6f

typedef struct {
    float f32_X;
    float f32_Y;
} Point;

/* =========================
 *  벡터 유틸 함수
 * ========================= */

static Point point_add(Point a, Point b)
{
    Point r;
    r.f32_X = a.f32_X + b.f32_X;
    r.f32_Y = a.f32_Y + b.f32_Y;
    return r;
}

static Point point_sub(Point a, Point b)
{
    Point r;
    r.f32_X = a.f32_X - b.f32_X;
    r.f32_Y = a.f32_Y - b.f32_Y;
    return r;
}

static Point point_mul(Point a, float t)
{
    Point r;
    r.f32_X = a.f32_X * t;
    r.f32_Y = a.f32_Y * t;
    return r;
}

static float cross(Point a, Point b)
{
    return a.f32_X * b.f32_Y - a.f32_Y * b.f32_X;
}

/* =========================
 *  선분-선분 교차
 * ========================= */

static int segment_intersect(
    Point p, Point p2,
    Point q, Point q2,
    Point* out)
{
    Point r = point_sub(p2, p);
    Point s = point_sub(q2, q);

    float rxs = cross(r, s);
    float q_pxr = cross(point_sub(q, p), r);

    /* 평행 or 일치 */
    if (fabsf(rxs) < EPS) {
        return 0;
    }

    float t = cross(point_sub(q, p), s) / rxs;
    float u = q_pxr / rxs;

    if ((t >= 0.0f) && (t <= 1.0f) &&
        (u >= 0.0f) && (u <= 1.0f))
    {
        if (out != NULL) {
            *out = point_add(p, point_mul(r, t));
        }
        return 1;
    }

    return 0;
}

/* =========================
 *  직선-선분 교차 (T 연장)
 * ========================= */

static int line_segment_intersect(
    Point p, Point p2,
    Point q, Point q2,
    Point* out)
{
    Point r = point_sub(p2, p);
    Point s = point_sub(q2, q);

    float rxs = cross(r, s);

    if (fabsf(rxs) < EPS) {
        return 0;
    }

    float t = cross(point_sub(q, p), s) / rxs;
    float u = cross(point_sub(q, p), r) / rxs;

    /* t: 제한 없음 (무한 직선)
       u: 선분 내부 */
    if ((u >= 0.0f) && (u <= 1.0f)) {
        if (out != NULL) {
            *out = point_add(p, point_mul(r, t));
        }
        return 1;
    }

    return 0;
}

/* =========================
 *  T vs Polyline 교차
 * ========================= */

int intersect_T_with_polyline(
    const Point* C, int C_size,
    const Point T[2],
    Point* out_intersection,
    int* is_extended)
{
    int i;

    if ((C == NULL) || (C_size < 2)) {
        return 0;
    }

    /* 1. 선분 T vs Polyline */
    for (i = 0; i < C_size - 1; i++) {
        if (segment_intersect(
                T[0], T[1],
                C[i], C[i + 1],
                out_intersection))
        {
            if (is_extended != NULL) {
                *is_extended = 0;
            }
            return 1;
        }
    }

    /* 2. 연장된 직선 T vs Polyline */
    for (i = 0; i < C_size - 1; i++) {
        if (line_segment_intersect(
                T[0], T[1],
                C[i], C[i + 1],
                out_intersection))
        {
            if (is_extended != NULL) {
                *is_extended = 1;
            }
            return 1;
        }
    }

    return 0;
}

/* =========================
 *  테스트용 main
 * ========================= */

int main(void)
{
    Point C[5] = {
        {0.0f, 0.0f},
        {5.0f, 0.0f},
        {5.0f, 5.0f},
        {0.0f, 5.0f},
        {0.0f, 10.0f}
    };

    Point T[2] = {
        {-2.0f, 2.5f},
        {-1.0f, 2.5f}
    };

    Point intersection;
    int is_extended = 0;

    if (intersect_T_with_polyline(
            C, 5, T,
            &intersection,
            &is_extended))
    {
        printf("Intersection at (%.3f, %.3f)\n",
               intersection.f32_X,
               intersection.f32_Y);

        if (is_extended) {
            printf("→ Found by extending T\n");
        } else {
            printf("→ Found within T segment\n");
        }
    } else {
        printf("No intersection found\n");
    }

    return 0;
}