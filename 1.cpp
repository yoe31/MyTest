// lateral_distance.c
// Compile: gcc -O2 -o lateral_distance lateral_distance.c -lm
// Run: ./lateral_distance

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
    double x;
    double y;
} Point;

// result for each query point
typedef struct {
    double signed_dist; // positive => left of segment direction, negative => right
    int seg_idx;        // index of segment in B that was closest (0..M-2), -1 if none
    Point proj;         // projection point on that segment (or closest endpoint)
} LateralResult;

// compute squared distance between two points
static double dist2(const Point *a, const Point *b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    return dx*dx + dy*dy;
}

// dot product
static double dot(double ax, double ay, double bx, double by) {
    return ax*bx + ay*by;
}

// cross product z component (2D)
static double cross(double ax, double ay, double bx, double by) {
    return ax*by - ay*bx;
}

// project point p onto segment s0->s1
// returns projection point in "proj", and t parameter (0..1 if within segment)
// if t<0 or t>1 projection is outside — proj will be endpoint (s0 or s1)
static double project_point_to_segment(const Point *p, const Point *s0, const Point *s1, Point *proj) {
    double vx = s1->x - s0->x;
    double vy = s1->y - s0->y;
    double wx = p->x - s0->x;
    double wy = p->y - s0->y;
    double vlen2 = vx*vx + vy*vy;
    if (vlen2 == 0.0) {
        // segment degenerate -> treat as s0
        proj->x = s0->x;
        proj->y = s0->y;
        return 0.0;
    }
    double t = dot(wx, wy, vx, vy) / vlen2;
    if (t <= 0.0) {
        proj->x = s0->x;
        proj->y = s0->y;
    } else if (t >= 1.0) {
        proj->x = s1->x;
        proj->y = s1->y;
    } else {
        proj->x = s0->x + t * vx;
        proj->y = s0->y + t * vy;
    }
    return t;
}

// compute signed lateral distance from point p to segment s0->s1
// positive if p is to the left of segment direction s0->s1
static double signed_distance_point_to_segment(const Point *p, const Point *s0, const Point *s1, Point *proj_out) {
    Point proj;
    double t = project_point_to_segment(p, s0, s1, &proj);
    if (proj_out) *proj_out = proj;
    double dx = p->x - proj.x;
    double dy = p->y - proj.y;
    double dist = sqrt(dx*dx + dy*dy);

    // determine sign: use cross product of segment direction and (p - proj).
    // segment direction:
    double sx = s1->x - s0->x;
    double sy = s1->y - s0->y;
    double c = cross(sx, sy, dx, dy);
    // If c > 0 => left, c < 0 => right.
    if (c > 0.0) return dist;
    if (c < 0.0) return -dist;
    return 0.0;
}

// For each point in A (N points), find nearest segment in polyline B (M points => M-1 segments),
// compute signed lateral distance and fill results array (length N).
// Returns 0 on success, non-zero on invalid input.
int compute_lateral_differences(
    const Point *A, int N,
    const Point *B, int M,
    LateralResult *out_results // must be allocated with size >= N
) {
    if (!A || !B || !out_results) return -1;
    if (N <= 0 || M <= 1) return -2;

    for (int i = 0; i < N; ++i) {
        const Point *p = &A[i];
        double best_d2 = 1e300;
        int best_seg = -1;
        Point best_proj = {0.0, 0.0};
        double best_signed = 0.0;

        // check all segments in B
        for (int s = 0; s < M - 1; ++s) {
            Point proj;
            project_point_to_segment(p, &B[s], &B[s+1], &proj);
            double d2 = dist2(p, &proj);
            if (d2 < best_d2) {
                best_d2 = d2;
                best_seg = s;
                best_proj = proj;
            }
        }
        if (best_seg >= 0) {
            double sd = signed_distance_point_to_segment(p, &B[best_seg], &B[best_seg+1], &best_proj);
            out_results[i].signed_dist = sd;
            out_results[i].seg_idx = best_seg;
            out_results[i].proj = best_proj;
        } else {
            // fallback (shouldn't happen if M>1)
            out_results[i].signed_dist = 0.0;
            out_results[i].seg_idx = -1;
            out_results[i].proj = *p;
        }
    }
    return 0;
}

// helper to compute summary stats
void compute_stats(const LateralResult *res, int N, double *mean, double *rms, double *maxabs) {
    double sum = 0.0;
    double sqsum = 0.0;
    double mabs = 0.0;
    for (int i = 0; i < N; ++i) {
        double v = res[i].signed_dist;
        sum += v;
        sqsum += v*v;
        double ab = fabs(v);
        if (ab > mabs) mabs = ab;
    }
    *mean = sum / N;
    *rms = sqrt(sqsum / N);
    *maxabs = mabs;
}

// simple demo / test
int main(void) {
    // Example: polyline A (N=5)
    Point A[] = {
        {0.0, 0.0},
        {10.0, 0.5},
        {20.0, 1.0},
        {30.0, 1.5},
        {40.0, 2.0}
    };
    int N = sizeof(A)/sizeof(A[0]);

    // Example: polyline B (M=4) - roughly parallel to A but offset to left by 1.0
    Point B[] = {
        {0.0, 1.0},
        {20.0, 1.5},
        {40.0, 2.0},
        {60.0, 2.5}
    };
    int M = sizeof(B)/sizeof(B[0]);

    LateralResult *res = (LateralResult*)malloc(sizeof(LateralResult) * N);
    if (!res) { fprintf(stderr, "malloc failed\n"); return 1; }

    int rc = compute_lateral_differences(A, N, B, M, res);
    if (rc != 0) {
        fprintf(stderr, "compute failed: %d\n", rc);
        free(res);
        return 1;
    }

    printf("point_idx\tAx\tAy\tsigned_lat_dist\tseg_idx\tproj_x\tproj_y\n");
    for (int i = 0; i < N; ++i) {
        printf("%d\t\t%.3f\t%.3f\t%.6f\t\t%d\t%.3f\t%.3f\n",
            i, A[i].x, A[i].y, res[i].signed_dist, res[i].seg_idx, res[i].proj.x, res[i].proj.y);
    }

    double mean, rms, maxabs;
    compute_stats(res, N, &mean, &rms, &maxabs);
    printf("\nSummary: mean=%.6f, rms=%.6f, maxabs=%.6f\n", mean, rms, maxabs);

    free(res);
    return 0;
}