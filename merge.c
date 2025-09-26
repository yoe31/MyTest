// filename: lane_merge.c
// Compile: gcc lane_merge.c -lm -o lane_merge
// Run: ./lane_merge
// 설명: 예시 프레임 데이터를 사용하여
// 1) 각 인식 차선을 weighted PCA로 직선 적합 (normal form r,theta)
// 2) 간단한 공분산 근사 계산 (논문 수식의 근사/휴리스틱)
// 3) 프레임간 DR(DeltaX,DeltaY,DeltaH)을 누적하여 글로벌 pose 계산
// 4) 각 프레임의 라인을 글로벌로 변환(선 파라미터 변환 & 공분산 전파)
// 5) Mahalanobis(chi2) 기준으로 기존 맵의 라인과 병합(정보 행렬 합산 방식)
// 6) 프레임별 병합 후 맵을 출력
//
// 주의: 일부 노이즈/가중치/공분산 전파는 실용적 근사(=추측입니다)로 구현했습니다.

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define MAX_POINTS 20
#define MAX_LANES   30
#define MAX_FRAMES  10
#define MAX_MAP_LINES 200
#define PI 3.14159265358979323846

// --- 데이터 구조 ---
typedef struct {
    double dx; // forward +X (m)
    double dy; // left +Y (m)
    double dh; // heading change (rad) (반시계+)
} DR;

typedef struct {
    uint8_t NumOfPoints;
    float Points[MAX_POINTS][2]; // 0:X, 1:Y  (local vehicle frame)
} LaneLineInfo;

typedef struct {
    uint8_t NumOfLane;
    LaneLineInfo lane_line_info_list[MAX_LANES];
} FrameObs;

// Line parameter in normal form: r = x*cos(theta) + y*sin(theta)
// store covariance of [r, theta] (2x2)
typedef struct {
    double r;
    double theta; // normalized to [-pi,pi)
    double cov[2][2];
    int id;       // map id
    int count;    // how many segments merged
} LineParam;

typedef struct {
    double x;
    double y;
    double theta;
} Pose;

// --- 유틸 ---
static double normalize_angle(double a) {
    while (a <= -PI) a += 2*PI;
    while (a > PI) a -= 2*PI;
    return a;
}

static void ensure_positive_r_theta(double *r, double *theta) {
    // make r >= 0 by flipping sign if needed (and rotate theta by pi)
    if (*r < 0) {
        *r = -(*r);
        *theta = normalize_angle((*theta) + PI);
    } else {
        *theta = normalize_angle(*theta);
    }
}

// invert 2x2 matrix A into invA; returns 0 if failed (near-singular)
static int invert_2x2(double A[2][2], double invA[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    double eps = 1e-9;
    if (fabs(det) < eps) return 0;
    invA[0][0] =  A[1][1] / det;
    invA[1][1] =  A[0][0] / det;
    invA[0][1] = -A[0][1] / det;
    invA[1][0] = -A[1][0] / det;
    return 1;
}

// add small diagonal regularization
static void regularize_2x2(double A[2][2], double eps) {
    A[0][0] += eps;
    A[1][1] += eps;
}

// multiply 2x2 * 2x1
static void mat2_mul_vec2(double M[2][2], double v[2], double out[2]) {
    out[0] = M[0][0]*v[0] + M[0][1]*v[1];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1];
}

// --- 가중 직선 적합 (PCA 기반) ---
// 입력: lane (points in vehicle frame)
// 출력: r,theta, cov(2x2) 근사
// 참고: 가중치는 거리 기반의 간단한 센서 노이즈 가정 사용 (추측입니다).
static int fit_weighted_line_pca(const LaneLineInfo *lane, double *out_r, double *out_theta, double cov_out[2][2]) {
    int n = lane->NumOfPoints;
    if (n < 2) return 0;

    double sum_w = 0.0;
    double cx = 0.0, cy = 0.0;

    // heuristic sensor noise model: sigma_range = base + k * range  (추측입니다)
    const double sigma_base = 0.05; // 5 cm base noise (추측)
    const double sigma_k = 0.01;    // noise grows 1cm per meter (추측)

    double weights[MAX_POINTS];
    for (int i=0;i<n;i++) {
        double x = lane->Points[i][0];
        double y = lane->Points[i][1];
        double range = sqrt(x*x + y*y);
        double sigma = sigma_base + sigma_k * range;
        double w = 1.0 / (sigma*sigma + 1e-12);
        weights[i] = w;
        sum_w += w;
        cx += w * x;
        cy += w * y;
    }
    if (sum_w <= 0) return 0;
    cx /= sum_w;
    cy /= sum_w;

    // weighted covariance matrix (Sxx,Syy,Sxy)
    double Sxx=0.0, Syy=0.0, Sxy=0.0;
    for (int i=0;i<n;i++) {
        double x = lane->Points[i][0] - cx;
        double y = lane->Points[i][1] - cy;
        double w = weights[i];
        Sxx += w * x * x;
        Syy += w * y * y;
        Sxy += w * x * y;
    }
    // normalize by sum_w to get covariance
    Sxx /= sum_w;
    Syy /= sum_w;
    Sxy /= sum_w;

    // PCA eigen decomposition (2x2)
    double trace = Sxx + Syy;
    double diff = Sxx - Syy;
    double sq = sqrt(diff*diff + 4*Sxy*Sxy);
    double lambda1 = 0.5*(trace + sq); // larger (along-line variance)
    double lambda2 = 0.5*(trace - sq); // smaller (perp variance)

    // direction (principal axis) angle
    double dir = 0.5 * atan2(2*Sxy, diff); // direction of largest variance
    // normal vector angle (theta): perpendicular to direction
    double theta = normalize_angle(dir + PI/2.0);

    double r = cx * cos(theta) + cy * sin(theta);
    ensure_positive_r_theta(&r, &theta);

    // Covariance approximation (heuristic / simplified)
    // - var_r ~ lambda2 / N_eff  (perp-variance averaged)
    // - var_theta ~ lambda2 / (N_eff * (lambda1 + eps))  (더 긴 선 -> 방향 추정 더 정확)
    // 이 식들은 논문의 closed-form을 대체하는 실용적 근사입니다. (추측입니다)
    double Neff = sum_w; // effective sample size (weights)
    double eps = 1e-6;
    double var_r = fabs(lambda2) / (Neff + eps);
    double var_theta = fabs(lambda2) / ((Neff * (fabs(lambda1) + eps)) + eps);

    // protect from tiny numbers
    if (var_r < 1e-8) var_r = 1e-8;
    if (var_theta < 1e-8) var_theta = 1e-8;

    cov_out[0][0] = var_r; cov_out[0][1] = 0.0;
    cov_out[1][0] = 0.0;  cov_out[1][1] = var_theta;

    *out_r = r;
    *out_theta = theta;
    return 1;
}

// --- 라인 파라미터를 로컬->글로벌로 변환 (pose : global pose of vehicle at time of detection) ---
// p_local: (r_local, theta_local) in vehicle frame
// p_global: r_global = r_local + x*cos(theta_global) + y*sin(theta_global)
//            theta_global = theta_local + pose.theta
// Covariance 전파: J * Cov_local * J^T  (여기서는 pose covariance는 무시함, 즉 pose는 정확하다고 가정합니다. 추측입니다.)
static void transform_line_to_global(const LineParam *local, const Pose *pose, LineParam *global_out) {
    double theta_global = normalize_angle(local->theta + pose->theta);
    double r_global = local->r + pose->x * cos(theta_global) + pose->y * sin(theta_global);

    // Jacobian J = [ dr/dr_local , dr/dtheta_local ]
    //              [ dtheta/dr_local , dtheta/dtheta_local ]
    // dr/dr_local = 1
    // dr/dtheta_local = d/dtheta_local [ x*cos(theta_local+pose.theta) + y*sin(...) ] = -x*sin(theta_global) + y*cos(theta_global)
    double ddr_dtheta = -pose->x * sin(theta_global) + pose->y * cos(theta_global);

    // dtheta/dr_local = 0
    // dtheta/dtheta_local = 1
    // J = [1, ddr_dtheta; 0, 1]
    double J[2][2] = {{1.0, ddr_dtheta},{0.0, 1.0}};

    // Cov_global = J * Cov_local * J^T
    double temp[2][2] = { {0,0},{0,0} };
    // temp = J * Cov_local
    for (int i=0;i<2;i++) for (int j=0;j<2;j++) {
        temp[i][j] = 0.0;
        for (int k=0;k<2;k++) temp[i][j] += J[i][k] * local->cov[k][j];
    }
    double covg[2][2] = {{0,0},{0,0}};
    for (int i=0;i<2;i++) for (int j=0;j<2;j++) {
        covg[i][j] = 0.0;
        for (int k=0;k<2;k++) covg[i][j] += temp[i][k] * J[j][k];
    }
    // numeric regularization
    regularize_2x2(covg, 1e-9);

    global_out->r = r_global;
    global_out->theta = theta_global;
    global_out->cov[0][0] = covg[0][0];
    global_out->cov[0][1] = covg[0][1];
    global_out->cov[1][0] = covg[1][0];
    global_out->cov[1][1] = covg[1][1];
    global_out->id = local->id;
    global_out->count = local->count;
}

// --- chi2 distance between two lines (p = [r,theta]) ---
// chi2 = (p1 - p2)^T * inv(S1+S2) * (p1 - p2)
static double line_chi2(const LineParam *a, const LineParam *b) {
    double da[2];
    da[0] = a->r - b->r;
    double dtheta = normalize_angle(a->theta - b->theta);
    da[1] = dtheta;

    double S[2][2];
    S[0][0] = a->cov[0][0] + b->cov[0][0];
    S[0][1] = a->cov[0][1] + b->cov[0][1];
    S[1][0] = a->cov[1][0] + b->cov[1][0];
    S[1][1] = a->cov[1][1] + b->cov[1][1];

    // regularize small values
    regularize_2x2(S, 1e-9);

    double invS[2][2];
    if (!invert_2x2(S, invS)) {
        // near singular => return large distance
        return 1e9;
    }

    double tmp[2];
    mat2_mul_vec2(invS, da, tmp);
    double chi2 = da[0]*tmp[0] + da[1]*tmp[1];
    return chi2;
}

// --- merge two lines using information form (Kalman-like fusion) ---
// p_merged = (S1^-1 + S2^-1)^-1 (S1^-1 p1 + S2^-1 p2)
static void merge_lines(const LineParam *a, const LineParam *b, LineParam *out) {
    double S1[2][2] = {{a->cov[0][0], a->cov[0][1]}, {a->cov[1][0], a->cov[1][1]}};
    double S2[2][2] = {{b->cov[0][0], b->cov[0][1]}, {b->cov[1][0], b->cov[1][1]}};
    regularize_2x2(S1, 1e-9);
    regularize_2x2(S2, 1e-9);

    double invS1[2][2], invS2[2][2];
    int ok1 = invert_2x2(S1, invS1);
    int ok2 = invert_2x2(S2, invS2);
    if (!ok1 || !ok2) {
        // fallback: simple average
        out->r = 0.5*(a->r + b->r);
        out->theta = normalize_angle(0.5*(a->theta + b->theta));
        out->cov[0][0] = 0.5*(a->cov[0][0] + b->cov[0][0]);
        out->cov[0][1] = 0.0;
        out->cov[1][0] = 0.0;
        out->cov[1][1] = 0.5*(a->cov[1][1] + b->cov[1][1]);
        out->count = a->count + b->count;
        return;
    }

    // Info = invS1 + invS2
    double Info[2][2];
    for (int i=0;i<2;i++) for (int j=0;j<2;j++) Info[i][j] = invS1[i][j] + invS2[i][j];

    double invInfo[2][2];
    if (!invert_2x2(Info, invInfo)) {
        // numeric fallback
        out->r = 0.5*(a->r + b->r);
        out->theta = normalize_angle(0.5*(a->theta + b->theta));
        out->cov[0][0] = 0.5*(a->cov[0][0] + b->cov[0][0]);
        out->cov[1][1] = 0.5*(a->cov[1][1] + b->cov[1][1]);
        out->cov[0][1]=out->cov[1][0]=0.0;
        out->count = a->count + b->count;
        return;
    }

    // rhs = invS1 * p1 + invS2 * p2
    double p1[2] = {a->r, a->theta};
    double p2[2] = {b->r, b->theta};
    double rhs[2] = {0,0};
    double tmp[2];
    mat2_mul_vec2(invS1, p1, tmp);
    rhs[0] += tmp[0]; rhs[1] += tmp[1];
    mat2_mul_vec2(invS2, p2, tmp);
    rhs[0] += tmp[0]; rhs[1] += tmp[1];

    double pmerged[2];
    mat2_mul_vec2(invInfo, rhs, pmerged);

    out->r = pmerged[0];
    out->theta = normalize_angle(pmerged[1]);
    // cov merged = inv( invS1 + invS2 ) = invInfo
    out->cov[0][0] = invInfo[0][0];
    out->cov[0][1] = invInfo[0][1];
    out->cov[1][0] = invInfo[1][0];
    out->cov[1][1] = invInfo[1][1];

    out->count = a->count + b->count;
    // keep id of 'a' (map entry) by convention
    out->id = a->id;
}

// --- MAP 저장소 ---
static LineParam map_lines[MAX_MAP_LINES];
static int map_line_count = 0;
static int next_map_id = 1;

// 매 프레임 변형된 라인(global) 을 맵에 병합
// threshold : chi2 threshold (chi-square DOF=2)
// 95% -> 5.991, 99% -> 9.210
static void merge_into_map(const LineParam *global_line, double chi2_thresh) {
    // find best match
    double best_chi2 = 1e9;
    int best_idx = -1;
    for (int i=0;i<map_line_count;i++) {
        double c = line_chi2(global_line, &map_lines[i]);
        if (c < best_chi2) { best_chi2 = c; best_idx = i; }
    }
    if (best_idx >=0 && best_chi2 < chi2_thresh) {
        // merge
        LineParam merged;
        merge_lines(&map_lines[best_idx], global_line, &merged);
        merged.id = map_lines[best_idx].id; // keep id
        map_lines[best_idx] = merged;
    } else {
        // add new map line
        if (map_line_count < MAX_MAP_LINES) {
            LineParam newl = *global_line;
            newl.id = next_map_id++;
            if (newl.count <= 0) newl.count = 1;
            map_lines[map_line_count++] = newl;
        } else {
            // map full - ignore
        }
    }
}

// --- 예시/테스트 데이터 (3 프레임 예시) ---
// 각 프레임: DR (delta motion from previous frame), 그리고 인식된 lane 라인들 (local frame)
static DR sample_dr[MAX_FRAMES] = {
    {0.0, 0.0, 0.0}, // 첫 프레임은 이동 없음
    {1.0, 0.0, 0.0}, // 차량이 +X (앞)으로 1m 이동
    {1.0, 0.0, 0.0},
};

static FrameObs sample_frames[MAX_FRAMES];

static void prepare_sample_frames() {
    memset(sample_frames,0,sizeof(sample_frames));
    // Frame 0: two lanes (approx x=10 and x=20 ahead)
    sample_frames[0].NumOfLane = 2;
    // lane 0: x ~ 10, y = -3,0,3
    sample_frames[0].lane_line_info_list[0].NumOfPoints = 3;
    sample_frames[0].lane_line_info_list[0].Points[0][0] = 10.0; sample_frames[0].lane_line_info_list[0].Points[0][1] = -3.0;
    sample_frames[0].lane_line_info_list[0].Points[1][0] = 10.0; sample_frames[0].lane_line_info_list[0].Points[1][1] = 0.0;
    sample_frames[0].lane_line_info_list[0].Points[2][0] = 10.0; sample_frames[0].lane_line_info_list[0].Points[2][1] = 3.0;
    // lane 1: x ~ 20
    sample_frames[0].lane_line_info_list[1].NumOfPoints = 3;
    sample_frames[0].lane_line_info_list[1].Points[0][0] = 20.0; sample_frames[0].lane_line_info_list[1].Points[0][1] = -3.0;
    sample_frames[0].lane_line_info_list[1].Points[1][0] = 20.0; sample_frames[0].lane_line_info_list[1].Points[1][1] = 0.0;
    sample_frames[0].lane_line_info_list[1].Points[2][0] = 20.0; sample_frames[0].lane_line_info_list[1].Points[2][1] = 3.0;

    // Frame1: robot moved forward 1m, so same global lane appears at local x ~ 9 and ~19 (plus small noise)
    sample_frames[1].NumOfLane = 2;
    sample_frames[1].lane_line_info_list[0].NumOfPoints = 3;
    sample_frames[1].lane_line_info_list[0].Points[0][0] = 9.05; sample_frames[1].lane_line_info_list[0].Points[0][1] = -3.1;
    sample_frames[1].lane_line_info_list[0].Points[1][0] = 8.95; sample_frames[1].lane_line_info_list[0].Points[1][1] = 0.05;
    sample_frames[1].lane_line_info_list[0].Points[2][0] = 9.02; sample_frames[1].lane_line_info_list[0].Points[2][1] = 3.0;
    sample_frames[1].lane_line_info_list[1].NumOfPoints = 3;
    sample_frames[1].lane_line_info_list[1].Points[0][0] = 19.1; sample_frames[1].lane_line_info_list[1].Points[0][1] = -2.9;
    sample_frames[1].lane_line_info_list[1].Points[1][0] = 19.0; sample_frames[1].lane_line_info_list[1].Points[1][1] = 0.1;
    sample_frames[1].lane_line_info_list[1].Points[2][0] = 19.02; sample_frames[1].lane_line_info_list[1].Points[2][1] = 2.95;

    // Frame2: forward another 1m
    sample_frames[2].NumOfLane = 2;
    sample_frames[2].lane_line_info_list[0].NumOfPoints = 3;
    sample_frames[2].lane_line_info_list[0].Points[0][0] = 8.1; sample_frames[2].lane_line_info_list[0].Points[0][1] = -3.05;
    sample_frames[2].lane_line_info_list[0].Points[1][0] = 8.0; sample_frames[2].lane_line_info_list[0].Points[1][1] = 0.08;
    sample_frames[2].lane_line_info_list[0].Points[2][0] = 8.05; sample_frames[2].lane_line_info_list[0].Points[2][1] = 3.02;
    sample_frames[2].lane_line_info_list[1].NumOfPoints = 3;
    sample_frames[2].lane_line_info_list[1].Points[0][0] = 18.2; sample_frames[2].lane_line_info_list[1].Points[0][1] = -3.0;
    sample_frames[2].lane_line_info_list[1].Points[1][0] = 18.05; sample_frames[2].lane_line_info_list[1].Points[1][1] = 0.05;
    sample_frames[2].lane_line_info_list[1].Points[2][0] = 18.1; sample_frames[2].lane_line_info_list[1].Points[2][1] = 3.1;
}

// --- 프레임 처리 루프 ---
static void process_all_frames(int num_frames) {
    Pose pose = {0.0, 0.0, 0.0}; // global pose (x,y,theta)
    map_line_count = 0;
    next_map_id = 1;

    const double CHI2_THRESH = 5.991; // 95% for dof=2 (설정 가능, 추측입니다)

    for (int f=0; f<num_frames; f++) {
        printf("\n=== Frame %d ===\n", f);
        DR d = sample_dr[f];
        // integrate DR to update pose (assumption: DR describes motion since previous frame -> we first update pose)
        // transform local delta to global: [dxg, dyg] = R(pose.theta) * [d.dx, d.dy]
        double c = cos(pose.theta), s = sin(pose.theta);
        double dxg = c * d.dx - s * d.dy;
        double dyg = s * d.dx + c * d.dy;
        pose.x += dxg;
        pose.y += dyg;
        pose.theta = normalize_angle(pose.theta + d.dh);

        printf("Pose updated: x=%.3f y=%.3f theta=%.3f deg\n",
               pose.x, pose.y, pose.theta * 180.0 / PI);

        // For each detected lane in this frame: fit line in local frame -> transform to global -> merge
        FrameObs *obs = &sample_frames[f];
        for (int L=0; L<obs->NumOfLane; L++) {
            LaneLineInfo *lli = &obs->lane_line_info_list[L];
            if (lli->NumOfPoints < 2) continue;
            LineParam local_lp;
            local_lp.id = -1;
            local_lp.count = 1;
            // fit
            double r_local, theta_local;
            double cov_local[2][2];
            int ok = fit_weighted_line_pca(lli, &r_local, &theta_local, cov_local);
            if (!ok) continue;

            local_lp.r = r_local;
            local_lp.theta = theta_local;
            local_lp.cov[0][0] = cov_local[0][0];
            local_lp.cov[0][1] = cov_local[0][1];
            local_lp.cov[1][0] = cov_local[1][0];
            local_lp.cov[1][1] = cov_local[1][1];

            // transform to global using updated pose
            LineParam global_lp;
            transform_line_to_global(&local_lp, &pose, &global_lp);

            printf(" Detected lane %d local: r=%.3f theta=%.3fdeg | global: r=%.3f theta=%.3fdeg\n",
                   L, local_lp.r, local_lp.theta*180.0/PI,
                   global_lp.r, global_lp.theta*180.0/PI);

            // merge into map
            merge_into_map(&global_lp, CHI2_THRESH);
        }

        // print current map summary
        printf(" Map lines after frame %d: count=%d\n", f, map_line_count);
        for (int i=0;i<map_line_count;i++) {
            LineParam *m = &map_lines[i];
            printf("  id=%d cnt=%d  r=%.3f  theta=%.3fdeg  cov_rr=%.4e cov_tt=%.4e\n",
                   m->id, m->count, m->r, m->theta*180.0/PI, m->cov[0][0], m->cov[1][1]);
        }
    }
}

int main() {
    prepare_sample_frames();
    // We set number of frames to 3 (as sample_dr and sample_frames prepared)
    process_all_frames(3);
    return 0;
}