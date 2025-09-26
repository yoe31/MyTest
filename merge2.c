// merge_local_pfister.c
// Compile: gcc -O2 merge_local_pfister.c -lm -o merge_local_pfister
// No file IO. Merge(...) is the API. Map is always kept in ego-local coordinates.
//
// Implements Pfister et al. ICRA2003 core formulas for weighted line-fitting and merging.
// Covariance computed via numeric Hessian inversion (log-likelihood Hessian).
//
// Usage (example at bottom):
//  MapLine map[MAX_MAP]; int map_count=0;
//  Merge(&dr, lanes, nlanes, map, &map_count, MAX_MAP);
//  => map[] updated (still in local coordinates)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------- User-tunable sensor noise params (modify to match your sensor) ----------
static double sigma_d = 0.005;    // range noise std (m)
static double sigma_phi = 0.0005; // angular noise std (rad)

// ---------- Types ----------
typedef struct {
    double dX;   // delta forward (vehicle-local +X)
    double dY;   // delta left    (vehicle-local +Y)
    double dH;   // delta heading (radians, CCW +)
} DR;

#define MAX_POINTS  128
#define MAX_LANES   32
#define MAX_MAP     512

typedef struct {
    int NumOfPoints;
    double pts[MAX_POINTS][2]; // [i][0]=x_local, [i][1]=y_local
} LaneLineInfo;

typedef struct {
    int id;
    double R;      // normal distance (in ego-local coords)
    double alpha;  // normal angle (rad) in ego-local coords
    double cov[2][2]; // covariance of [R, alpha]
    double x1,y1,x2,y2; // endpoints in ego-local coords
    int count;     // merge count
} MapLine;

// ---------- Math utils ----------
static double clamp_double(double x, double a, double b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}
static double norm_ang(double a) {
    while (a <= -M_PI) a += 2*M_PI;
    while (a > M_PI) a -= 2*M_PI;
    return a;
}
static int invert2x2(const double A[2][2], double out[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabs(det) < 1e-16) return 0;
    out[0][0] =  A[1][1]/det;
    out[1][1] =  A[0][0]/det;
    out[0][1] = -A[0][1]/det;
    out[1][0] = -A[1][0]/det;
    return 1;
}
static void mat2_mul_vec(const double M[2][2], const double v[2], double out[2]) {
    out[0] = M[0][0]*v[0] + M[0][1]*v[1];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1];
}

// ---------- Pfister core: Pk, negloglik alpha/R ----------
// Pk = sigma_d^2 * cos^2(alpha - phi_k) + sigma_phi^2 * d_k^2 * sin^2(alpha - phi_k)
static double compute_Pk(double alpha, double phi_k, double d_k) {
    double s = sin(alpha - phi_k);
    double c = cos(alpha - phi_k);
    double Pk = (sigma_d*sigma_d) * (c*c) + (sigma_phi*sigma_phi) * (d_k*d_k) * (s*s);
    if (Pk < 1e-12) Pk = 1e-12;
    return Pk;
}

typedef struct { int n; const double *d; const double *phi; } DPP;

static double negloglik_alpha(double alpha, const DPP *pp, double *outR) {
    int n = pp->n;
    double sum_invPk = 0.0;
    double numR = 0.0;
    for (int k=0;k<n;k++) {
        double Pk = compute_Pk(alpha, pp->phi[k], pp->d[k]);
        double invPk = 1.0 / Pk;
        sum_invPk += invPk;
        numR += pp->d[k] * cos(alpha - pp->phi[k]) * invPk;
    }
    double PRR = 1.0 / (sum_invPk + 1e-16);
    double Rhat = PRR * numR;
    if (outR) *outR = Rhat;

    double M = 0.0;
    for (int k=0;k<n;k++) {
        double Pk = compute_Pk(alpha, pp->phi[k], pp->d[k]);
        double delta = pp->d[k]*cos(alpha - pp->phi[k]) - Rhat;
        M += (delta*delta)/Pk + log(Pk);
    }
    return 0.5 * M;
}

// Brent minimizer (1D) for alpha. simple implementation
static double brent_minimize(double ax, double bx, double cx,
                             double (*f)(double,const DPP*,double*),
                             const DPP *pp, double *out_x, double tol) {
    const int ITMAX = 80;
    const double CGOLD = 0.3819660112501051;
    const double ZEPS = 1e-12;
    double a = fmin(ax,cx), b = fmax(ax,cx);
    double x = bx, w = x, v = x;
    double fx = f(x, pp, NULL), fw = fx, fv = fx;
    double d = 0.0, e = 0.0;
    for (int iter=0; iter<ITMAX; ++iter) {
        double xm = 0.5*(a+b);
        double tol1 = tol * fabs(x) + ZEPS;
        double tol2 = 2.0*tol1;
        if (fabs(x - xm) <= (tol2 - 0.5*(b-a))) break;
        double p=0,q=0,r=0;
        if (fabs(e) > tol1) {
            r = (x-w)*(fx - fv);
            q = (x-v)*(fx - fw);
            p = (x-v)*q - (x-w)*r;
            q = 2.0*(q - r);
            if (q > 0.0) p = -p;
            q = fabs(q);
            double etemp = e;
            e = d;
            if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a - x) || p >= q*(b - x)) {
                e = (x >= xm) ? a-x : b-x;
                d = CGOLD * e;
            } else {
                d = p / q;
                double u = x + d;
                if (u - a < tol2 || b - u < tol2) d = (xm - x >= 0) ? fabs(tol1) : -fabs(tol1);
            }
        } else {
            e = (x >= xm) ? a-x : b-x;
            d = CGOLD * e;
        }
        double u = (fabs(d) >= tol1) ? x + d : x + ((d>0)?tol1:-tol1);
        double fu = f(u, pp, NULL);
        if (fu <= fx) {
            if (u >= x) a = x; else b = x;
            v = w; fv = fw;
            w = x; fw = fx;
            x = u; fx = fu;
        } else {
            if (u < x) a = u; else b = u;
            if (fu <= fw || w == x) { v = w; fv = fw; w = u; fw = fu; }
            else if (fu <= fv || v == x || v == w) { v = u; fv = fu; }
        }
    }
    if (out_x) *out_x = x;
    return fx;
}

// ---------- Pfister fit: fit_line_pfister ----------
// returns R,alpha,cov(2x2), endpoints (local frame)
// cov computed by numeric Hessian of M(R,alpha) and inversion
static int fit_line_pfister(const LaneLineInfo *L,
                            double *outR, double *outAlpha, double cov_out[2][2],
                            double *end1x, double *end1y, double *end2x, double *end2y) {
    int n = L->NumOfPoints;
    if (n < 2) return 0;
    double d[MAX_POINTS], phi[MAX_POINTS];
    for (int i=0;i<n;i++) {
        double x = L->pts[i][0], y = L->pts[i][1];
        d[i] = sqrt(x*x + y*y);
        phi[i] = atan2(y, x);
    }
    DPP pp; pp.n = n; pp.d = d; pp.phi = phi;

    // initial alpha via PCA direction -> normal angle
    double cx=0, cy=0;
    for (int i=0;i<n;i++){ cx += L->pts[i][0]; cy += L->pts[i][1]; }
    cx /= n; cy /= n;
    double Sxx=0,Syy=0,Sxy=0;
    for (int i=0;i<n;i++){ double dx=L->pts[i][0]-cx, dy=L->pts[i][1]-cy; Sxx+=dx*dx; Syy+=dy*dy; Sxy+=dx*dy; }
    double theta_dir = 0.5 * atan2(2*Sxy, Sxx - Syy);
    double alpha0 = norm_ang(theta_dir + M_PI/2.0);

    // minimize M(alpha)
    double a = alpha0 - 1.0, b = alpha0, c = alpha0 + 1.0;
    double alpha_opt;
    brent_minimize(a,b,c, negloglik_alpha, &pp, &alpha_opt, 1e-8);
    double Rhat;
    negloglik_alpha(alpha_opt, &pp, &Rhat);

    // numeric Hessian of M(R,alpha)
    // define M(R,A)
    auto Mfunc = [&](double Rv, double Av)->double {
        double M=0.0;
        for (int k=0;k<n;k++) {
            double Pk = compute_Pk(Av, phi[k], d[k]);
            double delta = d[k]*cos(Av - phi[k]) - Rv;
            M += (delta*delta)/Pk + log(Pk);
        }
        return 0.5 * M;
    };

    double epsR = 1e-5;
    double epsA = 1e-6;
    double R = Rhat, A = alpha_opt;
    double M_rr = (Mfunc(R+epsR, A) - 2.0*Mfunc(R, A) + Mfunc(R-epsR, A)) / (epsR*epsR);
    double M_aa = (Mfunc(R, A+epsA) - 2.0*Mfunc(R, A) + Mfunc(R, A-epsA)) / (epsA*epsA);
    double M_ra = (Mfunc(R+epsR, A+epsA) - Mfunc(R+epsR, A-epsA) - Mfunc(R-epsR, A+epsA) + Mfunc(R-epsR, A-epsA)) / (4.0*epsR*epsA);

    double H[2][2] = {{M_rr, M_ra},{M_ra, M_aa}};
    double cov[2][2];
    if (!invert2x2(H, cov)) {
        // fallback
        cov[0][0] = 1e-4; cov[0][1] = 0.0;
        cov[1][0] = 0.0;    cov[1][1] = 1e-4;
    } // else cov is inv(H)

    // endpoints by projecting points onto line
    double dirx = -sin(A), diry = cos(A);
    double x0 = R * cos(A), y0 = R * sin(A);
    double minp = 1e12, maxp = -1e12;
    for (int k=0;k<n;k++) {
        double px = L->pts[k][0], py = L->pts[k][1];
        double proj = px*dirx + py*diry;
        if (proj < minp) minp = proj;
        if (proj > maxp) maxp = proj;
    }
    *end1x = x0 + dirx * minp;
    *end1y = y0 + diry * minp;
    *end2x = x0 + dirx * maxp;
    *end2y = y0 + diry * maxp;

    *outR = Rhat;
    *outAlpha = norm_ang(A);
    cov_out[0][0] = cov[0][0]; cov_out[0][1] = cov[0][1];
    cov_out[1][0] = cov[1][0]; cov_out[1][1] = cov[1][1];
    return 1;
}

// ---------- Map transform: when vehicle moves by dr (in vehicle-local coords),
// we must update every stored MapLine so that map remains in the *new* ego-local frame.
//
// Derivation summary:
// old normal: n_old = [cos alpha_old, sin alpha_old], line: n_old . p_old = R_old
// new frame: p_new = R(-dH)*(p_old - dr)  =>  p_old = R(dH)*p_new + dr
// Substitute: n_old.(R(dH)p_new + dr) = R_old => (R(-dH) n_old) . p_new = R_old - n_old . dr
// => alpha_new = alpha_old - dH
//    R_new = R_old - dr . n_old  (dr in old local coords)
// Covariance transform: p_new = f(p_old) => J = df/dp_old
// p = [R; alpha] -> R_new = R_old - dX*cos(alpha_old) - dY*sin(alpha_old)
// so J = [[1, dX*sin(alpha_old) - dY*cos(alpha_old)], [0,1]]
static void transform_map_by_dr(MapLine map[], int map_count, const DR *dr) {
    if (!dr) return;
    double dX = dr->dX, dY = dr->dY;
    double dH = dr->dH;
    double ca = cos(-dH), sa = sin(-dH); // rotation for endpoints: R(-dH)
    for (int i=0;i<map_count;i++) {
        MapLine *m = &map[i];
        // update alpha
        double alpha_old = m->alpha;
        double alpha_new = norm_ang(alpha_old - dH);
        // update R
        double R_new = m->R - (dX * cos(alpha_old) + dY * sin(alpha_old));
        // update covariance: J * cov_old * J^T
        double J[2][2];
        J[0][0] = 1.0;
        J[0][1] = dX * sin(alpha_old) - dY * cos(alpha_old);
        J[1][0] = 0.0;
        J[1][1] = 1.0;
        double tmp[2][2] = {{0,0},{0,0}};
        for (int r=0;r<2;r++) for (int c=0;c<2;c++) {
            tmp[r][c] = J[r][0]*m->cov[0][c] + J[r][1]*m->cov[1][c];
        }
        double cov_new[2][2] = {{0,0},{0,0}};
        for (int r=0;r<2;r++) for (int c=0;c<2;c++) {
            cov_new[r][c] = tmp[r][0]*J[c][0] + tmp[r][1]*J[c][1];
        }
        // transform endpoints: p_new = R(-dH) * (p_old - dr)
        double px1 = m->x1 - dX, py1 = m->y1 - dY;
        double px2 = m->x2 - dX, py2 = m->y2 - dY;
        double nx1 = ca*px1 - sa*py1;
        double ny1 = sa*px1 + ca*py1;
        double nx2 = ca*px2 - sa*py2;
        double ny2 = sa*px2 + ca*py2;

        // commit
        m->alpha = alpha_new;
        m->R = R_new;
        m->cov[0][0] = cov_new[0][0];
        m->cov[0][1] = cov_new[0][1];
        m->cov[1][0] = cov_new[1][0];
        m->cov[1][1] = cov_new[1][1];
        m->x1 = nx1; m->y1 = ny1;
        m->x2 = nx2; m->y2 = ny2;
    }
}

// ---------- merge utilities (chi2 and information fusion) ----------
static double chi2_between(const MapLine *a, const MapLine *b) {
    double dR = a->R - b->R;
    double da = norm_ang(a->alpha - b->alpha);
    double dv[2] = {dR, da};
    double S[2][2] = {
        { a->cov[0][0] + b->cov[0][0], a->cov[0][1] + b->cov[0][1] },
        { a->cov[1][0] + b->cov[1][0], a->cov[1][1] + b->cov[1][1] }
    };
    double invS[2][2];
    if (!invert2x2(S, invS)) return 1e12;
    double tmp[2];
    mat2_mul_vec(invS, dv, tmp);
    return dv[0]*tmp[0] + dv[1]*tmp[1];
}

static void merge_two_maplines(const MapLine *A, const MapLine *B, MapLine *out) {
    double S1[2][2] = {{A->cov[0][0], A->cov[0][1]}, {A->cov[1][0], A->cov[1][1]}};
    double S2[2][2] = {{B->cov[0][0], B->cov[0][1]}, {B->cov[1][0], B->cov[1][1]}};
    double invS1[2][2], invS2[2][2];
    if (!invert2x2(S1, invS1) || !invert2x2(S2, invS2)) {
        // fallback simple average
        *out = *A;
        out->R = 0.5*(A->R + B->R);
        out->alpha = norm_ang(0.5*(A->alpha + B->alpha));
        out->cov[0][0] = 0.5*(A->cov[0][0] + B->cov[0][0]);
        out->cov[0][1] = 0.0;
        out->cov[1][0] = 0.0;
        out->cov[1][1] = 0.5*(A->cov[1][1] + B->cov[1][1]);
        out->count = A->count + B->count;
        // endpoints average too
        out->x1 = 0.5*(A->x1 + B->x1); out->y1 = 0.5*(A->y1 + B->y1);
        out->x2 = 0.5*(A->x2 + B->x2); out->y2 = 0.5*(A->y2 + B->y2);
        return;
    }
    double Info[2][2] = {
        { invS1[0][0] + invS2[0][0], invS1[0][1] + invS2[0][1] },
        { invS1[1][0] + invS2[1][0], invS1[1][1] + invS2[1][1] }
    };
    double covm[2][2];
    if (!invert2x2(Info, covm)) {
        *out = *A;
        out->R = 0.5*(A->R + B->R);
        out->alpha = norm_ang(0.5*(A->alpha + B->alpha));
        out->cov[0][0] = 0.5*(A->cov[0][0] + B->cov[0][0]);
        out->cov[1][1] = 0.5*(A->cov[1][1] + B->cov[1][1]);
        out->count = A->count + B->count;
        out->x1 = 0.5*(A->x1 + B->x1); out->y1 = 0.5*(A->y1 + B->y1);
        out->x2 = 0.5*(A->x2 + B->x2); out->y2 = 0.5*(A->y2 + B->y2);
        return;
    }
    // rhs = invS1*p1 + invS2*p2
    double p1[2] = {A->R, A->alpha};
    double p2[2] = {B->R, B->alpha};
    double rhs[2] = {0,0}, tmpv[2];
    mat2_mul_vec(invS1, p1, tmpv); rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1];
    mat2_mul_vec(invS2, p2, tmpv); rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1];
    double pmerged[2];
    mat2_mul_vec(covm, rhs, pmerged);
    *out = *A;
    out->R = pmerged[0];
    out->alpha = norm_ang(pmerged[1]);
    out->cov[0][0] = covm[0][0]; out->cov[0][1] = covm[0][1];
    out->cov[1][0] = covm[1][0]; out->cov[1][1] = covm[1][1];
    out->count = A->count + B->count;
    // endpoints: project A/B endpoints onto merged line axis and take extremes
    double dirx = -sin(out->alpha), diry = cos(out->alpha);
    double x0 = out->R * cos(out->alpha), y0 = out->R * sin(out->alpha);
    double ptsx[4] = {A->x1, A->x2, B->x1, B->x2};
    double ptsy[4] = {A->y1, A->y2, B->y1, B->y2};
    double mn = 1e12, mx = -1e12;
    for (int i=0;i<4;i++) {
        double proj = (ptsx[i] - x0)*dirx + (ptsy[i] - y0)*diry;
        if (proj < mn) mn = proj;
        if (proj > mx) mx = proj;
    }
    out->x1 = x0 + dirx * mn; out->y1 = y0 + diry * mn;
    out->x2 = x0 + dirx * mx; out->y2 = y0 + diry * mx;
}

// ---------- Merge API ----------
// - map[] is always in ego-local coords and is updated in-place
// - dr (delta) is applied to map first so map remains in new ego-local frame
// - incoming lanes[] are in ego-local coordinates of the *new* frame already (caller should pass measurements in current vehicle frame).
//   If incoming measurements are taken before applying dr you'd call Merge with dr=NULL first for matching behavior.
// - map_count is in/out pointer
static void Merge(const DR *dr, const LaneLineInfo lanes[], int nlanes,
                  MapLine map[], int *map_count, int max_map) {
    // 1) update map coordinates for vehicle movement
    if (dr) transform_map_by_dr(map, *map_count, dr);

    // 2) for each incoming lane (already in current ego-local frame), fit using Pfister
    for (int i=0;i<nlanes;i++) {
        if (lanes[i].NumOfPoints < 2) continue;
        double Rloc, alphal;
        double covl[2][2];
        double e1x,e1y,e2x,e2y;
        if (!fit_line_pfister(&lanes[i], &Rloc, &alphal, covl, &e1x,&e1y,&e2x,&e2y)) continue;

        // candidate MapLine in local coords
        MapLine cand;
        memset(&cand,0,sizeof(cand));
        cand.R = Rloc; cand.alpha = alphal;
        cand.cov[0][0] = covl[0][0]; cand.cov[0][1] = covl[0][1];
        cand.cov[1][0] = covl[1][0]; cand.cov[1][1] = covl[1][1];
        cand.x1 = e1x; cand.y1 = e1y; cand.x2 = e2x; cand.y2 = e2y;
        cand.count = 1;

        // merge into existing map by chi2
        double best_chi2 = 1e12;
        int best_idx = -1;
        for (int m=0;m<*map_count;m++) {
            double c2 = chi2_between(&cand, &map[m]);
            if (c2 < best_chi2) { best_chi2 = c2; best_idx = m; }
        }
        double CHI2_THRESH = 5.991; // DOF=2, 95%
        if (best_idx >= 0 && best_chi2 < CHI2_THRESH) {
            MapLine merged;
            merge_two_maplines(&map[best_idx], &cand, &merged);
            merged.id = map[best_idx].id;
            map[best_idx] = merged;
        } else {
            if (*map_count < max_map) {
                cand.id = (*map_count) + 1;
                map[(*map_count)++] = cand;
            } else {
                // map full: ignore
            }
        }
    }
}

// ----------------- minimal example usage -----------------
#ifdef MERGE_LOCAL_EXAMPLE
int main() {
    MapLine map[MAX_MAP];
    int map_count = 0;

    // example: initial frame - create two synthetic lanes (local coords)
    LaneLineInfo lanes[2];
    lanes[0].NumOfPoints = 3;
    lanes[0].pts[0][0] = 10.0; lanes[0].pts[0][1] = -3.0;
    lanes[0].pts[1][0] = 10.0; lanes[0].pts[1][1] = 0.0;
    lanes[0].pts[2][0] = 10.0; lanes[0].pts[2][1] = 3.0;
    lanes[1].NumOfPoints = 3;
    lanes[1].pts[0][0] = 20.0; lanes[1].pts[0][1] = -3.0;
    lanes[1].pts[1][0] = 20.0; lanes[1].pts[1][1] = 0.0;
    lanes[1].pts[2][0] = 20.0; lanes[1].pts[2][1] = 3.0;

    DR dr0 = {0.0,0.0,0.0};
    Merge(&dr0, lanes, 2, map, &map_count, MAX_MAP);

    printf("After frame0 map_count=%d\n", map_count);
    for (int i=0;i<map_count;i++) {
        printf("  id=%d R=%.3f alpha=%.3fdeg cnt=%d\n", map[i].id, map[i].R, map[i].alpha*180.0/M_PI, map[i].count);
    }

    // vehicle moves forward 1.0m (local dx=1), no rotate
    DR dr1 = {1.0,0.0,0.0};
    // new detections in new local frame (approx shifted by -1 in x)
    LaneLineInfo lanes1[2];
    lanes1[0].NumOfPoints = 3;
    lanes1[0].pts[0][0] = 9.05; lanes1[0].pts[0][1] = -3.1;
    lanes1[0].pts[1][0] = 8.95; lanes1[0].pts[1][1] = 0.05;
    lanes1[0].pts[2][0] = 9.02; lanes1[0].pts[2][1] = 3.0;
    lanes1[1].NumOfPoints = 3;
    lanes1[1].pts[0][0] = 19.1; lanes1[1].pts[0][1] = -2.9;
    lanes1[1].pts[1][0] = 19.0; lanes1[1].pts[1][1] = 0.1;
    lanes1[1].pts[2][0] = 19.02; lanes1[1].pts[2][1] = 2.95;

    Merge(&dr1, lanes1, 2, map, &map_count, MAX_MAP);
    printf("After frame1 map_count=%d\n", map_count);
    for (int i=0;i<map_count;i++) {
        printf("  id=%d R=%.3f alpha=%.3fdeg cnt=%d endpoints: (%.2f,%.2f)-(%.2f,%.2f)\n",
               map[i].id, map[i].R, map[i].alpha*180.0/M_PI, map[i].count,
               map[i].x1, map[i].y1, map[i].x2, map[i].y2);
    }
    return 0;
}
#endif

// End of file