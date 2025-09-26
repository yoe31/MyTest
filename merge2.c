// merge_only_pfister.c
// Compile: gcc -O2 merge_only_pfister.c -lm -o merge_only_pfister
// This file provides Merge(...) function implementing Pfister et al. ICRA2003
// core line-fit (Eq.11/12, Prop.1 Eq.17/18), numeric Hessian covariance, and chi2 merge.
// No file IO. No preprocessing. See comments for formulas references.
// Paper: Pfister, Roumeliotis, Burdick, "Weighted Line Fitting Algorithms..." ICRA 2003.  [oai_citation:1‡robotics.caltech.edu](https://robotics.caltech.edu/~sam/ConferencePapers/ICRA03/LineFit/LineFitMerge_ICRA03.pdf)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------- types ----------
typedef struct {
    double dX; // ΔX in vehicle frame (forward = +X)
    double dY; // ΔY in vehicle frame (left = +Y)
    double dH; // Δheading in radians (CCW positive)
} DR;

#define MAX_POINTS  64
#define MAX_LANES   32
#define MAX_MAP     512

typedef struct {
    int NumOfPoints;
    double pts[MAX_POINTS][2]; // [i][0]=X_local, [i][1]=Y_local (vehicle frame)
} LaneLineInfo;

typedef struct {
    double x;   // up = +X
    double y;   // left = +Y
    double theta; // rad, vehicle global heading (CCW +)
} Pose;

// MapLine stores a merged line in global frame in normal form (R,alpha) plus endpoints and cov
typedef struct {
    int id;
    double R;         // normal distance (global)
    double alpha;     // normal angle (rad) global
    double cov[2][2]; // covariance of [R, alpha]
    double x1,y1,x2,y2; // endpoints in global coords
    int count;        // how many segments merged
} MapLine;

// ---------- global settings (sensor noise) ----------
// Values here are placeholders; set to your sensor's characteristics.
// sigma_d: standard deviation of range measurement (m)
// sigma_phi: std of angular measurement (rad)
static double sigma_d = 0.005;    // e.g., 5mm -- adjust for your sensor
static double sigma_phi = 0.0005; // e.g., ~0.03deg

// ---------- utility ----------
static double norm_ang(double a) {
    while (a <= -M_PI) a += 2*M_PI;
    while (a > M_PI) a -= 2*M_PI;
    return a;
}
static int invert2x2(const double A[2][2], double out[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    double eps = 1e-14;
    if (fabs(det) < eps) return 0;
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

// ---------- Pfister sensor-model Pk (Eq.11/Eq.12) ----------
// Using polar form for point: d (range), phi (angle)
// Eq.(11): Pk = sigma_d^2 * cos^2(alpha - phi_k) + sigma_phi^2 * d_k^2 * sin^2(alpha - phi_k)
// Eq.(12): more general: if Qi is 2x2 covariance for point then Pk = Q11 cos^2 α + 2 Q12 sinα cosα + Q22 sin^2 α
static double compute_Pk(double alpha, double phi_k, double d_k) {
    double s = sin(alpha - phi_k);
    double c = cos(alpha - phi_k);
    double Pk = (sigma_d*sigma_d) * (c*c) + (sigma_phi*sigma_phi) * (d_k*d_k) * (s*s);
    if (Pk < 1e-12) Pk = 1e-12;
    return Pk;
}

// ---------- negative log-likelihood M(alpha) with R computed analytically (Prop.1 Eq.17/18) ----------
// Given arrays d[k], phi[k] (k=0..n-1), for a candidate alpha compute Pk, compute PRR and Rhat by Eq.18/17
// and return M = 0.5 * sum_k ( (delta_k^2)/Pk + ln(Pk) )  where delta_k = d_k cos(alpha-phi_k) - Rhat
typedef struct { int n; const double *d; const double *phi; } DPP;

static double negloglik_alpha(double alpha, const DPP *pp, double *outR) {
    int n = pp->n;
    double sum_invPk = 0.0;
    double numR = 0.0;
    // first compute Pk and needed sums
    for (int k=0;k<n;k++) {
        double Pk = compute_Pk(alpha, pp->phi[k], pp->d[k]);
        double invPk = 1.0 / Pk;
        sum_invPk += invPk;
        numR += pp->d[k] * cos(alpha - pp->phi[k]) * invPk;
    }
    double PRR = 1.0 / (sum_invPk + 1e-16); // Eq.18
    double Rhat = PRR * numR; // Eq.17
    if (outR) *outR = Rhat;

    double M = 0.0;
    for (int k=0;k<n;k++) {
        double Pk = compute_Pk(alpha, pp->phi[k], pp->d[k]);
        double delta = pp->d[k]*cos(alpha - pp->phi[k]) - Rhat;
        M += (delta*delta)/Pk + log(Pk);
    }
    M *= 0.5;
    return M;
}

// ---------- Brent 1D minimizer (for alpha) ----------
static double brent_minimize(double ax, double bx, double cx,
                             double (*f)(double,const DPP*,double*),
                             const DPP *pp, double *out_x, double tol) {
    const int ITMAX = 100;
    const double CGOLD = 0.3819660;
    const double ZEPS = 1e-12;
    double a = fmin(ax,cx);
    double b = fmax(ax,cx);
    double x = bx, w = x, v = x;
    double fx = f(x, pp, NULL), fw = fx, fv = fx;
    double d = 0.0, e = 0.0;
    for (int iter=0; iter<ITMAX; ++iter) {
        double xm = 0.5*(a+b);
        double tol1 = tol*fabs(x) + ZEPS;
        double tol2 = 2.0*tol1;
        if (fabs(x - xm) <= (tol2 - 0.5*(b-a))) break;
        double p=0,q=0,r=0;
        if (fabs(e) > tol1) {
            r = (x - w)*(fx - fv);
            q = (x - v)*(fx - fw);
            p = (x - v)*q - (x - w)*r;
            q = 2.0*(q - r);
            if (q > 0) p = -p;
            q = fabs(q);
            double etemp = e;
            e = d;
            if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a - x) || p >= q*(b - x))
            { e = (x >= xm) ? a-x : b-x; d = CGOLD * e; }
            else { d = p / q; double u = x + d; if (u - a < tol2 || b - u < tol2) d = (xm - x >= 0) ? fabs(tol1) : -fabs(tol1); }
        } else {
            e = (x >= xm) ? a-x : b-x;
            d = CGOLD * e;
        }
        double u = (fabs(d) >= tol1) ? x + d : x + ((d>0)?tol1:-tol1);
        double fu = f(u, pp, NULL);
        if (fu <= fx) {
            if (u >= x) a = x; else b = x;
            v=w; fv=fw;
            w=x; fw=fx;
            x=u; fx=fu;
        } else {
            if (u < x) a = u; else b = u;
            if (fu <= fw || w==x) { v=w; fv=fw; w=u; fw=fu; }
            else if (fu <= fv || v==x || v==w) { v=u; fv=fu; }
        }
    }
    if (out_x) *out_x = x;
    return fx;
}

// ---------- Fit single line (per Pfister) ----------
// Inputs: lane (points in vehicle frame), outputs: R,alpha,cov(2x2), endpoints in local frame
// Implementation notes:
// - Convert points to polar (d_k, phi_k)
// - minimize negloglik over alpha (Brent) using Eq.(11)/(12) and Eq.(17)/(18) for R
// - compute covariance: numeric Hessian of M(R,alpha) at optimum, invert -> cov
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

    // initial alpha guess via PCA (direction), then normal = dir + pi/2
    double cx=0, cy=0;
    for (int i=0;i<n;i++) { cx += L->pts[i][0]; cy += L->pts[i][1]; }
    cx /= n; cy /= n;
    double Sxx=0,Syy=0,Sxy=0;
    for (int i=0;i<n;i++) { double dx = L->pts[i][0]-cx, dy=L->pts[i][1]-cy; Sxx += dx*dx; Syy += dy*dy; Sxy += dx*dy; }
    double theta_dir = 0.5 * atan2(2*Sxy, Sxx - Syy);
    double alpha0 = norm_ang(theta_dir + M_PI/2.0);

    // bracket around alpha0
    double a = alpha0 - 0.8, b = alpha0, c = alpha0 + 0.8;
    double alpha_opt;
    brent_minimize(a, b, c, negloglik_alpha, &pp, &alpha_opt, 1e-8);

    double Rhat;
    negloglik_alpha(alpha_opt, &pp, &Rhat);

    // numeric Hessian of M(R,alpha) at (Rhat,alpha_opt)
    auto Mfunc = [&](double R, double A)->double {
        double M=0;
        for (int k=0;k<n;k++) {
            double Pk = compute_Pk(A, phi[k], d[k]);
            double delta = d[k]*cos(A - phi[k]) - R;
            M += (delta*delta)/Pk + log(Pk);
        }
        return 0.5 * M;
    };

    double epsR = 1e-5; // small perturbation (m)
    double epsA = 1e-6; // small perturbation (rad)
    double R = Rhat, A = alpha_opt;

    double M_rr = (Mfunc(R+epsR, A) - 2.0*Mfunc(R, A) + Mfunc(R-epsR, A)) / (epsR*epsR);
    double M_aa = (Mfunc(R, A+epsA) - 2.0*Mfunc(R, A) + Mfunc(R, A-epsA)) / (epsA*epsA);
    double M_ra = (Mfunc(R+epsR, A+epsA) - Mfunc(R+epsR, A-epsA) - Mfunc(R-epsR, A+epsA) + Mfunc(R-epsR, A-epsA)) / (4.0*epsR*epsA);

    double H[2][2] = { {M_rr, M_ra}, {M_ra, M_aa} };

    double cov[2][2];
    if (!invert2x2(H, cov)) {
        // fallback small diagonal covariance if Hessian singular
        cov[0][0] = 1e-4; cov[0][1] = 0.0;
        cov[1][0] = 0.0;    cov[1][1] = 1e-4;
    } else {
        // cov is inv(H)
    }

    // endpoints: project points onto line (local frame)
    double dirx = -sin(A), diry = cos(A); // direction along the line
    double x0 = R * cos(A), y0 = R * sin(A); // a point on the line in polar normal form
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

// ---------- chi2 distance and merge (information form) ----------
static double chi2_between(const MapLine *a, const MapLine *b) {
    double dR = a->R - b->R;
    double da = norm_ang(a->alpha - b->alpha);
    double dv[2] = { dR, da };
    double S[2][2] = {
        { a->cov[0][0] + b->cov[0][0], a->cov[0][1] + b->cov[0][1] },
        { a->cov[1][0] + b->cov[1][0], a->cov[1][1] + b->cov[1][1] }
    };
    double invS[2][2];
    if (!invert2x2(S, invS)) return 1e9;
    double tmp[2];
    mat2_mul_vec(invS, dv, tmp);
    return dv[0]*tmp[0] + dv[1]*tmp[1];
}
static void merge_two_lines(const MapLine *A, const MapLine *B, MapLine *out) {
    double S1[2][2] = {{A->cov[0][0], A->cov[0][1]}, {A->cov[1][0], A->cov[1][1]}};
    double S2[2][2] = {{B->cov[0][0], B->cov[0][1]}, {B->cov[1][0], B->cov[1][1]}};
    double invS1[2][2], invS2[2][2];
    if (!invert2x2(S1, invS1) || !invert2x2(S2, invS2)) {
        // fallback average
        *out = *A;
        out->R = 0.5*(A->R + B->R);
        out->alpha = norm_ang(0.5*(A->alpha + B->alpha));
        out->cov[0][0] = 0.5*(A->cov[0][0] + B->cov[0][0]);
        out->cov[1][1] = 0.5*(A->cov[1][1] + B->cov[1][1]);
        out->count = A->count + B->count;
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
        return;
    }
    // rhs = invS1 * p1 + invS2 * p2
    double p1[2] = {A->R, A->alpha};
    double p2[2] = {B->R, B->alpha};
    double rhs[2] = {0,0}, tmpv[2];
    mat2_mul_vec(invS1, p1, tmpv); rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1];
    mat2_mul_vec(invS2, p2, tmpv); rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1];
    double pmerged[2];
    mat2_mul_vec(covm, rhs, pmerged);
    *out = *A; // base
    out->R = pmerged[0];
    out->alpha = norm_ang(pmerged[1]);
    out->cov[0][0] = covm[0][0]; out->cov[0][1] = covm[0][1];
    out->cov[1][0] = covm[1][0]; out->cov[1][1] = covm[1][1];
    out->count = A->count + B->count;
    // merge endpoints by projecting A and B endpoints onto merged line and taking extremes
    double dirx = -sin(out->alpha), diry = cos(out->alpha);
    double x0 = out->R * cos(out->alpha), y0 = out->R * sin(out->alpha);
    double px[4] = {A->x1, A->x2, B->x1, B->x2};
    double py[4] = {A->y1, A->y2, B->y1, B->y2};
    double mn = 1e12, mx = -1e12;
    for (int i=0;i<4;i++) {
        double pr = (px[i]-x0)*dirx + (py[i]-y0)*diry;
        if (pr < mn) mn = pr;
        if (pr > mx) mx = pr;
    }
    out->x1 = x0 + dirx * mn; out->y1 = y0 + diry * mn;
    out->x2 = x0 + dirx * mx; out->y2 = y0 + diry * mx;
}

// ---------- Merge API ----------
// Merge processes one frame: it applies DR to ego pose (assumed to be delta from previous),
// fits each lane in vehicle frame to a line (R,alpha) per Pfister, transforms to global frame,
// computes covariance, and merges into map[] (size *map_count). No I/O done here.
//
// Parameters:
//  - dr: DR delta since last call (apply to ego pose). If dr == NULL, pose unchanged.
//  - lanes: array of LaneLineInfo (in vehicle frame) for this frame
//  - nlanes: number of lanes in lanes[]
//  - ego: pointer to Pose (in/out). On entry: previous global ego pose; on exit: updated pose
//  - map: array of MapLine (in/out). map_count is in/out
//  - map_count: pointer to current number of map lines; updated by Merge
//  - max_map: capacity of map[]
//
// Notes: chi2 threshold uses DOF=2, 95% => 5.991
static void Merge(const DR *dr, const LaneLineInfo lanes[], int nlanes,
                  Pose *ego, MapLine map[], int *map_count, int max_map) {
    // 1) apply DR to ego pose
    if (dr) {
        double c = cos(ego->theta), s = sin(ego->theta);
        double dxg = c * dr->dX - s * dr->dY;
        double dyg = s * dr->dX + c * dr->dY;
        ego->x += dxg;
        ego->y += dyg;
        ego->theta = norm_ang(ego->theta + dr->dH);
    }

    // For each detected lane: fit line in local, compute cov, transform to global, merge
    for (int i=0;i<nlanes;i++) {
        const LaneLineInfo *L = &lanes[i];
        if (L->NumOfPoints < 2) continue;
        double Rloc, alphal;
        double covl[2][2];
        double e1x,e1y,e2x,e2y;
        if (!fit_line_pfister(L, &Rloc, &alphal, covl, &e1x,&e1y,&e2x,&e2y)) continue;

        // transform line to global using ego pose (pose assumed exact here)
        double alphag = norm_ang(alphal + ego->theta);
        double rg = Rloc + ego->x * cos(alphag) + ego->y * sin(alphag);
        // Jacobian J = [1, dR/dalphal; 0, 1], where dR/dalphal = -x sin(alphag) + y cos(alphag)
        double dR_dalpha = -ego->x * sin(alphag) + ego->y * cos(alphag);
        double J[2][2] = {{1.0, dR_dalpha}, {0.0, 1.0}};
        double tmp[2][2] = {{0,0},{0,0}};
        for (int r2=0;r2<2;r2++) for (int c2=0;c2<2;c2++) {
            tmp[r2][c2] = J[r2][0]*covl[0][c2] + J[r2][1]*covl[1][c2];
        }
        double covg[2][2] = {{0,0},{0,0}};
        for (int r2=0;r2<2;r2++) for (int c2=0;c2<2;c2++) {
            covg[r2][c2] = tmp[r2][0]*J[c2][0] + tmp[r2][1]*J[c2][1];
        }

        // build candidate MapLine
        MapLine cand;
        memset(&cand,0,sizeof(cand));
        cand.R = rg; cand.alpha = alphag;
        cand.cov[0][0]=covg[0][0]; cand.cov[0][1]=covg[0][1];
        cand.cov[1][0]=covg[1][0]; cand.cov[1][1]=covg[1][1];
        cand.count = 1;
        // transform endpoints from local to global
        double ch = cos(ego->theta), sh = sin(ego->theta);
        cand.x1 = ego->x + e1x*ch - e1y*sh;
        cand.y1 = ego->y + e1x*sh + e1y*ch;
        cand.x2 = ego->x + e2x*ch - e2y*sh;
        cand.y2 = ego->y + e2x*sh + e2y*ch;

        // merge into map by chi2
        double best_chi2 = 1e12;
        int best_idx = -1;
        for (int m=0;m<*map_count;m++) {
            double c2 = chi2_between(&cand, &map[m]);
            if (c2 < best_chi2) { best_chi2 = c2; best_idx = m; }
        }
        double CHI2_THRESH = 5.991; // 95% for dof=2
        if (best_idx >=0 && best_chi2 < CHI2_THRESH) {
            MapLine merged;
            merge_two_lines(&map[best_idx], &cand, &merged);
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