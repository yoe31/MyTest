// merge_pfister.c
// Compile: gcc -O2 merge_pfister.c -lm -o merge_pfister
// Usage: ./merge_pfister input.csv outdir
// Outputs: outdir/merged.csv  and outdir/merged.geojson (overwritten each frame)
//
// Implements weighted line fitting following Pfister et al. ICRA 2003.
// Uses Pk = sigma_d^2 cos^2(alpha-phi) + sigma_phi^2 d^2 sin^2(alpha-phi) (Eq.11).
// R computed via Eq.(17). alpha found by 1D numeric minimization of neg-log-likelihood.
// Covariance computed by numeric Hessian of the negative log-likelihood and inverted.
//
// Citation (paper used): Pfister, Roumeliotis, Burdick, "Weighted Line Fitting Algorithms ...", ICRA 2003.
// See: https://robotics.caltech.edu/~sam/ConferencePapers/ICRA03/LineFit/LineFitMerge_ICRA03.pdf
// (the code uses the same formulas; I used numerical Hessian inversion for covariance).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#define MAX_POINTS  200
#define MAX_LANES   64
#define MAX_MAP     512
#define PI 3.14159265358979323846

typedef struct {
    int NumOfPoints;
    double x[MAX_POINTS];
    double y[MAX_POINTS];
} LaneLineInfo;

typedef struct {
    double dX, dY, dH; // DR: dx, dy (m), dH (deg, CCW +)
    int hasDR;
    int NumOfLane;
    LaneLineInfo lanes[MAX_LANES];
} FrameObs;

typedef struct {
    double x,y,theta; // global pose: x (up), y (left), theta (rad), CCW +
} Pose;

typedef struct {
    int id;
    double R;       // normal distance
    double alpha;   // normal angle (rad) in global frame, normalized (-pi,pi]
    double cov[2][2]; // cov matrix for [R, alpha]
    double x1,y1,x2,y2; // segment endpoints in global coords (for geojson)
    int count; // number of merges
} MapLine;

// global map
static MapLine g_map[MAX_MAP];
static int g_map_count = 0;
static int g_next_id = 1;

// default sensor noise params (can be tuned)
static double sigma_d = 0.005;     // meter (range measurement noise std) — user may change
static double sigma_phi = 1e-4;    // rad (angle noise std) — user may change

// ---------------------- utils ------------------------
static double norm_ang(double a) {
    while (a <= -PI) a += 2*PI;
    while (a > PI) a -= 2*PI;
    return a;
}
static int invert2x2(double A[2][2], double out[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    double eps = 1e-12;
    if (fabs(det) < eps) return 0;
    out[0][0] =  A[1][1]/det;
    out[1][1] =  A[0][0]/det;
    out[0][1] = -A[0][1]/det;
    out[1][0] = -A[1][0]/det;
    return 1;
}
static void mat2_mul_vec(double M[2][2], double v[2], double out[2]) {
    out[0] = M[0][0]*v[0] + M[0][1]*v[1];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1];
}

// ---------------- sensor-model / Pk (Eq 11/12) ----------------
// Pk = sigma_d^2 * cos^2(alpha-phi_k) + sigma_phi^2 * d_k^2 * sin^2(alpha-phi_k)
static double compute_Pk(double alpha, double phi_k, double d_k) {
    double s = sin(alpha - phi_k);
    double c = cos(alpha - phi_k);
    double Pk = (sigma_d*sigma_d) * (c*c) + (sigma_phi*sigma_phi) * (d_k*d_k) * (s*s);
    // regularize
    if (Pk < 1e-12) Pk = 1e-12;
    return Pk;
}

// ---------------- negate log-likelihood for given alpha (we compute R via Eq 17 inside) ----------------
// Inputs: points arrays (d_k, phi_k), n
// Return M = 0.5 * sum_k( (delta_k^2)/Pk + ln(Pk) )
typedef struct {
    int n;
    double *d;
    double *phi;
} PP;

static double negloglik_alpha(double alpha, const PP *pp, double *outR) {
    int n = pp->n;
    // compute Pk for each
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
    M *= 0.5;
    return M;
}

// ---------- 1D minimizer: Brent (bounded) for alpha in [a,b] ----------
static double brent_minimize(double ax, double bx, double cx, double (*f)(double, const PP*, double*), const PP *pp, double *out_alpha, double tol) {
    // from "Numerical Recipes" style; simpler: do safeguarded parabolic + golden section
    const int ITMAX = 100;
    const double CGOLD = 0.3819660;
    const double ZEPS = 1e-10;
    double a = fmin(ax, cx);
    double b = fmax(ax, cx);
    double x = bx;
    double w = x;
    double v = x;
    double fx = f(x, pp, NULL);
    double fw = fx;
    double fv = fx;
    double d = 0.0, e = 0.0;
    for (int iter=0; iter<ITMAX; ++iter) {
        double xm = 0.5*(a+b);
        double tol1 = tol * fabs(x) + ZEPS;
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
                { // golden
                    e = (x >= xm) ? a - x : b - x;
                    d = CGOLD * e;
                }
            else {
                d = p / q;
                double u = x + d;
                if (u - a < tol2 || b - u < tol2) d = (xm - x >= 0) ? fabs(tol1) : -fabs(tol1);
            }
        } else {
            e = (x >= xm) ? a - x : b - x;
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
    if (out_alpha) *out_alpha = x;
    return fx;
}

// ---------- fit line (ML) for a set of (x,y) points in vehicle frame ----------
// returns R, alpha (rad), cov 2x2, endpoints in vehicle frame (proj)
// method: compute (d,phi) from (x,y), find alpha minimizing negative log-likelihood,
// compute R by Eq(17), then compute numeric Hessian of M(R,alpha) at (R,alpha) and invert -> cov
static int fit_line_ml(const LaneLineInfo *L, double *outR, double *outAlpha, double cov_out[2][2], double *end1x, double *end1y, double *end2x, double *end2y) {
    int n = L->NumOfPoints;
    if (n < 2) return 0;
    double darr[MAX_POINTS], pharr[MAX_POINTS];
    for (int i=0;i<n;i++) {
        double x = L->x[i], y = L->y[i];
        double d = sqrt(x*x + y*y);
        double phi = atan2(y, x);
        darr[i] = d;
        pharr[i] = phi;
    }
    PP pp; pp.n = n; pp.d = darr; pp.phi = pharr;

    // initial alpha guess: PCA principal direction of points (line direction) -> convert to normal-angle alpha
    // compute centroid and covariance matrix for direction
    double cx=0,cy=0;
    for (int i=0;i<n;i++){ cx+=L->x[i]; cy+=L->y[i]; }
    cx/=n; cy/=n;
    double Sxx=0,Syy=0,Sxy=0;
    for (int i=0;i<n;i++){
        double dx = L->x[i]-cx, dy=L->y[i]-cy;
        Sxx += dx*dx; Syy += dy*dy; Sxy += dx*dy;
    }
    // principal direction angle (line direction)
    double theta_dir = 0.5*atan2(2*Sxy, Sxx - Syy); // direction of largest variance
    double alpha0 = norm_ang(theta_dir + PI/2.0); // normal angle
    // bracket search near alpha0
    double a = alpha0 - 1.0;
    double c = alpha0 + 1.0;
    // minimize neg-log-likelihood over alpha
    double alpha_opt;
    brent_minimize(a, alpha0, c, negloglik_alpha, &pp, &alpha_opt, 1e-6);
    // compute R at alpha_opt
    double Rhat;
    negloglik_alpha(alpha_opt, &pp, &Rhat);

    // numeric Hessian of M(R,alpha) wrt R and alpha
    // define function M(R,alpha)
    auto Mfunc = [&](double R, double alpha)->double {
        double M = 0.0;
        for (int k=0;k<n;k++) {
            double Pk = compute_Pk(alpha, pharr[k], darr[k]);
            double delta = darr[k]*cos(alpha - pharr[k]) - R;
            M += (delta*delta)/Pk + log(Pk);
        }
        return 0.5 * M;
    };

    // central differences
    double epsR = 1e-4; // ~0.1 mm
    double epsA = 1e-5; // rad (~0.0006 deg)
    double R = Rhat, A = alpha_opt;

    // partials:
    double M_rr = (Mfunc(R+epsR, A) - 2*Mfunc(R, A) + Mfunc(R-epsR, A)) / (epsR*epsR);
    double M_aa = (Mfunc(R, A+epsA) - 2*Mfunc(R, A) + Mfunc(R, A-epsA)) / (epsA*epsA);
    double M_ra = (Mfunc(R+epsR, A+epsA) - Mfunc(R+epsR, A-epsA) - Mfunc(R-epsR, A+epsA) + Mfunc(R-epsR, A-epsA)) / (4*epsR*epsA);

    double H[2][2];
    H[0][0] = M_rr;
    H[0][1] = M_ra;
    H[1][0] = M_ra;
    H[1][1] = M_aa;

    // invert Hessian -> covariance
    double cov[2][2];
    if (!invert2x2(H, cov)) {
        // fallback: small diagonal
        cov[0][0] = 1e-4; cov[0][1] = 0.0;
        cov[1][0] = 0.0; cov[1][1] = 1e-4;
    } else {
        // covariance = inv(H)
    }
    // If invert2x2 succeeded we have 'cov' as result; if failed, fallback used.
    double covresult[2][2];
    if (isnan(cov[0][0]) || fabs(cov[0][0])<1e-16) {
        covresult[0][0] = 1e-4; covresult[0][1] = 0.0;
        covresult[1][0] = 0.0; covresult[1][1] = 1e-4;
    } else {
        // actually invert2x2 returned inv(H) already; that's our covariance
        covresult[0][0] = cov[0][0];
        covresult[0][1] = cov[0][1];
        covresult[1][0] = cov[1][0];
        covresult[1][1] = cov[1][1];
    }

    // endpoints: project points onto line (in vehicle frame)
    // line normal (R,A) : points on line satisfy x cosA + y sinA = R
    // to get line direction vector: dir = (-sinA, cosA)
    double dirx = -sin(A), diry = cos(A);
    double minproj = 1e12, maxproj = -1e12;
    double cxp=0,cyp=0;
    for (int k=0;k<n;k++) {
        double px = L->x[k], py = L->y[k];
        // projection along dir: proj = dot([px,py], dir)
        double proj = px*dirx + py*diry;
        if (proj < minproj) minproj = proj;
        if (proj > maxproj) maxproj = proj;
    }
    // compute point on line at distance 0: pick point (x0,y0) = (R cosA, R sinA)
    double x0 = R * cos(A), y0 = R * sin(A);
    *end1x = x0 + dirx * minproj;
    *end1y = y0 + diry * minproj;
    *end2x = x0 + dirx * maxproj;
    *end2y = y0 + diry * maxproj;

    // fill outputs
    *outR = R;
    *outAlpha = norm_ang(A);
    cov_out[0][0] = covresult[0][0];
    cov_out[0][1] = covresult[0][1];
    cov_out[1][0] = covresult[1][0];
    cov_out[1][1] = covresult[1][1];
    return 1;
}

// ---------------- transform line (R,alpha) from vehicle frame to global pose ----------------
// R_global = R_local + x*cos(alpha_global) + y*sin(alpha_global)
// alpha_global = alpha_local + pose.theta
// propagate covariance via J (pose assumed exact here)
static void transform_line_to_global(double Rl, double alphal, double covl[2][2], const Pose *pose, MapLine *out) {
    double alphag = norm_ang(alphal + pose->theta);
    double rg = Rl + pose->x * cos(alphag) + pose->y * sin(alphag);
    // Jacobian J = [ dR/dRl , dR/dalphal ; 0 , 1 ]
    // dR/dalphal = d/dalphal [ x cos(alphal+theta) + y sin(...) ] = -x sin(alphag) + y cos(alphag)
    double dR_dalpha = -pose->x * sin(alphag) + pose->y * cos(alphag);
    double J[2][2] = {{1.0, dR_dalpha}, {0.0, 1.0}};
    // covg = J * covl * J^T
    double tmp[2][2] = {{0,0},{0,0}};
    for (int i=0;i<2;i++) for (int j=0;j<2;j++) {
        tmp[i][j]=0;
        for (int k=0;k<2;k++) tmp[i][j] += J[i][k]*covl[k][j];
    }
    double covg[2][2] = {{0,0},{0,0}};
    for (int i=0;i<2;i++) for (int j=0;j<2;j++){
        covg[i][j]=0;
        for (int k=0;k<2;k++) covg[i][j] += tmp[i][k]*J[j][k];
    }
    // fill out
    out->R = rg;
    out->alpha = norm_ang(alphag);
    out->cov[0][0]=covg[0][0]; out->cov[0][1]=covg[0][1];
    out->cov[1][0]=covg[1][0]; out->cov[1][1]=covg[1][1];
}

// chi2 test & merging (map update) ------------------------------------------------
// chi2 = (L1-L2)^T * inv(P1 + P2) * (L1-L2)  (L vector = [R, alpha])
static double chi2_between(const MapLine *a, const MapLine *b) {
    double dR = a->R - b->R;
    double da = norm_ang(a->alpha - b->alpha);
    double dv[2] = {dR, da};
    double S[2][2];
    S[0][0] = a->cov[0][0] + b->cov[0][0];
    S[0][1] = a->cov[0][1] + b->cov[0][1];
    S[1][0] = a->cov[1][0] + b->cov[1][0];
    S[1][1] = a->cov[1][1] + b->cov[1][1];
    double invS[2][2];
    if (!invert2x2(S, invS)) return 1e9;
    double tmp[2];
    mat2_mul_vec(invS, dv, tmp);
    return dv[0]*tmp[0] + dv[1]*tmp[1];
}

// merge two lines by information form (same as paper Eq.(31)-(32) approach)
static void merge_two_lines(const MapLine *A, const MapLine *B, MapLine *out) {
    double S1[2][2] = {{A->cov[0][0], A->cov[0][1]},{A->cov[1][0], A->cov[1][1]}};
    double S2[2][2] = {{B->cov[0][0], B->cov[0][1]},{B->cov[1][0], B->cov[1][1]}};
    double invS1[2][2], invS2[2][2];
    if (!invert2x2(S1, invS1) || !invert2x2(S2, invS2)) {
        // fallback average
        *out = *A;
        out->R = 0.5*(A->R + B->R);
        out->alpha = norm_ang(0.5*(A->alpha + B->alpha));
        out->cov[0][0] = 0.5*(A->cov[0][0] + B->cov[0][0]);
        out->cov[1][1] = 0.5*(A->cov[1][1] + B->cov[1][1]);
        out->id = A->id;
        out->count = A->count + B->count;
    } else {
        double Info[2][2] = {
            {invS1[0][0] + invS2[0][0], invS1[0][1] + invS2[0][1]},
            {invS1[1][0] + invS2[1][0], invS1[1][1] + invS2[1][1]}
        };
        double covm[2][2];
        if (!invert2x2(Info, covm)) {
            // fallback
            *out = *A;
            out->R = 0.5*(A->R + B->R);
            out->alpha = norm_ang(0.5*(A->alpha + B->alpha));
            out->cov[0][0] = 0.5*(A->cov[0][0] + B->cov[0][0]);
            out->cov[1][1] = 0.5*(A->cov[1][1] + B->cov[1][1]);
            out->id = A->id;
            out->count = A->count + B->count;
            return;
        }
        // rhs = invS1 * p1 + invS2 * p2
        double p1[2] = {A->R, A->alpha};
        double p2[2] = {B->R, B->alpha};
        double rhs[2] = {0,0}, tmp[2];
        mat2_mul_vec(invS1, p1, tmp); rhs[0]+=tmp[0]; rhs[1]+=tmp[1];
        mat2_mul_vec(invS2, p2, tmp); rhs[0]+=tmp[0]; rhs[1]+=tmp[1];
        double pmerged[2];
        mat2_mul_vec(covm, rhs, pmerged);
        out->R = pmerged[0];
        out->alpha = norm_ang(pmerged[1]);
        out->cov[0][0] = covm[0][0];
        out->cov[0][1] = covm[0][1];
        out->cov[1][0] = covm[1][0];
        out->cov[1][1] = covm[1][1];
        out->id = A->id; // keep A's id
        out->count = A->count + B->count;
    }
    // endpoints merging: combine endpoints projected onto merged line
    // project each endpoint from A and B onto merged line direction
    double dirx = -sin(out->alpha), diry = cos(out->alpha);
    double x0 = out->R * cos(out->alpha), y0 = out->R * sin(out->alpha);
    double projs[4];
    double ptsx[4] = {A->x1, A->x2, B->x1, B->x2};
    double ptsy[4] = {A->y1, A->y2, B->y1, B->y2};
    for (int i=0;i<4;i++) projs[i] = (ptsx[i]-x0)*dirx + (ptsy[i]-y0)*diry;
    double mn = projs[0], mx = projs[0];
    for (int i=1;i<4;i++){ if (projs[i]<mn) mn=projs[i]; if (projs[i]>mx) mx=projs[i]; }
    out->x1 = x0 + dirx*mn; out->y1 = y0 + diry*mn;
    out->x2 = x0 + dirx*mx; out->y2 = y0 + diry*mx;
}

// merge attempt for incoming line 'cand' into map
static void merge_into_map(const MapLine *cand) {
    // find best match
    double best_chi2 = 1e12;
    int best_idx = -1;
    for (int i=0;i<g_map_count;i++) {
        double c = chi2_between(cand, &g_map[i]);
        if (c < best_chi2) { best_chi2 = c; best_idx = i; }
    }
    double CHI2_THRESH = 5.991; // 95% for dof=2 (paper uses chi2 test)  [oai_citation:5‡robotics.caltech.edu](https://robotics.caltech.edu/~sam/ConferencePapers/ICRA03/LineFit/LineFitMerge_ICRA03.pdf)
    if (best_idx>=0 && best_chi2 < CHI2_THRESH) {
        MapLine merged;
        merge_two_lines(&g_map[best_idx], cand, &merged);
        merged.id = g_map[best_idx].id;
        g_map[best_idx] = merged;
    } else {
        if (g_map_count < MAX_MAP) {
            MapLine m = *cand;
            m.id = g_next_id++;
            if (m.count<=0) m.count = 1;
            g_map[g_map_count++] = m;
        }
    }
}

// ----------------- I/O: write merged.csv & merged.geojson -----------------
static void write_outputs(const char *outdir) {
    // CSV
    char csvpath[1024]; snprintf(csvpath,sizeof(csvpath), "%s/merged.csv", outdir);
    FILE *f = fopen(csvpath,"w");
    if (f) {
        fprintf(f,"id,count,R,alpha_deg,cov_rr,cov_ralpha,cov_alphar,cov_alphaalpha,x1,y1,x2,y2\n");
        for (int i=0;i<g_map_count;i++){
            MapLine *m = &g_map[i];
            fprintf(f,"%d,%d,%.6f,%.6f,%.6e,%.6e,%.6e,%.6e,%.6f,%.6f,%.6f,%.6f\n",
                m->id, m->count, m->R, m->alpha*180.0/M_PI,
                m->cov[0][0], m->cov[0][1], m->cov[1][0], m->cov[1][1],
                m->x1, m->y1, m->x2, m->y2);
        }
        fclose(f);
    } else {
        fprintf(stderr,"Failed to open %s: %s\n", csvpath, strerror(errno));
    }
    // geojson (LineString collection)
    char gjpath[1024]; snprintf(gjpath,sizeof(gjpath), "%s/merged.geojson", outdir);
    FILE *g = fopen(gjpath,"w");
    if (g) {
        fprintf(g,"{\n\"type\": \"FeatureCollection\",\n\"features\": [\n");
        for (int i=0;i<g_map_count;i++){
            MapLine *m = &g_map[i];
            fprintf(g,"  {\n    \"type\":\"Feature\",\n    \"properties\":{\"id\":%d,\"count\":%d,\"R\":%.6f,\"alpha_deg\":%.6f},\n",
                m->id, m->count, m->R, m->alpha*180.0/M_PI);
            fprintf(g,"    \"geometry\":{\"type\":\"LineString\",\"coordinates\":[[%.6f,%.6f],[%.6f,%.6f]]}\n  }%s\n",
                m->x1, m->y1, m->x2, m->y2, (i==g_map_count-1)?"":",");
        }
        fprintf(g,"]\n}\n");
        fclose(g);
    } else {
        fprintf(stderr,"Failed to open %s: %s\n", gjpath, strerror(errno));
    }
}

// ---------------- parsing input CSV (simple) -----------------
// Format: lines with tokens separated by comma
// Frame,frame_idx,dx,dy,dH_deg
// Lane,lane_idx,num_points
// Point,x,y
static int parse_and_run(const char *infile, const char *outdir) {
    FILE *f = fopen(infile,"r");
    if (!f) { fprintf(stderr,"Cannot open %s\n", infile); return 0; }
    char line[4096];
    FrameObs cur; memset(&cur,0,sizeof(cur));
    Pose ego = {0,0,0}; // initial
    int frame_counter = -1;
    while (fgets(line, sizeof(line), f)) {
        // strip newline
        char *p = line; while (*p && (*p==' ' || *p=='\t')) p++;
        if (*p=='\n' || *p=='\0') continue;
        // tokenize
        char *tok = strtok(p, ",\r\n");
        if (!tok) continue;
        if (strcmp(tok, "Frame")==0) {
            // If we had a previous frame with actual content, process it
            if (cur.hasDR || cur.NumOfLane>0) {
                // process frame
                frame_counter++;
                // apply DR to ego pose (DR is delta from previous frame)
                if (cur.hasDR) {
                    double h = ego.theta;
                    double c = cos(h), s = sin(h);
                    double dxg = c * cur.dX - s * cur.dY;
                    double dyg = s * cur.dX + c * cur.dY;
                    ego.x += dxg; ego.y += dyg; ego.theta = norm_ang(ego.theta + cur.dH);
                }
                // for each lane: fit and transform & merge
                for (int L=0; L<cur.NumOfLane; L++) {
                    LaneLineInfo *lli = &cur.lanes[L];
                    double Rloc, alphal;
                    double covl[2][2];
                    double e1x,e1y,e2x,e2y;
                    if (!fit_line_ml(lli, &Rloc, &alphal, covl, &e1x,&e1y,&e2x,&e2y)) continue;
                    MapLine cand; memset(&cand,0,sizeof(cand));
                    cand.count = 1;
                    // transform to global coords
                    transform_line_to_global(& (MapLine){0}, &ego, NULL); // avoid warning (unused)
                    // fill local temp then transform:
                    MapLine tmp; tmp.R = Rloc; tmp.alpha = alphal; tmp.cov[0][0]=covl[0][0]; tmp.cov[0][1]=covl[0][1];
                    tmp.cov[1][0]=covl[1][0]; tmp.cov[1][1]=covl[1][1];
                    tmp.x1 = e1x; tmp.y1 = e1y; tmp.x2=e2x; tmp.y2=e2y;
                    // transform tmp to global
                    MapLine global;
                    transform_line_to_global(tmp.R, tmp.alpha, tmp.cov, &ego, &global);
                    // endpoints: transform local endpoints to global coordinates
                    double h = ego.theta, ch = cos(h), sh = sin(h);
                    global.x1 = ego.x + e1x*ch - e1y*sh;
                    global.y1 = ego.y + e1x*sh + e1y*ch;
                    global.x2 = ego.x + e2x*ch - e2y*sh;
                    global.y2 = ego.y + e2x*sh + e2y*ch;
                    global.count = 1;
                    merge_into_map(&global);
                }
                // after processing frame, write outputs
                write_outputs(outdir);
            }
            // start new frame: parse dx,dy,dH
            char *tok_idx = strtok(NULL, ",\r\n");
            char *tok_dx = strtok(NULL, ",\r\n");
            char *tok_dy = strtok(NULL, ",\r\n");
            char *tok_dh = strtok(NULL, ",\r\n");
            memset(&cur,0,sizeof(cur));
            if (tok_dx && tok_dy && tok_dh) {
                cur.hasDR = 1;
                cur.dX = atof(tok_dx);
                cur.dY = atof(tok_dy);
                cur.dH = atof(tok_dh) * M_PI/180.0; // store rad
            } else {
                cur.hasDR = 0;
            }
        } else if (strcmp(tok, "Lane")==0) {
            char *tok_lane = strtok(NULL, ",\r\n");
            char *tok_n = strtok(NULL, ",\r\n");
            if (!tok_n) continue;
            int lane_idx = atoi(tok_lane);
            int num = atoi(tok_n);
            if (lane_idx<0 || lane_idx>=MAX_LANES) continue;
            cur.NumOfLane = cur.NumOfLane > (lane_idx+1) ? cur.NumOfLane : (lane_idx+1);
            cur.lanes[lane_idx].NumOfPoints = 0;
            // read next num Point lines (assume they follow)
            for (int i=0;i<num;i++) {
                if (!fgets(line, sizeof(line), f)) break;
                char *tt = strtok(line, ",\r\n");
                if (!tt) { i--; continue; }
                if (strcmp(tt,"Point")!=0) { i--; continue; }
                char *tx = strtok(NULL,",\r\n"); char *ty = strtok(NULL,",\r\n");
                if (tx && ty) {
                    int pidx = cur.lanes[lane_idx].NumOfPoints++;
                    cur.lanes[lane_idx].x[pidx] = atof(tx);
                    cur.lanes[lane_idx].y[pidx] = atof(ty);
                } else { i--; }
            }
        } else {
            // ignore unknown token
        }
    }
    // process last frame if present
    if (cur.hasDR || cur.NumOfLane>0) {
        if (cur.hasDR) {
            double h = ego.theta; double c = cos(h), s = sin(h);
            double dxg = c * cur.dX - s * cur.dY;
            double dyg = s * cur.dX + c * cur.dY;
            ego.x += dxg; ego.y += dyg; ego.theta = norm_ang(ego.theta + cur.dH);
        }
        for (int L=0; L<cur.NumOfLane; L++) {
            LaneLineInfo *lli = &cur.lanes[L];
            double Rloc, alphal;
            double covl[2][2];
            double e1x,e1y,e2x,e2y;
            if (!fit_line_ml(lli, &Rloc, &alphal, covl, &e1x,&e1y,&e2x,&e2y)) continue;
            MapLine tmp; tmp.R = Rloc; tmp.alpha = alphal; tmp.cov[0][0]=covl[0][0]; tmp.cov[0][1]=covl[0][1]; tmp.cov[1][0]=covl[1][0]; tmp.cov[1][1]=covl[1][1];
            tmp.x1 = ego.x + e1x*cos(ego.theta) - e1y*sin(ego.theta);
            tmp.y1 = ego.y + e1x*sin(ego.theta) + e1y*cos(ego.theta);
            tmp.x2 = ego.x + e2x*cos(ego.theta) - e2y*sin(ego.theta);
            tmp.y2 = ego.y + e2x*sin(ego.theta) + e2y*cos(ego.theta);
            merge_into_map(&tmp);
        }
        write_outputs(outdir);
    }
    fclose(f);
    return 1;
}

int main(int argc,char**argv) {
    if (argc < 3) {
        fprintf(stderr,"Usage: %s input.csv outdir\n", argv[0]);
        return 1;
    }
    const char *infile = argv[1];
    const char *outdir = argv[2];

    // create outdir if necessary
    char cmd[1024];
    snprintf(cmd,sizeof(cmd),"mkdir -p %s", outdir);
    system(cmd);

    // parse and run
    if (!parse_and_run(infile, outdir)) {
        fprintf(stderr,"Failed processing\n");
        return 1;
    }
    printf("Done. Outputs in %s/merged.csv and %s/merged.geojson\n", outdir, outdir);
    return 0;
}