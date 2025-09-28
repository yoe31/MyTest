// merge_line_circle_local.c
// Compile: gcc -std=c11 -O2 merge_line_circle_local.c -lm -o merge_test
// Optionally build with example: gcc -std=c11 -O2 -DMERGE_EXAMPLE merge_line_circle_local.c -lm -o merge_test
//
// Merge() keeps map in ego-local coords; supports LINE and CIRCLE models with AIC selection.
// No file I/O. No lambdas. All in C.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----------------- User-tunable params -----------------
static double sigma_d = 0.02;     // range noise std (m) — adjust for your sensor (example 2cm)
static double sigma_phi = 0.001;  // angular noise std (rad) — adjust (~0.057deg)
static double LINE_CHI2_THRESH = 5.991;  // DOF=2, 95%
static double CIRCLE_CHI2_THRESH = 7.815; // DOF=3, 95%

// ----------------- Types -----------------
typedef struct { double dX, dY, dH; } DR; // dH in radians, CCW+

#define MAX_POINTS 128
#define MAX_LANES  32
#define MAX_MAP    512

typedef struct {
    int NumOfPoints;
    double pts[MAX_POINTS][2]; // local coords: [i][0]=x, [i][1]=y
} LaneLineInfo;

typedef enum { PRIM_LINE = 0, PRIM_CIRCLE = 1 } PrimitiveType;

typedef struct {
    int id;
    PrimitiveType type;
    // For LINE: R, alpha (normal form). For CIRCLE: a,b,r (center + radius).
    double params[3];
    double cov[3][3]; // for LINE only [0..1][0..1] used; for CIRCLE full 3x3
    double x1,y1,x2,y2; // endpoints for visualization (local coords). For circle store arc endpoints (or approximations)
    int count; // number of merges
} MapLine;

// ----------------- Utilities -----------------
static double norm_ang(double a){
    while (a <= -M_PI) a += 2*M_PI;
    while (a > M_PI) a -= 2*M_PI;
    return a;
}

static int invert2x2(const double A[2][2], double out[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabs(det) < 1e-14) return 0;
    out[0][0] =  A[1][1]/det;
    out[1][1] =  A[0][0]/det;
    out[0][1] = -A[0][1]/det;
    out[1][0] = -A[1][0]/det;
    return 1;
}
static int invert3x3(const double A[3][3], double out[3][3]) {
    // analytic inverse for 3x3
    double a11=A[0][0], a12=A[0][1], a13=A[0][2];
    double a21=A[1][0], a22=A[1][1], a23=A[1][2];
    double a31=A[2][0], a32=A[2][1], a33=A[2][2];
    double det = a11*(a22*a33 - a23*a32) - a12*(a21*a33 - a23*a31) + a13*(a21*a32 - a22*a31);
    if (fabs(det) < 1e-16) return 0;
    out[0][0] =  (a22*a33 - a23*a32)/det;
    out[0][1] = -(a12*a33 - a13*a32)/det;
    out[0][2] =  (a12*a23 - a13*a22)/det;
    out[1][0] = -(a21*a33 - a23*a31)/det;
    out[1][1] =  (a11*a33 - a13*a31)/det;
    out[1][2] = -(a11*a23 - a13*a21)/det;
    out[2][0] =  (a21*a32 - a22*a31)/det;
    out[2][1] = -(a11*a32 - a12*a31)/det;
    out[2][2] =  (a11*a22 - a12*a21)/det;
    return 1;
}

static void mat3_mul_vec(const double M[3][3], const double v[3], double out[3]) {
    for (int i=0;i<3;i++) out[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
}
static void mat3_mul_mat(const double A[3][3], const double B[3][3], double C[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++){
        double s=0;
        for (int k=0;k<3;k++) s += A[i][k]*B[k][j];
        C[i][j]=s;
    }
}

// ----------------- Sensor model Pk approx -----------------
// Use practical approx: Pk = sigma_d^2 + (sigma_phi * d_k)^2  (diagonalized simplification)
// This is a reasonable practical approximation (추측입니다).
static double compute_Pk_simple(double d_k) {
    double P = sigma_d*sigma_d + (sigma_phi * d_k) * (sigma_phi * d_k);
    if (P < 1e-12) P = 1e-12;
    return P;
}

// ----------------- LINE: Pfister core functions -----------------
// We implement negloglik_alpha and R computation as in Pfister.
// input: arrays d[k], phi[k], n
typedef struct { int n; const double *d; const double *phi; } DPP;
static double negloglik_alpha_pfister(double alpha, const DPP *pp, double *outR) {
    int n = pp->n;
    double sum_invPk = 0.0;
    double numR = 0.0;
    for (int k=0;k<n;k++) {
        double Pk = compute_Pk_simple(pp->d[k]); // using simple Pk (we drop angular dependence for speed here)
        double invPk = 1.0 / Pk;
        sum_invPk += invPk;
        numR += pp->d[k] * cos(alpha - pp->phi[k]) * invPk;
    }
    double PRR = 1.0 / (sum_invPk + 1e-16);
    double Rhat = PRR * numR;
    if (outR) *outR = Rhat;
    double M = 0.0;
    for (int k=0;k<n;k++) {
        double Pk = compute_Pk_simple(pp->d[k]);
        double delta = pp->d[k]*cos(alpha - pp->phi[k]) - Rhat;
        M += (delta*delta)/Pk + log(Pk);
    }
    return 0.5 * M;
}

static double brent_minimize_double(double ax,double bx,double cx,
    double (*f)(double,const DPP*,double*), const DPP *pp, double *out_x, double tol) {
    const int ITMAX=80; const double CGOLD=0.3819660112501051; const double ZEPS=1e-12;
    double a=fmin(ax,cx), b=fmax(ax,cx);
    double x=bx,w=x,v=x;
    double fx=f(x,pp,NULL), fw=fx, fv=fx;
    double d=0,e=0;
    for (int iter=0; iter<ITMAX; ++iter) {
        double xm = 0.5*(a+b);
        double tol1 = tol*fabs(x)+ZEPS;
        double tol2 = 2.0*tol1;
        if (fabs(x-xm) <= (tol2 - 0.5*(b-a))) break;
        double p=0,q=0,r=0;
        if (fabs(e) > tol1) {
            r = (x-w)*(fx-fv);
            q = (x-v)*(fx-fw);
            p = (x-v)*q - (x-w)*r;
            q = 2.0*(q-r);
            if (q>0) p = -p;
            q = fabs(q);
            double etemp=e; e=d;
            if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x)) {
                e = (x >= xm)? a-x : b-x;
                d = CGOLD * e;
            } else {
                d = p/q;
                double u = x + d;
                if (u - a < tol2 || b - u < tol2) d = (xm - x >= 0) ? fabs(tol1) : -fabs(tol1);
            }
        } else {
            e = (x >= xm)? a-x : b-x;
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
            if (fu <= fw || w==x) { v=w; fv=fw; w=u; fw=fu; }
            else if (fu <= fv || v==x || v==w) { v=u; fv=fu; }
        }
    }
    if (out_x) *out_x = x;
    return fx;
}

// numeric Hessian for line: M(R,A). We'll compute M_of_R_A function.
static double M_of_R_A(const double Rv, const double Av, const DPP *pp) {
    int n = pp->n;
    double M = 0.0;
    for (int k=0;k<n;k++) {
        double Pk = compute_Pk_simple(pp->d[k]);
        double delta = pp->d[k]*cos(Av - pp->phi[k]) - Rv;
        M += (delta*delta)/Pk + log(Pk);
    }
    return 0.5 * M;
}

// fit line per Pfister (uses simplified Pk for speed but follows ML structure)
static int fit_line_pfister_local(const LaneLineInfo *L, double *outR, double *outAlpha, double cov2x2[2][2], double *e1x,double *e1y,double *e2x,double *e2y, double *negloglik_out) {
    int n = L->NumOfPoints;
    if (n < 2) return 0;
    double d[MAX_POINTS], phi[MAX_POINTS];
    for (int i=0;i<n;i++) { double x=L->pts[i][0], y=L->pts[i][1]; d[i]=hypot(x,y); phi[i]=atan2(y,x); }
    DPP pp; pp.n=n; pp.d=d; pp.phi=phi;
    // initial alpha: PCA
    double cx=0,cy=0;
    for (int i=0;i<n;i++){ cx += L->pts[i][0]; cy += L->pts[i][1]; }
    cx /= n; cy /= n;
    double Sxx=0,Syy=0,Sxy=0;
    for (int i=0;i<n;i++){ double dx=L->pts[i][0]-cx, dy=L->pts[i][1]-cy; Sxx+=dx*dx; Syy+=dy*dy; Sxy+=dx*dy; }
    double theta_dir = 0.5 * atan2(2*Sxy, Sxx - Syy);
    double alpha0 = norm_ang(theta_dir + M_PI/2.0);
    double a = alpha0-1.0, b=alpha0, c=alpha0+1.0;
    double alpha_opt;
    brent_minimize_double(a,b,c, negloglik_alpha_pfister, &pp, &alpha_opt, 1e-8);
    double Rhat;
    negloglik_alpha_pfister(alpha_opt, &pp, &Rhat);
    // numeric Hessian of M(R,A)
    double epsR = 1e-5, epsA = 1e-6;
    double R=Rhat, A=alpha_opt;
    double M_rr = (M_of_R_A(R+epsR,A,&pp) - 2.0*M_of_R_A(R,A,&pp) + M_of_R_A(R-epsR,A,&pp)) / (epsR*epsR);
    double M_aa = (M_of_R_A(R,A+epsA,&pp) - 2.0*M_of_R_A(R,A,&pp) + M_of_R_A(R,A-epsA,&pp)) / (epsA*epsA);
    double M_ra = (M_of_R_A(R+epsR,A+epsA,&pp) - M_of_R_A(R+epsR,A-epsA,&pp) - M_of_R_A(R-epsR,A+epsA,&pp) + M_of_R_A(R-epsR,A-epsA,&pp)) / (4.0*epsR*epsA);
    double H[2][2] = {{M_rr, M_ra},{M_ra, M_aa}};
    double cov[2][2];
    if (!invert2x2(H, cov)) { cov[0][0]=1e-4; cov[0][1]=0; cov[1][0]=0; cov[1][1]=1e-4; }
    // compute endpoints by projecting points onto line
    double dirx = -sin(A), diry = cos(A);
    double x0 = R * cos(A), y0 = R * sin(A);
    double mn=1e12, mx=-1e12;
    for (int k=0;k<n;k++){
        double proj = L->pts[k][0]*dirx + L->pts[k][1]*diry;
        if (proj < mn) mn = proj;
        if (proj > mx) mx = proj;
    }
    *e1x = x0 + dirx * mn; *e1y = y0 + diry * mn;
    *e2x = x0 + dirx * mx; *e2y = y0 + diry * mx;
    *outR = Rhat; *outAlpha = norm_ang(A);
    cov2x2[0][0] = cov[0][0]; cov2x2[0][1] = cov[0][1];
    cov2x2[1][0] = cov[1][0]; cov2x2[1][1] = cov[1][1];
    if (negloglik_out) *negloglik_out = M_of_R_A(R,A,&pp);
    return 1;
}

// ----------------- CIRCLE fit (weighted Gauss-Newton) -----------------
// Kasa initial guess then GN with damping, covariance via inv(J^T W J).
// residual r_i = sqrt((xi-a)^2+(yi-b)^2) - r
// Jacobian row: [ - (xi-a)/ri , - (yi-b)/ri , -1 ]
static int circle_kasa_init(const LaneLineInfo *L, double *a0, double *b0, double *r0) {
    int n = L->NumOfPoints; if (n<3) return 0;
    // Kasa method (algebraic) for circle init: solve A*c = b where c=[D,E,F], circle: x^2+y^2 + D x + E y + F = 0
    double Sxx=0,Sxy=0,Syy=0,Sx=0,Sy=0,Sxz=0,Syz=0, Sz=0;
    for (int i=0;i<n;i++){
        double x=L->pts[i][0], y=L->pts[i][1];
        double z = x*x + y*y;
        Sx += x; Sy += y; Sz += z;
        Sxx += x*x; Syy += y*y; Sxy += x*y;
        Sxz += x*z; Syz += y*z;
    }
    // Build normal eqn for [D E F]: [[Sxx Sxy Sx],[Sxy Syy Sy],[Sx Sy n]] * [D E F]^T = -[Sxz Syz Sz]^T
    double M[3][3] = {
        {Sxx, Sxy, Sx},
        {Sxy, Syy, Sy},
        {Sx,  Sy,  (double)n}
    };
    double rhs[3] = {-Sxz, -Syz, -Sz};
    // simple solve 3x3 via Cramer's rule (or LU). Use analytic inverse for robustness.
    double invM[3][3];
    if (!invert3x3(M, invM)) return 0;
    double c[3];
    mat3_mul_vec(invM, rhs, c); // c = invM * rhs
    double D = c[0], E = c[1], F = c[2];
    double a = -D/2.0;
    double b = -E/2.0;
    double r = sqrt(a*a + b*b - F);
    if (!isfinite(r) || r <= 0) return 0;
    *a0 = a; *b0 = b; *r0 = r;
    return 1;
}

static int fit_circle_weighted(const LaneLineInfo *L, double *out_a, double *out_b, double *out_r, double cov3x3[3][3], double *negloglik_out, double *e1x,double *e1y,double *e2x,double *e2y) {
    int n = L->NumOfPoints;
    if (n < 3) return 0;
    double a,b,r;
    if (!circle_kasa_init(L,&a,&b,&r)) {
        // fallback centroid and mean radius
        double cx=0,cy=0; for (int i=0;i<n;i++){ cx+=L->pts[i][0]; cy+=L->pts[i][1]; }
        cx/=n; cy/=n;
        double mr=0; for (int i=0;i<n;i++) mr += hypot(L->pts[i][0]-cx, L->pts[i][1]-cy);
        mr /= n;
        a = cx; b = cy; r = mr;
    }
    // Gauss-Newton with Levenberg damping
    double lambda = 1e-3;
    for (int iter=0; iter<50; ++iter) {
        double JtWJ[3][3] = {{0}}; double JtWe[3] = {0,0,0};
        double cost = 0.0;
        for (int i=0;i<n;i++) {
            double xi = L->pts[i][0], yi = L->pts[i][1];
            double dx = xi - a, dy = yi - b;
            double ri = hypot(dx,dy);
            if (ri < 1e-12) ri = 1e-12;
            double res = ri - r;
            double Pk = compute_Pk_simple(hypot(xi,yi));
            double w = 1.0 / Pk;
            cost += 0.5 * w * res * res;
            double J0 = -(dx/ri), J1 = -(dy/ri), J2 = -1.0;
            // accumulate J^T W J and J^T W e
            JtWJ[0][0] += w * J0 * J0; JtWJ[0][1] += w * J0 * J1; JtWJ[0][2] += w * J0 * J2;
            JtWJ[1][0] += w * J1 * J0; JtWJ[1][1] += w * J1 * J1; JtWJ[1][2] += w * J1 * J2;
            JtWJ[2][0] += w * J2 * J0; JtWJ[2][1] += w * J2 * J1; JtWJ[2][2] += w * J2 * J2;
            JtWe[0] += w * J0 * res; JtWe[1] += w * J1 * res; JtWe[2] += w * J2 * res;
        }
        // Levenberg damping
        for (int d=0; d<3; d++) JtWJ[d][d] *= (1.0 + lambda);
        // solve linear system JtWJ * dx = JtWe  (we seek dx to add to params)
        double invJtWJ[3][3];
        if (!invert3x3(JtWJ, invJtWJ)) {
            // singular -> increase damping and continue
            lambda *= 10;
            if (lambda > 1e6) break;
            continue;
        }
        double dx[3]; mat3_mul_vec(invJtWJ, JtWe, dx);
        // update params
        double an = a + dx[0], bn = b + dx[1], rn = r + dx[2];
        if (rn <= 0) { lambda *= 10; continue; }
        // compute new cost to check improvement
        double newcost = 0.0;
        for (int i=0;i<n;i++){
            double xi=L->pts[i][0], yi=L->pts[i][1];
            double ri = hypot(xi - an, yi - bn); if (ri<1e-12) ri=1e-12;
            double res = ri - rn;
            double Pk = compute_Pk_simple(hypot(xi,yi));
            double w = 1.0 / Pk;
            newcost += 0.5 * w * res * res;
        }
        if (newcost < cost) {
            // accept
            a=an; b=bn; r=rn;
            lambda *= 0.7; if (lambda < 1e-12) lambda = 1e-12;
            // check small update
            if (fabs(dx[0])<1e-6 && fabs(dx[1])<1e-6 && fabs(dx[2])<1e-6) break;
        } else {
            // reject, increase damping
            lambda *= 10;
            if (lambda > 1e8) break;
        }
    }
    // compute final covariance as inv(J^T W J) without damping
    double JtWJf[3][3] = {{0}};
    for (int i=0;i<n;i++){
        double xi=L->pts[i][0], yi=L->pts[i][1];
        double dx = xi - a, dy = yi - b;
        double ri = hypot(dx,dy); if (ri < 1e-12) ri = 1e-12;
        double res = ri - r;
        double Pk = compute_Pk_simple(hypot(xi,yi));
        double w = 1.0 / Pk;
        double J0 = -(dx/ri), J1 = -(dy/ri), J2 = -1.0;
        JtWJf[0][0] += w*J0*J0; JtWJf[0][1] += w*J0*J1; JtWJf[0][2] += w*J0*J2;
        JtWJf[1][0] += w*J1*J0; JtWJf[1][1] += w*J1*J1; JtWJf[1][2] += w*J1*J2;
        JtWJf[2][0] += w*J2*J0; JtWJf[2][1] += w*J2*J1; JtWJf[2][2] += w*J2*J2;
    }
    double cov3[3][3];
    if (!invert3x3(JtWJf, cov3)) {
        // fallback diagonal small
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) cov3[i][j] = (i==j)?1e-4:0.0;
    }
    // endpoints: project points onto circle and take extremes along some angle span (approx)
    // Compute angles of points relative to center and take min/max to get arc endpoints (local coords)
    double amin = 1e12, amax = -1e12;
    for (int i=0;i<n;i++){
        double ang = atan2(L->pts[i][1] - b, L->pts[i][0] - a);
        if (ang < amin) amin = ang;
        if (ang > amax) amax = ang;
    }
    *e1x = a + r * cos(amin); *e1y = b + r * sin(amin);
    *e2x = a + r * cos(amax); *e2y = b + r * sin(amax);
    *out_a = a; *out_b = b; *out_r = r;
    if (negloglik_out) {
        double M=0;
        for (int i=0;i<n;i++){
            double xi=L->pts[i][0], yi=L->pts[i][1];
            double ri = hypot(xi - a, yi - b); if (ri < 1e-12) ri=1e-12;
            double res = ri - r;
            double Pk = compute_Pk_simple(hypot(xi,yi));
            M += (res*res)/Pk + log(Pk);
        }
        *negloglik_out = 0.5 * M;
    }
    // copy cov
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) cov3x3[i][j] = cov3[i][j];
    return 1;
}

// ----------------- Map transform when vehicle moves by dr (map stored in old ego-local, we want new ego-local) -----------------
// For LINE: derived earlier:
// alpha_new = alpha_old - dH
// R_new = R_old - (dX * cos(alpha_old) + dY * sin(alpha_old))
// covariance J = [[1, dX*sin(alpha_old) - dY*cos(alpha_old)], [0,1]] applied on cov
// endpoints: p_new = R(-dH) * (p_old - dr)
// For CIRCLE: center transforms as point, radius unchanged.
// Covariance for circle: J = [[cos(-dH), -sin(-dH), 0],[sin(-dH), cos(-dH),0],[0,0,1]] applied to cov(a,b,r)
static void transform_map_by_dr(MapLine map[], int map_count, const DR *dr) {
    if (!dr) return;
    double dX = dr->dX, dY = dr->dY, dH = dr->dH;
    double ca = cos(-dH), sa = sin(-dH); // for endpoints transform
    for (int i=0;i<map_count;i++){
        MapLine *m = &map[i];
        if (m->type == PRIM_LINE) {
            double alpha_old = m->params[1];
            double alpha_new = norm_ang(alpha_old - dH);
            double R_new = m->params[0] - (dX * cos(alpha_old) + dY * sin(alpha_old));
            // jacobian J (2x2)
            double J[2][2];
            J[0][0] = 1.0;
            J[0][1] = dX * sin(alpha_old) - dY * cos(alpha_old);
            J[1][0] = 0.0;
            J[1][1] = 1.0;
            // cov_old is in m->cov [0..1][0..1]
            double tmp[2][2] = {{0,0},{0,0}}, cov_new2[2][2] = {{0,0},{0,0}};
            for (int r=0;r<2;r++) for (int c=0;c<2;c++) {
                tmp[r][c] = J[r][0]*m->cov[0][c] + J[r][1]*m->cov[1][c];
            }
            for (int r=0;r<2;r++) for (int c=0;c<2;c++) {
                cov_new2[r][c] = tmp[r][0]*J[c][0] + tmp[r][1]*J[c][1];
            }
            // endpoints rotation+translate
            double px1 = m->x1 - dX, py1 = m->y1 - dY;
            double px2 = m->x2 - dX, py2 = m->y2 - dY;
            double nx1 = ca*px1 - sa*py1, ny1 = sa*px1 + ca*py1;
            double nx2 = ca*px2 - sa*py2, ny2 = sa*px2 + ca*py2;
            // commit
            m->params[0] = R_new; m->params[1] = alpha_new;
            m->cov[0][0] = cov_new2[0][0]; m->cov[0][1] = cov_new2[0][1];
            m->cov[1][0] = cov_new2[1][0]; m->cov[1][1] = cov_new2[1][1];
            m->x1 = nx1; m->y1 = ny1; m->x2 = nx2; m->y2 = ny2;
        } else if (m->type == PRIM_CIRCLE) {
            double a = m->params[0], b = m->params[1], r = m->params[2];
            // center transform: p_new = R(-dH)*(p_old - dr)
            double cx_old = a, cy_old = b;
            double tx = cx_old - dX, ty = cy_old - dY;
            double cx_new = ca*tx - sa*ty;
            double cy_new = sa*tx + ca*ty;
            // covariance transform: J = [[cos(-dH), -sin(-dH),0],[sin(-dH),cos(-dH),0],[0,0,1]]
            double J[3][3] = {{ca,-sa,0},{sa,ca,0},{0,0,1}};
            double tmp[3][3] = {{0}}, cov_new[3][3] = {{0}};
            for (int r1=0;r1<3;r1++) for (int c1=0;c1<3;c1++){
                tmp[r1][c1] = 0.0;
                for (int k=0;k<3;k++) tmp[r1][c1] += J[r1][k] * m->cov[k][c1];
            }
            for (int r1=0;r1<3;r1++) for (int c1=0;c1<3;c1++){
                cov_new[r1][c1] = 0.0;
                for (int k=0;k<3;k++) cov_new[r1][c1] += tmp[r1][k] * J[c1][k];
            }
            // transform endpoints similarly
            double px1 = m->x1 - dX, py1 = m->y1 - dY;
            double px2 = m->x2 - dX, py2 = m->y2 - dY;
            double nx1 = ca*px1 - sa*py1, ny1 = sa*px1 + ca*py1;
            double nx2 = ca*px2 - sa*py2, ny2 = sa*px2 + ca*py2;
            m->params[0] = cx_new; m->params[1] = cy_new; m->params[2] = r;
            for (int r1=0;r1<3;r1++) for (int c1=0;c1<3;c1++) m->cov[r1][c1] = cov_new[r1][c1];
            m->x1 = nx1; m->y1 = ny1; m->x2 = nx2; m->y2 = ny2;
        }
    }
}

// ----------------- chi2 and merge for LINE and CIRCLE -----------------
static double chi2_line(const MapLine *A, const MapLine *B) {
    // dv = [R_A - R_B, wrap(alpha_A - alpha_B)]
    double dv[2] = { A->params[0] - B->params[0], norm_ang(A->params[1] - B->params[1]) };
    double S[2][2] = {
        { A->cov[0][0] + B->cov[0][0], A->cov[0][1] + B->cov[0][1] },
        { A->cov[1][0] + B->cov[1][0], A->cov[1][1] + B->cov[1][1] }
    };
    double invS[2][2];
    if (!invert2x2(S, invS)) return 1e12;
    double tmp[2];
    tmp[0] = invS[0][0]*dv[0] + invS[0][1]*dv[1];
    tmp[1] = invS[1][0]*dv[0] + invS[1][1]*dv[1];
    return dv[0]*tmp[0] + dv[1]*tmp[1];
}
static double chi2_circle(const MapLine *A, const MapLine *B) {
    double dv[3] = { A->params[0]-B->params[0], A->params[1]-B->params[1], A->params[2]-B->params[2] };
    double S[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) S[i][j] = A->cov[i][j] + B->cov[i][j];
    double invS[3][3];
    if (!invert3x3(S, invS)) return 1e12;
    double tmp[3]; mat3_mul_vec(invS, dv, tmp);
    return dv[0]*tmp[0] + dv[1]*tmp[1] + dv[2]*tmp[2];
}

// info-form merge helpers
static void merge_line_info(const MapLine *A, const MapLine *B, MapLine *out) {
    double S1[2][2] = {{A->cov[0][0], A->cov[0][1]},{A->cov[1][0], A->cov[1][1]}};
    double S2[2][2] = {{B->cov[0][0], B->cov[0][1]},{B->cov[1][0], B->cov[1][1]}};
    double invS1[2][2], invS2[2][2];
    if (!invert2x2(S1, invS1) || !invert2x2(S2, invS2)) {
        // fallback average
        *out = *A;
        out->params[0] = 0.5*(A->params[0] + B->params[0]);
        out->params[1] = norm_ang(0.5*(A->params[1] + B->params[1]));
        out->cov[0][0] = 0.5*(A->cov[0][0] + B->cov[0][0]);
        out->cov[0][1] = 0.0; out->cov[1][0] = 0.0;
        out->cov[1][1] = 0.5*(A->cov[1][1] + B->cov[1][1]);
        out->count = A->count + B->count;
        out->x1 = 0.5*(A->x1 + B->x1); out->y1 = 0.5*(A->y1 + B->y1);
        out->x2 = 0.5*(A->x2 + B->x2); out->y2 = 0.5*(A->y2 + B->y2);
        return;
    }
    double Info[2][2] = { {invS1[0][0] + invS2[0][0], invS1[0][1] + invS2[0][1]}, {invS1[1][0] + invS2[1][0], invS1[1][1] + invS2[1][1]}};
    double covm[2][2];
    if (!invert2x2(Info, covm)) {
        *out = *A;
        out->params[0] = 0.5*(A->params[0] + B->params[0]);
        out->params[1] = norm_ang(0.5*(A->params[1] + B->params[1]));
        out->cov[0][0] = 0.5*(A->cov[0][0] + B->cov[0][0]);
        out->cov[1][1] = 0.5*(A->cov[1][1] + B->cov[1][1]);
        out->count = A->count + B->count;
        return;
    }
    double p1[2] = {A->params[0], A->params[1]};
    double p2[2] = {B->params[0], B->params[1]};
    double rhs[2] = {0,0}, tmpv[2];
    // rhs = invS1*p1 + invS2*p2
    tmpv[0] = invS1[0][0]*p1[0] + invS1[0][1]*p1[1]; tmpv[1] = invS1[1][0]*p1[0] + invS1[1][1]*p1[1];
    rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1];
    tmpv[0] = invS2[0][0]*p2[0] + invS2[0][1]*p2[1]; tmpv[1] = invS2[1][0]*p2[0] + invS2[1][1]*p2[1];
    rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1];
    // pmerged = covm * rhs
    double pmerged[2];
    pmerged[0] = covm[0][0]*rhs[0] + covm[0][1]*rhs[1];
    pmerged[1] = covm[1][0]*rhs[0] + covm[1][1]*rhs[1];
    *out = *A;
    out->params[0] = pmerged[0];
    out->params[1] = norm_ang(pmerged[1]);
    out->cov[0][0] = covm[0][0]; out->cov[0][1] = covm[0][1]; out->cov[1][0] = covm[1][0]; out->cov[1][1] = covm[1][1];
    out->count = A->count + B->count;
    // endpoints merge: project endpoints of A,B onto merged axis and take extremes
    double dirx = -sin(out->params[1]), diry = cos(out->params[1]);
    double x0 = out->params[0] * cos(out->params[1]), y0 = out->params[0] * sin(out->params[1]);
    double ptsx[4] = {A->x1, A->x2, B->x1, B->x2}, ptsy[4] = {A->y1, A->y2, B->y1, B->y2};
    double mn = 1e12, mx = -1e12;
    for (int i=0;i<4;i++){
        double proj = (ptsx[i]-x0)*dirx + (ptsy[i]-y0)*diry;
        if (proj < mn) mn = proj; if (proj > mx) mx = proj;
    }
    out->x1 = x0 + dirx*mn; out->y1 = y0 + diry*mn; out->x2 = x0 + dirx*mx; out->y2 = y0 + diry*mx;
}

static void merge_circle_info(const MapLine *A, const MapLine *B, MapLine *out) {
    double S1[3][3], S2[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++){ S1[i][j]=A->cov[i][j]; S2[i][j]=B->cov[i][j]; }
    double invS1[3][3], invS2[3][3];
    if (!invert3x3(S1, invS1) || !invert3x3(S2, invS2)) {
        // fallback average
        *out = *A;
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) out->cov[i][j] = 0.5*(S1[i][j] + S2[i][j]);
        out->params[0] = 0.5*(A->params[0] + B->params[0]);
        out->params[1] = 0.5*(A->params[1] + B->params[1]);
        out->params[2] = 0.5*(A->params[2] + B->params[2]);
        out->count = A->count + B->count;
        out->x1 = 0.5*(A->x1 + B->x1); out->y1 = 0.5*(A->y1 + B->y1);
        out->x2 = 0.5*(A->x2 + B->x2); out->y2 = 0.5*(A->y2 + B->y2);
        return;
    }
    double Info[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Info[i][j] = invS1[i][j] + invS2[i][j];
    double covm[3][3];
    if (!invert3x3(Info, covm)) {
        *out = *A; out->count = A->count + B->count; return;
    }
    // rhs = invS1*p1 + invS2*p2
    double p1[3] = {A->params[0], A->params[1], A->params[2]};
    double p2[3] = {B->params[0], B->params[1], B->params[2]};
    double rhs[3] = {0,0,0}, tmpv[3];
    mat3_mul_vec(invS1, p1, tmpv); rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1]; rhs[2]+=tmpv[2];
    mat3_mul_vec(invS2, p2, tmpv); rhs[0]+=tmpv[0]; rhs[1]+=tmpv[1]; rhs[2]+=tmpv[2];
    double pmerged[3]; mat3_mul_vec(covm, rhs, pmerged);
    *out = *A;
    out->params[0] = pmerged[0]; out->params[1] = pmerged[1]; out->params[2] = pmerged[2];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) out->cov[i][j] = covm[i][j];
    out->count = A->count + B->count;
    // endpoints: take extremes of A/B endpoints projected onto radial angles around merged center
    double a = out->params[0], b = out->params[1], r = out->params[2];
    double angs[4];
    for (int i=0;i<4;i++) angs[i] = atan2((i<2?A->y1:A->y2) - b, (i<2?A->x1:A->x2) - a); // rough but ok
    // just set endpoints from A and B average for simplicity
    out->x1 = 0.5*(A->x1 + B->x1); out->y1 = 0.5*(A->y1 + B->y1);
    out->x2 = 0.5*(A->x2 + B->x2); out->y2 = 0.5*(A->y2 + B->y2);
}

// ----------------- AIC model selection -----------------
// For LINE we use negloglik from Pfister M_of_R_A; for CIRCLE we computed negloglik in fit_circle_weighted.
// AIC = 2*k + 2*M  (we use k = number of parameters)
static double compute_AIC_line(int k_params, double negloglik) { return 2.0 * k_params + 2.0 * negloglik; }
static double compute_AIC_circle(int k_params, double negloglik) { return 2.0 * k_params + 2.0 * negloglik; }

// ----------------- Merge API -----------------
// Merge processes one frame:
//  - dr: vehicle delta (in previous ego-local coords) -> transform existing map to new ego-local frame
//  - lanes: detected lane clusters in current ego-local frame (caller must supply measurements in current ego-local)
//  - nlanes: number of LaneLineInfo
//  - map/map_count/max_map: in/out map storage (always ego-local)
static void Merge(const DR *dr, const LaneLineInfo lanes[], int nlanes, MapLine map[], int *map_count, int max_map) {
    // 1) transform existing map to new ego-local frame
    if (dr) transform_map_by_dr(map, *map_count, dr);

    // 2) for each lane, fit both LINE and CIRCLE, compute neg-log-likelihood and AIC, choose model
    for (int li=0; li<nlanes; ++li) {
        const LaneLineInfo *L = &lanes[li];
        if (L->NumOfPoints < 2) continue;

        // Fit LINE
        double Rlin, Alin, cov2[2][2], e1x_l,e1y_l,e2x_l,e2y_l, neglog_line;
        int ok_line = fit_line_pfister_local(L, &Rlin, &Alin, cov2, &e1x_l,&e1y_l,&e2x_l,&e2y_l, &neglog_line);
        double AIC_line = 1e99;
        if (ok_line) {
            AIC_line = compute_AIC_line(2, neglog_line); // k=2
        }

        // Fit CIRCLE
        double a_c, b_c, r_c, cov3[3][3], neglog_circle, e1x_c,e1y_c,e2x_c,e2y_c;
        int ok_circle = fit_circle_weighted(L, &a_c,&b_c,&r_c, cov3, &neglog_circle, &e1x_c,&e1y_c,&e2x_c,&e2y_c);
        double AIC_circle = 1e99;
        if (ok_circle) {
            AIC_circle = compute_AIC_circle(3, neglog_circle); // k=3
        }

        // choose model by AIC (smaller better). If both failed, skip.
        PrimitiveType chosen;
        if (!ok_line && !ok_circle) continue;
        if (ok_line && (!ok_circle || AIC_line <= AIC_circle)) chosen = PRIM_LINE;
        else chosen = PRIM_CIRCLE;

        // build candidate MapLine
        MapLine cand; memset(&cand,0,sizeof(cand));
        cand.count = 1;
        if (chosen == PRIM_LINE) {
            cand.type = PRIM_LINE;
            cand.params[0] = Rlin; cand.params[1] = Alin; cand.params[2] = 0.0;
            // fill cov 3x3 with top-left 2x2 for compatibility
            for (int i=0;i<3;i++) for (int j=0;j<3;j++) cand.cov[i][j] = 0.0;
            cand.cov[0][0] = cov2[0][0]; cand.cov[0][1] = cov2[0][1];
            cand.cov[1][0] = cov2[1][0]; cand.cov[1][1] = cov2[1][1];
            cand.x1 = e1x_l; cand.y1 = e1y_l; cand.x2 = e2x_l; cand.y2 = e2y_l;
        } else {
            cand.type = PRIM_CIRCLE;
            cand.params[0] = a_c; cand.params[1] = b_c; cand.params[2] = r_c;
            for (int i=0;i<3;i++) for (int j=0;j<3;j++) cand.cov[i][j] = cov3[i][j];
            cand.x1 = e1x_c; cand.y1 = e1y_c; cand.x2 = e2x_c; cand.y2 = e2y_c;
        }

        // 3) merge candidate into map if matching same primitive found; else append new
        double best_chi2 = 1e99; int best_idx = -1;
        for (int m=0; m<*map_count; ++m) {
            if (map[m].type != cand.type) continue;
            double c2 = (cand.type == PRIM_LINE) ? chi2_line(&cand, &map[m]) : chi2_circle(&cand, &map[m]);
            if (c2 < best_chi2) { best_chi2 = c2; best_idx = m; }
        }
        int do_merge = 0;
        if (best_idx >= 0) {
            if (cand.type == PRIM_LINE) do_merge = (best_chi2 < LINE_CHI2_THRESH);
            else do_merge = (best_chi2 < CIRCLE_CHI2_THRESH);
        }
        if (do_merge && best_idx >= 0) {
            MapLine merged;
            if (cand.type == PRIM_LINE) merge_line_info(&map[best_idx], &cand, &merged);
            else merge_circle_info(&map[best_idx], &cand, &merged);
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
    } // end lanes loop
}

// ----------------- Minimal example (define MERGE_EXAMPLE to compile example main) -----------------
#ifdef MERGE_EXAMPLE
int main() {
    MapLine map[MAX_MAP]; int map_count = 0;
    // Frame0: two straight lanes
    LaneLineInfo lanes0[2];
    lanes0[0].NumOfPoints = 3;
    lanes0[0].pts[0][0]=10; lanes0[0].pts[0][1]=-3;
    lanes0[0].pts[1][0]=10; lanes0[0].pts[1][1]=0;
    lanes0[0].pts[2][0]=10; lanes0[0].pts[2][1]=3;
    lanes0[1].NumOfPoints = 3;
    lanes0[1].pts[0][0]=20; lanes0[1].pts[0][1]=-3;
    lanes0[1].pts[1][0]=20; lanes0[1].pts[1][1]=0;
    lanes0[1].pts[2][0]=20; lanes0[1].pts[2][1]=3;
    DR dr0 = {0,0,0};
    Merge(&dr0, lanes0, 2, map, &map_count, MAX_MAP);
    printf("After frame0 map_count=%d\n", map_count);

    // Frame1: vehicle moved +1m and sees approx same lanes (or slight curvature)
    DR dr1 = {1.0, 0.0, 0.0};
    LaneLineInfo lanes1[2];
    lanes1[0].NumOfPoints = 3;
    lanes1[0].pts[0][0]=9.05; lanes1[0].pts[0][1]=-3.1;
    lanes1[0].pts[1][0]=8.95; lanes1[0].pts[1][1]=0.05;
    lanes1[0].pts[2][0]=9.02; lanes1[0].pts[2][1]=3.0;
    lanes1[1].NumOfPoints = 4;
    lanes1[1].pts[0][0]=19.1; lanes1[1].pts[0][1]=-2.9;
    lanes1[1].pts[1][0]=19.0; lanes1[1].pts[1][1]=0.1;
    lanes1[1].pts[2][0]=19.02; lanes1[1].pts[2][1]=2.95;
    lanes1[1].pts[3][0]=18.9; lanes1[1].pts[3][1]=3.0;
    Merge(&dr1, lanes1, 2, map, &map_count, MAX_MAP);
    printf("After frame1 map_count=%d\n", map_count);
    for (int i=0;i<map_count;i++){
        MapLine *m=&map[i];
        if (m->type==PRIM_LINE) {
            printf("Line id=%d R=%.3f alpha=%.3fdeg cnt=%d\n", m->id, m->params[0], m->params[1]*180.0/M_PI, m->count);
        } else {
            printf("Circle id=%d center=(%.3f,%.3f) r=%.3f cnt=%d\n", m->id, m->params[0], m->params[1], m->params[2], m->count);
        }
    }
    return 0;
}
#endif