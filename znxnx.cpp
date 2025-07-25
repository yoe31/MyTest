#include <math.h>
#include <string.h> // memcpy

#define DT 0.05
#define PI 3.14159265358979323846

void mat3x3_mul(double A[3][3], double B[3][3], double out[3][3]) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        out[i][j] = 0.0;
        for (int k = 0; k < 3; ++k) out[i][j] += A[i][k] * B[k][j];
    }
}

void mat3x1_mul(double A[3][3], double B[3], double out[3]) {
    for (int i = 0; i < 3; ++i) {
        out[i] = 0.0;
        for (int k = 0; k < 3; ++k) out[i] += A[i][k] * B[k];
    }
}

void mat3x3_add(double A[3][3], double B[3][3], double out[3][3]) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        out[i][j] = A[i][j] + B[i][j];
    }
}

void mat3x3_sub(double A[3][3], double B[3][3], double out[3][3]) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        out[i][j] = A[i][j] - B[i][j];
    }
}

void mat3x3_transpose(double A[3][3], double out[3][3]) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        out[i][j] = A[j][i];
    }
}

// 역행렬은 OpenCV 대신 수동 구현 (3x3 고정)
int mat3x3_inverse(double A[3][3], double inv[3][3]) {
    double det =
        A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabs(det) < 1e-6) return 0; // singular

    double invDet = 1.0 / det;

    inv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invDet;
    inv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invDet;
    inv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invDet;
    inv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invDet;
    inv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invDet;
    inv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invDet;
    inv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invDet;
    inv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invDet;
    inv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invDet;

    return 1;
}

double sec(double z_r) {
    return 1.0 / cos(z_r);
}

// xEst: 3x1, PEst: 3x3, z: 3x1
void ekf_update(double xEst[3], double PEst[3][3], double z[3], double V, double w, double result[4][3]) {
    double Q[3][3] = {
        {0.01, 0.0, 0.0},
        {0.0, 0.005, 0.0},
        {0.0, 0.0, 0.01}
    };
    double R[3][3] = {
        {10.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double H[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    double pre_x = xEst[0];
    double pre_h = xEst[1];
    double pre_width = xEst[2];

    double dx = V * DT * cos(PI / 2 + w * DT / 2);
    double dy = V * DT * sin(PI / 2 + w * DT / 2);

    double denom = cos(w * DT) - tan(pre_h) * sin(w * DT);
    double x = (pre_x + tan(pre_h) * dy - dx) / denom;
    double h = pre_h + w * DT;
    double width = pre_width;

    double xPred[3] = {x, h, width};

    double A[3][3] = {
        {0}, {0}, {0}
    };
    A[0][0] = 1.0 / denom;
    double sec2 = sec(pre_h) * sec(pre_h);
    double dA_dh = (sec2 * dy * denom - (pre_x + tan(pre_h) * dy - dx) * (-sec2 * sin(w * DT))) / (denom * denom);
    A[0][1] = dA_dh;
    A[1][1] = 1.0;
    A[2][2] = 1.0;

    double At[3][3], APA[3][3], PPred[3][3];
    mat3x3_transpose(A, At);
    mat3x3_mul(A, PEst, APA);
    mat3x3_mul(APA, At, PPred);
    mat3x3_add(PPred, Q, PPred);

    double new_xEst[3], new_PEst[3][3];

    if (z[0] == -1000 || V == 0 || w == 0) {
        memcpy(new_xEst, xPred, sizeof(new_xEst));
        memcpy(new_PEst, PPred, sizeof(new_PEst));
    } else {
        double S[3][3], Ht[3][3], PHT[3][3], Sinv[3][3];
        mat3x3_transpose(H, Ht);
        mat3x3_mul(PPred, Ht, PHT);
        mat3x3_mul(H, PPred, S);
        mat3x3_mul(S, Ht, S);
        mat3x3_add(S, R, S);

        if (!mat3x3_inverse(S, Sinv)) return; // singular

        double K[3][3];
        mat3x3_mul(PHT, Sinv, K);

        double zPred[3], y[3], Ky[3];
        mat3x1_mul(H, xPred, zPred);
        for (int i = 0; i < 3; ++i) y[i] = z[i] - zPred[i];
        mat3x1_mul(K, y, Ky);
        for (int i = 0; i < 3; ++i) new_xEst[i] = xPred[i] + Ky[i];

        double KH[3][3], I_KH[3][3];
        mat3x3_mul(K, H, KH);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        mat3x3_mul(I_KH, PPred, new_PEst);
    }

    // 결과: 4x3 행렬 [xEst (1x3), PEst (3x3)]
    for (int i = 0; i < 3; ++i) result[0][i] = new_xEst[i];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            result[i + 1][j] = new_PEst[i][j];
}
