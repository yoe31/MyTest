#include <math.h>
#include <string.h>  // memcpy

#define STATE_DIM 3

// 행렬 계산 유틸
void mat_mult_3x3(double A[3][3], double B[3][3], double result[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0.0;
            for (int k = 0; k < 3; k++)
                result[i][j] += A[i][k] * B[k][j];
        }
}

void mat_transpose_3x3(double A[3][3], double At[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            At[i][j] = A[j][i];
}

// 단순 역행렬 계산 (diagonal 가정)
void mat_inv_diag_3x3(double A[3][3], double invA[3][3]) {
    for (int i = 0; i < 3; i++) {
        invA[i][i] = (A[i][i] != 0.0) ? 1.0 / A[i][i] : 0.0;
        for (int j = 0; j < 3; j++)
            if (i != j) invA[i][j] = 0.0;
    }
}

// EKF 업데이트
void ekf_update_lane(
    double x[3], double P[3][3], double z[3],
    double dx, double dy, double dtheta,
    double Q[3][3], double R[3][3],
    double out_state[3], double out_cov[3][3]
) {
    // === Prediction Step ===

    // 예측 상태
    double h = x[1];
    double tan_h = tan(h);

    double x_pred[3];
    x_pred[0] = x[0] - dx * tan_h + dy;
    x_pred[1] = x[1] - dtheta;
    x_pred[2] = x[2];  // 폭은 그대로

    // Jacobian A
    double A[3][3] = {
        {1.0, -dx * (1.0 + tan_h * tan_h), 0.0},
        {0.0, 1.0,                        0.0},
        {0.0, 0.0,                        1.0}
    };

    // 공분산 예측: P' = A * P * Aᵗ + Q
    double AP[3][3], At[3][3], APPt[3][3], P_pred[3][3];
    mat_mult_3x3(A, P, AP);
    mat_transpose_3x3(A, At);
    mat_mult_3x3(AP, At, APPt);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P_pred[i][j] = APPt[i][j] + Q[i][j];

    // === Update Step ===

    // H = Identity (관측값 z가 x, h, w 전부 포함)
    double H[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    // S = H * P_pred * Hᵗ + R  → H가 단위행렬이므로 생략 가능
    double S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P_pred[i][j] + R[i][j];

    // K = P_pred * Hᵗ * inv(S) → Hᵗ = I → K = P_pred * inv(S)
    double S_inv[3][3], K[3][3];
    mat_inv_diag_3x3(S, S_inv);
    mat_mult_3x3(P_pred, S_inv, K);

    // y = z - x_pred
    double y[3];
    for (int i = 0; i < 3; i++)
        y[i] = z[i] - x_pred[i];

    // new state = x_pred + K * y
    for (int i = 0; i < 3; i++) {
        out_state[i] = x_pred[i];
        for (int j = 0; j < 3; j++)
            out_state[i] += K[i][j] * y[j];
    }

    // new P = (I - K*H) * P_pred
    double KH[3][3], I_KH[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            KH[i][j] = K[i][j];  // since H = I
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];

    mat_mult_3x3(I_KH, P_pred, out_cov);
}
