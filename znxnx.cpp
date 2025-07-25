// 초기화
double xEst[3] = {0.0, 0.0, 0.0};           // 추정 상태 (x, h, width)
double PEst[3][3] = {                       // 추정 공분산
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
};
double z[3] = {10.0, 0.2, 1.0};             // 측정값
double V = 1.0, w = 0.1;                    // 속도, 각속도

double result[4][3];                        // EKF 결과: xEst + PEst

// EKF 업데이트 수행
ekf_update(xEst, PEst, z, V, w, result);

// xEst, PEst 다시 추출
for (int i = 0; i < 3; ++i) {
    xEst[i] = result[0][i];                // 첫 번째 행이 상태값
}

for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
        PEst[i][j] = result[i + 1][j];     // 나머지 행이 공분산 행렬
    }
}

// 결과값 변수에 저장
double new_meas_x = xEst[0];
double new_meas_h = xEst[1];
double new_lane_width = xEst[2];

double last_meas_x = new_meas_x;
double last_meas_h = new_meas_h;
double last_lane_width = new_lane_width;
