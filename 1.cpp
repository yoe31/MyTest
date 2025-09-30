// estimate_rgb2gray_coeff.cpp
// 컴파일 예: g++ -std=c++17 estimate_rgb2gray_coeff.cpp `pkg-config --cflags --libs opencv4` -O2 -o estimate_rgb2gray_coeff

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <original_rgb.png> <gray.png> [stride]\n";
        std::cerr << "  stride: (optional) 샘플링 간격. 기본값 5 (속도/정확도 균형)\n";
        return -1;
    }

    std::string rgb_path = argv[1];
    std::string gray_path = argv[2];
    int stride = 5;
    if (argc >= 4) stride = std::max(1, std::atoi(argv[3]));

    cv::Mat img_rgb = cv::imread(rgb_path, cv::IMREAD_COLOR);       // BGR
    cv::Mat img_gray = cv::imread(gray_path, cv::IMREAD_GRAYSCALE); // single channel

    if (img_rgb.empty() || img_gray.empty()) {
        std::cerr << "이미지를 불러올 수 없습니다. 경로를 확인하세요.\n";
        return -1;
    }

    int gw = img_gray.cols;
    int gh = img_gray.rows;

    // 원본이 작으면 종료
    if (img_rgb.cols < gw || img_rgb.rows < gh) {
        std::cerr << "원본 RGB 이미지가 gray 이미지보다 작습니다. 원본 크기: "
                  << img_rgb.cols << "x" << img_rgb.rows << ", gray 크기: "
                  << gw << "x" << gh << "\n";
        return -1;
    }

    // 잘라내기 설정: 밑부분을 잘라서(아래를 제거) 상단 gh행을 보존
    // 만약 '하단을 보존하고 상단을 잘라서' 하고 싶으면 keepBottom = true로 바꾸세요.
    bool keepBottom = false;
    int crop_x = (img_rgb.cols - gw) / 2; // 가로는 중앙 크롭
    int crop_y = keepBottom ? (img_rgb.rows - gh) : 0; // keepBottom==false면 top(0)부터 gh행을 취함

    cv::Rect roi(crop_x, crop_y, gw, gh);
    cv::Mat rgb_crop = img_rgb(roi).clone(); // 잘린 영역 (BGR)

    // 샘플링으로 메모리/속도 조절
    std::vector<cv::Vec3d> samplesRGB;
    std::vector<double> samplesGray;
    for (int r = 0; r < gh; r += stride) {
        for (int c = 0; c < gw; c += stride) {
            cv::Vec3b bgr = rgb_crop.at<cv::Vec3b>(r, c);
            double R = static_cast<double>(bgr[2]);
            double G = static_cast<double>(bgr[1]);
            double B = static_cast<double>(bgr[0]);
            double gval = static_cast<double>(img_gray.at<uchar>(r, c));
            samplesRGB.emplace_back(R, G, B);
            samplesGray.push_back(gval);
        }
    }

    int N = static_cast<int>(samplesGray.size());
    if (N < 3) {
        std::cerr << "샘플 수가 너무 적습니다.\n";
        return -1;
    }

    // A (Nx3), y (Nx1) - double
    cv::Mat A(N, 3, CV_64F);
    cv::Mat y(N, 1, CV_64F);
    for (int i = 0; i < N; ++i) {
        A.at<double>(i, 0) = samplesRGB[i][0]; // R
        A.at<double>(i, 1) = samplesRGB[i][1]; // G
        A.at<double>(i, 2) = samplesRGB[i][2]; // B
        y.at<double>(i, 0) = samplesGray[i];
    }

    cv::Mat coeff; // 3x1
    bool solved = cv::solve(A, y, coeff, cv::DECOMP_NORMAL); // normal equations (least squares)
    if (!solved) {
        std::cerr << "선형 시스템 해를 구할 수 없습니다.\n";
        return -1;
    }

    double a = coeff.at<double>(0, 0);
    double b = coeff.at<double>(1, 0);
    double c = coeff.at<double>(2, 0);

    // RMS error 계산 (샘플)
    double sse = 0.0;
    for (int i = 0; i < N; ++i) {
        double pred = a * samplesRGB[i][0] + b * samplesRGB[i][1] + c * samplesRGB[i][2];
        double diff = pred - samplesGray[i];
        sse += diff * diff;
    }
    double rmse = std::sqrt(sse / N);

    std::cout << "추정된 변환 식 (Gray ≈ a*R + b*G + c*B):\n";
    std::cout << "  a (R coeff) = " << a << "\n";
    std::cout << "  b (G coeff) = " << b << "\n";
    std::cout << "  c (B coeff) = " << c << "\n";
    std::cout << "샘플 기반 RMSE = " << rmse << " (0..255 스케일)\n";

    // 예측 그레이 이미지 생성 및 저장 (검증용)
    cv::Mat pred_gray(gh, gw, CV_8U);
    for (int r = 0; r < gh; ++r) {
        for (int c = 0; c < gw; ++c) {
            cv::Vec3b bgr = rgb_crop.at<cv::Vec3b>(r, c);
            double val = a * bgr[2] + b * bgr[1] + c * bgr[0]; // R,G,B 순
            val = std::min(255.0, std::max(0.0, val));
            pred_gray.at<uchar>(r, c) = static_cast<uchar>(std::round(val));
        }
    }

    cv::imwrite("rgb_cropped.png", rgb_crop);
    cv::imwrite("pred_gray.png", pred_gray);
    std::cout << "검증 이미지 저장: rgb_cropped.png, pred_gray.png\n";

    return 0;
}