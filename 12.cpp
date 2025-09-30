#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Gray, RGB 이미지 로드 (경로 맞게 수정)
    cv::Mat gray = cv::imread("gray.png", cv::IMREAD_GRAYSCALE); // 1채널
    cv::Mat rgb  = cv::imread("rgb.png", cv::IMREAD_COLOR);      // 3채널

    if (gray.empty() || rgb.empty()) {
        std::cerr << "이미지를 불러올 수 없습니다." << std::endl;
        return -1;
    }

    if (gray.size() != rgb.size()) {
        std::cerr << "Gray와 RGB 크기가 다릅니다." << std::endl;
        return -1;
    }

    // 회귀를 위한 행렬 준비
    int n = gray.rows * gray.cols;
    cv::Mat X(n, 2, CV_32F); // [Gray, 1] (bias 항 포함)
    cv::Mat Y_r(n, 1, CV_32F);
    cv::Mat Y_g(n, 1, CV_32F);
    cv::Mat Y_b(n, 1, CV_32F);

    int idx = 0;
    for (int y = 0; y < gray.rows; y++) {
        for (int x = 0; x < gray.cols; x++) {
            uchar gval = gray.at<uchar>(y, x);
            cv::Vec3b rgbval = rgb.at<cv::Vec3b>(y, x);

            X.at<float>(idx, 0) = static_cast<float>(gval);
            X.at<float>(idx, 1) = 1.0f; // bias

            Y_b.at<float>(idx, 0) = static_cast<float>(rgbval[0]);
            Y_g.at<float>(idx, 0) = static_cast<float>(rgbval[1]);
            Y_r.at<float>(idx, 0) = static_cast<float>(rgbval[2]);

            idx++;
        }
    }

    // 최소자승법으로 계수 계산: W = (X^T X)^(-1) X^T Y
    cv::Mat coeff_b, coeff_g, coeff_r;
    cv::solve(X, Y_b, coeff_b, cv::DECOMP_NORMAL);
    cv::solve(X, Y_g, coeff_g, cv::DECOMP_NORMAL);
    cv::solve(X, Y_r, coeff_r, cv::DECOMP_NORMAL);

    std::cout << "B = " << coeff_b.at<float>(0) << " * Gray + " << coeff_b.at<float>(1) << std::endl;
    std::cout << "G = " << coeff_g.at<float>(0) << " * Gray + " << coeff_g.at<float>(1) << std::endl;
    std::cout << "R = " << coeff_r.at<float>(0) << " * Gray + " << coeff_r.at<float>(1) << std::endl;

    return 0;
}