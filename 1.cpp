#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 원본 RGB 이미지 (3채널, BGR 순서)
    cv::Mat img_rgb = cv::imread("rgb.png");
    // 대응하는 Gray 이미지 (1채널)
    cv::Mat img_gray = cv::imread("gray.png", cv::IMREAD_GRAYSCALE);

    if (img_rgb.empty() || img_gray.empty()) {
        std::cerr << "이미지를 불러올 수 없습니다." << std::endl;
        return -1;
    }

    if (img_rgb.size() != img_gray.size()) {
        std::cerr << "RGB와 Gray 크기가 다릅니다." << std::endl;
        return -1;
    }

    int rows = img_rgb.rows;
    int cols = img_rgb.cols;

    // 최소제곱을 위한 행렬 준비
    cv::Mat A; // (N x 3)
    cv::Mat y; // (N x 1)

    // 모든 픽셀 사용 시 메모리 부담 크므로 샘플링 가능
    std::vector<cv::Vec3f> samplesX;
    std::vector<float> samplesY;

    for (int r = 0; r < rows; r += 5) {   // 5픽셀 간격 샘플링
        for (int c = 0; c < cols; c += 5) {
            cv::Vec3b rgb = img_rgb.at<cv::Vec3b>(r, c);
            uchar g = img_gray.at<uchar>(r, c);

            // OpenCV는 BGR 순서라서 맞춰줌
            float B = static_cast<float>(rgb[0]);
            float G = static_cast<float>(rgb[1]);
            float R = static_cast<float>(rgb[2]);

            samplesX.push_back(cv::Vec3f(R, G, B));
            samplesY.push_back(static_cast<float>(g));
        }
    }

    // cv::Mat 변환
    A = cv::Mat(samplesX).reshape(1); // N x 3
    y = cv::Mat(samplesY).reshape(1); // N x 1

    // 계수 (3x1)
    cv::Mat coeffs;
    cv::solve(A, y, coeffs, cv::DECOMP_NORMAL);

    double a = coeffs.at<double>(0, 0);
    double b = coeffs.at<double>(1, 0);
    double c = coeffs.at<double>(2, 0);

    std::cout << "추정된 변환 식: Gray = "
              << a << " * R + "
              << b << " * G + "
              << c << " * B" << std::endl;

    return 0;
}