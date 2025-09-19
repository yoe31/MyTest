// main.cpp
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <iostream>

int main()
{
    // --- 가정: 가로(width)=1280, 세로(height)=576 (추측입니다) ---
    const int width  = 1280;
    const int height = 576;

    // 실제로는 아래 imagearry를 사용자가 채운 데이터로 교체하세요.
    // 아래는 테스트용으로 그라데이션을 채워 보여주는 예시입니다.
    static uint8_t imagearry[width * height];
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            imagearry[y * width + x] = static_cast<uint8_t>((x * 255) / (width - 1));
        }
    }

    // imagearry 포인터를 이용해 OpenCV Mat 생성
    // (만약 buffer의 row stride(피치)가 width와 다르면 마지막 인자에 stride를 넣으세요)
    cv::Mat img_gray(height, width, CV_8UC1, imagearry);

    // 원본 버퍼가 이후 변경될 가능성이 있으면 clone()으로 안전하게 복사하세요
    cv::Mat display = img_gray.clone();

    // 회색조 창
    cv::namedWindow("Grayscale", cv::WINDOW_AUTOSIZE);
    cv::imshow("Grayscale", display);

    // (선택) 컬러맵 적용해서 보기 (가독성 좋음)
    cv::Mat color;
    cv::applyColorMap(display, color, cv::COLORMAP_JET);
    cv::namedWindow("Color (JET)", cv::WINDOW_AUTOSIZE);
    cv::imshow("Color (JET)", color);

    std::cout << "이미지 창에서 아무 키를 누르면 종료됩니다.\n";
    cv::waitKey(0);

    // (선택) 파일로 저장
    cv::imwrite("out_gray.png", display);
    cv::imwrite("out_color.png", color);

    return 0;
}