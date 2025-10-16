#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#define H 162
#define W 38
#define GRID 0.8

struct Point {
    double x;
    double y;
};

struct Line {
    Point start;
    Point end;
};

// 차량좌표계 선들을 픽셀 좌표 리스트로 변환
std::vector<std::pair<int,int>> worldLineToPixel(
    const std::vector<Line>& lines,
    int thickness = 2,
    double sample_step = 0.1)
{
    const double x_min = -49.6;
    const double x_max = 80.0;
    const double y_min = -15.2;
    const double y_max = 15.2;

    std::vector<std::pair<int,int>> pixelList;

    for (const auto& line : lines)
    {
        double dx = line.end.x - line.start.x;
        double dy = line.end.y - line.start.y;
        double len = std::sqrt(dx*dx + dy*dy);
        if (len < 1e-6) continue; // 너무 짧은 선은 무시

        int n_steps = static_cast<int>(len / sample_step);
        for (int i = 0; i <= n_steps; ++i)
        {
            double t = static_cast<double>(i) / n_steps;
            double x = line.start.x + t * dx;
            double y = line.start.y + t * dy;

            if (x < x_min || x > x_max || y < y_min || y > y_max)
                continue;

            int row = static_cast<int>((x_max - x) / GRID);
            int col = static_cast<int>((y_max - y) / GRID);

            // 두께 반영 (2×2 block)
            int start_r = row - ((thickness - 1) / 2);
            int start_c = col - ((thickness - 1) / 2);

            for (int dr = 0; dr < thickness; ++dr)
            {
                for (int dc = 0; dc < thickness; ++dc)
                {
                    int rr = start_r + dr;
                    int cc = start_c + dc;
                    if (rr >= 0 && rr < H && cc >= 0 && cc < W)
                        pixelList.emplace_back(rr, cc);
                }
            }
        }
    }

    // 중복 제거
    std::sort(pixelList.begin(), pixelList.end());
    pixelList.erase(std::unique(pixelList.begin(), pixelList.end()), pixelList.end());

    return pixelList;
}

int main()
{
    std::vector<Line> lines = {
        {{-10.0, 0.0}, {10.0, 0.0}},   // 수평선
        {{0.0, -5.0}, {0.0, 5.0}},     // 수직선
        {{-10.0, -10.0}, {10.0, 10.0}} // 대각선
    };

    auto pixels = worldLineToPixel(lines, 2, 0.1);

    std::cout << "Pixel coordinates (row, col):\n";
    for (const auto& rc : pixels)
        std::cout << "(" << rc.first << ", " << rc.second << ")\n";

    return 0;
}