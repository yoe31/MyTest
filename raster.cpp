#include <iostream>
#include <vector>
#include <algorithm> // sort, unique
#include <cmath>

#define H 162
#define W 38
#define GRID 0.8

struct Point {
    double x;
    double y;
};

// thickness: 양의 정수. thickness=2 -> 2x2 block (4 pixels)
// 반환: 중복 제거된 (row, col) 리스트
std::vector<std::pair<int,int>> worldToPixel(const std::vector<Point>& points, int thickness = 2)
{
    const double x_min = -49.6;
    const double x_max = 80.0;
    const double y_min = -15.2;
    const double y_max = 15.2;

    std::vector<std::pair<int,int>> pixelList;
    if (thickness <= 0) return pixelList;

    for (const auto& p : points)
    {
        // 범위 체크
        if (p.x < x_min || p.x > x_max || p.y < y_min || p.y > y_max)
            continue;

        // 차량좌표 -> 픽셀 좌표
        int row = static_cast<int>((x_max - p.x) / GRID);
        int col = static_cast<int>((y_max - p.y) / GRID);

        // thickness에 따라 시작좌표 계산
        // start = center - floor((thickness-1)/2)
        int start_r = row - ( (thickness - 1) / 2 );
        int start_c = col - ( (thickness - 1) / 2 );

        for (int dr = 0; dr < thickness; ++dr)
        {
            for (int dc = 0; dc < thickness; ++dc)
            {
                int rr = start_r + dr;
                int cc = start_c + dc;
                if (rr >= 0 && rr < H && cc >= 0 && cc < W)
                {
                    pixelList.emplace_back(rr, cc);
                }
            }
        }
    }

    // 중복 제거 (정렬 후 unique)
    std::sort(pixelList.begin(), pixelList.end());
    pixelList.erase(std::unique(pixelList.begin(), pixelList.end()), pixelList.end());

    return pixelList;
}

int main()
{
    std::vector<Point> points = {
        {0.0, 0.0},
        {10.0, 5.0},
        {-20.0, -10.0}
    };

    auto pixels = worldToPixel(points, 2); // thickness = 2 => 2x2 block

    std::cout << "Pixel coordinates (row, col):\n";
    for (const auto &rc : pixels)
        std::cout << "(" << rc.first << ", " << rc.second << ")\n";

    return 0;
}