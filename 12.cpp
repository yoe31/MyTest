bool getLineIntersection(const Point& p1, const Point& p2, const Point& q1, const Point& q2, Point& intersection) {
    double a1 = p2.y - p1.y;
    double b1 = p1.x - p2.x;
    double c1 = a1 * p1.x + b1 * p1.y;

    double a2 = q2.y - q1.y;
    double b2 = q1.x - q2.x;
    double c2 = a2 * q1.x + b2 * q1.y;

    double det = a1 * b2 - a2 * b1;

    if (det == 0) {
        return false; // 평행
    } else {
        double x = (b2 * c1 - b1 * c2) / det;
        double y = (a1 * c2 - a2 * c1) / det;

        // 교차점이 두 선분의 범위 내에 있는지 확인
        if (std::min(p1.x, p2.x) - 1e-8 <= x && x <= std::max(p1.x, p2.x) + 1e-8 &&
            std::min(p1.y, p2.y) - 1e-8 <= y && y <= std::max(p1.y, p2.y) + 1e-8 &&
            std::min(q1.x, q2.x) - 1e-8 <= x && x <= std::max(q1.x, q2.x) + 1e-8 &&
            std::min(q1.y, q2.y) - 1e-8 <= y && y <= std::max(q1.y, q2.y) + 1e-8) {
            intersection = {x, y};
            return true;
        }
    }

    return false;
}

// 박스와 선분이 교차하는지 검사하고 교차점을 반환
bool lineIntersectsBox(const Point& p1, const Point& p2, const Box& box, std::vector<Point>& intersections) {
    // 점이 내부에 있으면 true
    if (box.contains(p1) || box.contains(p2)) {
        // 내부에 있으므로 교차로 간주
        return true;
    }

    // 박스의 네 변
    Point top1 = {box.xmin, box.ymax}, top2 = {box.xmax, box.ymax};
    Point bottom1 = {box.xmin, box.ymin}, bottom2 = {box.xmax, box.ymin};
    Point left1 = {box.xmin, box.ymin}, left2 = {box.xmin, box.ymax};
    Point right1 = {box.xmax, box.ymin}, right2 = {box.xmax, box.ymax};

    Point inter;
    bool intersects = false;

    // 각 변과의 교차 검사
    if (getLineIntersection(p1, p2, top1, top2, inter)) {
        intersections.push_back(inter); intersects = true;
    }
    if (getLineIntersection(p1, p2, bottom1, bottom2, inter)) {
        intersections.push_back(inter); intersects = true;
    }
    if (getLineIntersection(p1, p2, left1, left2, inter)) {
        intersections.push_back(inter); intersects = true;
    }
    if (getLineIntersection(p1, p2, right1, right2, inter)) {
        intersections.push_back(inter); intersects = true;
    }

    return intersects;
}

// 