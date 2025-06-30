#include <GL/glut.h>
#include <math.h>

#define PI 3.14159265f

// 화살촉 그리기: base_x, base_y 위치에서 angle 방향
void draw_arrow_head(float base_x, float base_y, float angle_rad) {
    float length = 1.0f;
    float arrow_width = 0.3f;

    float tip_x = base_x + length * cosf(angle_rad);
    float tip_y = base_y + length * sinf(angle_rad);

    float left_x = base_x + arrow_width * cosf(angle_rad + 5.0f * PI / 6.0f);
    float left_y = base_y + arrow_width * sinf(angle_rad + 5.0f * PI / 6.0f);

    float right_x = base_x + arrow_width * cosf(angle_rad - 5.0f * PI / 6.0f);
    float right_y = base_y + arrow_width * sinf(angle_rad - 5.0f * PI / 6.0f);

    glBegin(GL_TRIANGLES);
    glVertex2f(tip_x, tip_y);
    glVertex2f(left_x, left_y);
    glVertex2f(right_x, right_y);
    glEnd();
}

// 직진 화살표: 줄기 + 삼각형
void draw_arrow_straight(float x0, float y0, float x1, float y1) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float angle = atan2f(dy, dx);

    // base: x1에서 2m 뒤
    float base_x = x1 - 2.0f * cosf(angle);
    float base_y = y1 - 2.0f * sinf(angle);

    // 줄기 (shaft)
    glBegin(GL_LINES);
    glVertex2f(base_x - 2.0f * cosf(angle), base_y - 2.0f * sinf(angle)); // 꼬리
    glVertex2f(x1, y1); // 끝점
    glEnd();

    draw_arrow_head(base_x, base_y, angle);
}

// 곡선 화살표: 좌회전(is_left=1) or 우회전(is_left=0)
void draw_arrow_curve(float x0, float y0, float x1, float y1, int is_left) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float angle = atan2f(dy, dx);
    float radius = 2.0f;

    // 회전 중심점: 끝점 기준, 수직방향 반지름만큼
    float center_angle = angle + (is_left ? PI / 2.0f : -PI / 2.0f);
    float cx = x1 + radius * cosf(center_angle);
    float cy = y1 + radius * sinf(center_angle);

    // 원호 각도 범위
    float start_angle = angle + (is_left ? PI : 0);
    float end_angle   = angle + (is_left ? PI / 2.0f : -PI / 2.0f);

    int segments = 20;
    float step = (end_angle - start_angle) / segments;

    float tip_x = 0, tip_y = 0;

    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float theta = start_angle + i * step;
        float x = cx + radius * cosf(theta);
        float y = cy + radius * sinf(theta);
        glVertex2f(x, y);

        if (i == segments) {
            tip_x = x;
            tip_y = y;
        }
    }
    glEnd();

    // 화살촉 base는 tip에서 1m 뒤
    float tip_angle = end_angle;
    float base_x = tip_x - cosf(tip_angle);
    float base_y = tip_y - sinf(tip_angle);
    draw_arrow_head(base_x, base_y, tip_angle);
}

// ---------- OpenGL 기본 설정 ----------
void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    float x0 = -3.0f, y0 = -3.0f;
    float x1 = 0.0f, y1 = 0.0f;

    // 기본 선 그리기 (기준용)
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
    glVertex2f(x0, y0);
    glVertex2f(x1, y1);
    glEnd();

    // 직진 화살표 (빨간색)
    glColor3f(1.0, 0.0, 0.0);
    draw_arrow_straight(x0, y0, x1, y1);

    // 좌회전 화살표 (초록색)
    glColor3f(0.0, 1.0, 0.0);
    draw_arrow_curve(x0, y0, x1, y1, 1);  // is_left = 1

    // 우회전 화살표 (파란색)
    glColor3f(0.0, 0.0, 1.0);
    draw_arrow_curve(x0, y0, x1, y1, 0);  // is_left = 0

    glFlush();
}

void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0); // 검정 배경
    gluOrtho2D(-10, 10, -10, 10);     // 좌표계 설정
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitWindowSize(800, 800);
    glutCreateWindow("OpenGL Arrows: Straight / Left / Right");
    init();
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}