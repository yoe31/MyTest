#include <GL/glut.h>
#include <math.h>

#define PI 3.14159265f

void draw_arrow(float x, float y, float angle_rad) {
    float length = 1.0f;
    float arrow_width = 0.3f;

    // 화살표 끝점
    float tip_x = x + length * cosf(angle_rad);
    float tip_y = y + length * sinf(angle_rad);

    // 좌우 날개
    float left_x = x + arrow_width * cosf(angle_rad + 5.0f * PI / 6.0f);
    float left_y = y + arrow_width * sinf(angle_rad + 5.0f * PI / 6.0f);

    float right_x = x + arrow_width * cosf(angle_rad - 5.0f * PI / 6.0f);
    float right_y = y + arrow_width * sinf(angle_rad - 5.0f * PI / 6.0f);

    glBegin(GL_TRIANGLES);
    glVertex2f(tip_x, tip_y);
    glVertex2f(left_x, left_y);
    glVertex2f(right_x, right_y);
    glEnd();
}

// 직진 화살표
void draw_arrow_straight(float x0, float y0, float x1, float y1) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float angle = atan2f(dy, dx);

    float base_x = x1 - cosf(angle); // 1m 뒤
    float base_y = y1 - sinf(angle);
    draw_arrow(base_x, base_y, angle);
}

// 좌회전 화살표
void draw_arrow_left(float x0, float y0, float x1, float y1) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float angle = atan2f(dy, dx) + PI / 2.0f; // +90도 회전

    float base_x = x1 - cosf(angle);
    float base_y = y1 - sinf(angle);
    draw_arrow(base_x, base_y, angle);
}

// 우회전 화살표
void draw_arrow_right(float x0, float y0, float x1, float y1) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float angle = atan2f(dy, dx) - PI / 2.0f; // -90도 회전

    float base_x = x1 - cosf(angle);
    float base_y = y1 - sinf(angle);
    draw_arrow(base_x, base_y, angle);
}

// 예시 렌더링 코드
void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    float x0 = 0.0f, y0 = 0.0f;
    float x1 = 3.0f, y1 = 0.0f;

    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
    glVertex2f(x0, y0);
    glVertex2f(x1, y1);
    glEnd();

    glColor3f(1.0, 0.0, 0.0);
    draw_arrow_straight(x0, y0, x1, y1);

    glColor3f(0.0, 1.0, 0.0);
    draw_arrow_left(x0, y0, x1, y1);

    glColor3f(0.0, 0.0, 1.0);
    draw_arrow_right(x0, y0, x1, y1);

    glFlush();
}

void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    gluOrtho2D(-5, 5, -5, 5);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitWindowSize(600, 600);
    glutCreateWindow("Arrow Drawing");
    init();
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}