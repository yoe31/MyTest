#include <GL/glut.h>
#include <math.h>

typedef struct {
    float x, y;
} Point;

Point quad[4];

// 패턴 크기
float stripeSize = 1.0f;

// 회전 각도 (단위: degree)
float angleDeg = 45.0f;

// 로컬 좌표 계산용
float dot(float ax, float ay, float bx, float by) {
    return ax * bx + ay * by;
}

void vec2(Point a, Point b, float* vx, float* vy) {
    *vx = b.x - a.x;
    *vy = b.y - a.y;
}

// 사각형 로컬 좌표(u, v) 계산
void computeUV(Point p, float* u, float* v) {

    float ux, uy, vx, vy;
    vec2(quad[0], quad[1], &ux, &uy);  // U축
    vec2(quad[0], quad[3], &vx, &vy);  // V축

    float lenU = ux * ux + uy * uy;
    float lenV = vx * vx + vy * vy;

    float px = p.x - quad[0].x;
    float py = p.y - quad[0].y;

    *u = dot(px, py, ux, uy) / lenU;
    *v = dot(px, py, vx, vy) / lenV;
}

// 사각형 내부 스트라이프 렌더링
void drawSpeedBump()
{
    float minX = fmin(fmin(quad[0].x, quad[1].x), fmin(quad[2].x, quad[3].x));
    float maxX = fmax(fmax(quad[0].x, quad[1].x), fmax(quad[2].x, quad[3].x));
    float minY = fmin(fmin(quad[0].y, quad[1].y), fmin(quad[2].y, quad[3].y));
    float maxY = fmax(fmax(quad[0].y, quad[1].y), fmax(quad[2].y, quad[3].y));

    float step = 0.02f;

    float rad = angleDeg * 3.1415926535f / 180.0f;
    float cs = cos(rad);
    float sn = sin(rad);

    for(float x=minX; x < maxX; x+=step) {
        for(float y=minY; y < maxY; y+=step) {

            Point p = {x, y};
            float u, v;
            computeUV(p, &u, &v);

            // 회전 적용 (u', v')
            float uprime =  u * cs - v * sn;
            // float vprime =  u * sn + v * cs;  // 필요 시 사용

            int stripeIdx = (int)floor(uprime / stripeSize);

            if(stripeIdx % 2 == 0)
                glColor3f(1.0, 1.0, 1.0);   // 흰색
            else
                glColor3f(1.0, 0.5, 0.0);   // 주황색

            glBegin(GL_QUADS);
                glVertex2f(x, y);
                glVertex2f(x+step, y);
                glVertex2f(x+step, y+step);
                glVertex2f(x, y+step);
            glEnd();
        }
    }
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    drawSpeedBump();
    glutSwapBuffers();
}

int main(int argc, char** argv)
{
    quad[0] = (Point){0,0};
    quad[1] = (Point){6,0};
    quad[2] = (Point){6,3};
    quad[3] = (Point){0,3};

    stripeSize = 1.0f;
    angleDeg   = 135.0f;  // 원하는 각도

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(800,600);
    glutCreateWindow("Speed bump");

    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}