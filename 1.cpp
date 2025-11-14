void drawSpeedBumpDiagonal() {
    int steps = 200;   // 사각형을 촘촘히 샘플링 (대각선 패턴 용)
    float angle = 45.0f * 3.141592f / 180.0f;

    float pattern = patternSize;  // 패턴 간격

    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < steps; j++) {

            float u1 = (float)i     / steps;
            float v1 = (float)j     / steps;
            float u2 = (float)(i+1) / steps;
            float v2 = (float)(j+1) / steps;

            // UV → 실제 사각형 좌표 보간
            Point p1 = {
                quad[0].x*(1-u1)*(1-v1) + quad[1].x*u1*(1-v1) + quad[3].x*(1-u1)*v1 + quad[2].x*u1*v1,
                quad[0].y*(1-u1)*(1-v1) + quad[1].y*u1*(1-v1) + quad[3].y*(1-u1)*v1 + quad[2].y*u1*v1
            };
            Point p2 = {
                quad[0].x*(1-u2)*(1-v1) + quad[1].x*u2*(1-v1) + quad[3].x*(1-u2)*v1 + quad[2].x*u2*v1,
                quad[0].y*(1-u2)*(1-v1) + quad[1].y*u2*(1-v1) + quad[3].y*(1-u2)*v1 + quad[2].y*u2*v1
            };
            Point p3 = {
                quad[0].x*(1-u2)*(1-v2) + quad[1].x*u2*(1-v2) + quad[3].x*(1-u2)*v2 + quad[2].x*u2*v2,
                quad[0].y*(1-u2)*(1-v2) + quad[1].y*u2*(1-v2) + quad[3].y*(1-u2)*v2 + quad[2].y*u2*v2
            };
            Point p4 = {
                quad[0].x*(1-u1)*(1-v2) + quad[1].x*u1*(1-v2) + quad[3].x*(1-u1)*v2 + quad[2].x*u1*v2,
                quad[0].y*(1-u1)*(1-v2) + quad[1].y*u1*(1-v2) + quad[3].y*(1-u1)*v2 + quad[2].y*u1*v2
            };

            float stripeValue = (u1 + v1) / pattern;
            int stripeIndex = ((int)floorf(stripeValue)) & 1;

            if (stripeIndex == 0) glColor3f(1.0f, 0.9f, 0.6f);   // 흰색
            else                  glColor3f(1.0f, 0.6f, 0.1f);   // 주황색

            glBegin(GL_QUADS);
                glVertex2f(p1.x, p1.y);
                glVertex2f(p2.x, p2.y);
                glVertex2f(p3.x, p3.y);
                glVertex2f(p4.x, p4.y);
            glEnd();
        }
    }
}