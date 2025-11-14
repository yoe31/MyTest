void drawSpeedBumpDiagonal() 
{
    int steps = 200;                 // 해상도
    float patternSize = 0.1f;        // 패턴 1개 폭 (현실 ~10cm)
    
    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < steps; j++) {

            float u1 = (float)i     / steps;
            float v1 = (float)j     / steps;
            float u2 = (float)(i+1) / steps;
            float v2 = (float)(j+1) / steps;

            // UV → 월드좌표 (bilinear)
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

            // 45° 사선 방향 거리
            float d = u1 + v1;

            // 패턴 내 위치 (0~1)
            float local = fmodf(d, patternSize) / patternSize;

            // ★ 정확한 과속방지턱 패턴: 하나의 사선 내 색 두 개(흰/주)
            if (local < 0.5f)
                glColor3f(1.0f, 1.0f, 1.0f);      // 흰색
            else
                glColor3f(1.0f, 0.6f, 0.1f);      // 주황색

            glBegin(GL_QUADS);
                glVertex2f(p1.x, p1.y);
                glVertex2f(p2.x, p2.y);
                glVertex2f(p3.x, p3.y);
                glVertex2f(p4.x, p4.y);
            glEnd();
        }
    }
}