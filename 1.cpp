int intersect_T_with_polyline(
    const Point* C, int C_size,
    const Point T[2],
    Point* out,
    int* is_extended)
{
    int i;

    /* 1. 선분 T vs 폴리라인 */
    for (i = 0; i < C_size - 1; i++) {
        if (segment_intersect(T[0], T[1], C[i], C[i+1], out)) {
            if (is_extended) *is_extended = 0;
            return 1;
        }
    }

    /* 2. 연장된 직선 T vs 폴리라인 */
    for (i = 0; i < C_size - 1; i++) {
        if (line_segment_intersect(T[0], T[1], C[i], C[i+1], out)) {
            if (is_extended) *is_extended = 1;
            return 1;
        }
    }

    return 0;
}