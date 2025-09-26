// --- 추가할 helper 함수 (람다 대신) ---
static double M_of_R_A(double Rv, double Av, const DPP *pp) {
    int n = pp->n;
    double M = 0.0;
    for (int k = 0; k < n; ++k) {
        double Pk = compute_Pk(Av, pp->phi[k], pp->d[k]);
        double delta = pp->d[k] * cos(Av - pp->phi[k]) - Rv;
        M += (delta * delta) / Pk + log(Pk);
    }
    return 0.5 * M;
}