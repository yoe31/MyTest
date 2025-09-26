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


// before: auto Mfunc = [&](double Rv, double Av)->double { ... };
// after: (use M_of_R_A)

    double epsR = 1e-5;
    double epsA = 1e-6;
    double R = Rhat, A = alpha_opt;

    double M_rr = (M_of_R_A(R+epsR, A, &pp) - 2.0*M_of_R_A(R, A, &pp) + M_of_R_A(R-epsR, A, &pp)) / (epsR*epsR);
    double M_aa = (M_of_R_A(R, A+epsA, &pp) - 2.0*M_of_R_A(R, A, &pp) + M_of_R_A(R, A-epsA, &pp)) / (epsA*epsA);
    double M_ra = (M_of_R_A(R+epsR, A+epsA, &pp) - M_of_R_A(R+epsR, A-epsA, &pp)
                   - M_of_R_A(R-epsR, A+epsA, &pp) + M_of_R_A(R-epsR, A-epsA, &pp))
                   / (4.0 * epsR * epsA);

