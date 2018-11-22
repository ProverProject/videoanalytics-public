//
// Created by babay on 11.08.2018.
//

#ifndef PROVER_GAUSSIANWINDOWCALC_H
#define PROVER_GAUSSIANWINDOWCALC_H


#include <cmath>
#include "swype/common.h"

class GaussianWindowCalc {
public:
    void calculate(double *target, int N, double sigma) {
        this->N = N;
        doubleSigma = sigma * 2;
        NMinus1By2 = (N - 1) / 2.0;

        double mul = G(0.5) / (G(-0.5 + N) + G(-0.5 - N));

        for (int i = 0; i < N; ++i) {
            target[i] = G(i) - mul * (G(i + N) + G(i - N));
            //LOGI_NATIVE("gauss %d, %d, %.5f\n", i, N, target[i]);
        }
    }


private:
    double G(double x) {
        double value = (x - NMinus1By2) / doubleSigma;
        return exp(-value * value);
    }

    int N;
    double doubleSigma;
    double NMinus1By2;
};


#endif //PROVER_GAUSSIANWINDOWCALC_H
