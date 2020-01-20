//
// Created by Babay on 14.01.2020.
//

#ifndef PROVER_OPTIMIZEDPHASECORRELATE_H
#define PROVER_OPTIMIZEDPHASECORRELATE_H

#include <opencv2/core/mat.hpp>
#include "opencvPrecomp.hpp"


void doPaddedWindow(cv::InputArray _src, cv::OutputArray _dst);

void doFFT(cv::InputArray _src, cv::InputArray _paddedWindow, cv::OutputArray _dst);

cv::Point2d myPhaseCorrelatePart2(cv::InputArray _FFT1, cv::InputArray _FFT2,
                                  uchar *phaseDebug = nullptr, int phaseDebugSize = 0,
                                  int *actualDebugSize = nullptr);

#endif //PROVER_OPTIMIZEDPHASECORRELATE_H
