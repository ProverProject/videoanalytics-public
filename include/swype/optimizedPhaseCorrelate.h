//
// Created by Babay on 14.01.2020.
//

#ifndef PROVER_OPTIMIZEDPHASECORRELATE_H
#define PROVER_OPTIMIZEDPHASECORRELATE_H

#include <opencv2/core/mat.hpp>
#include "opencvPrecomp.hpp"
#include "PhaseCorrelatePeaks.h"

struct PhaseCorrelateDebugFrame {
    char *const _pData;
    const int _size;
    int _width;
    int _height;

    PhaseCorrelateDebugFrame(char *pData, const int size) : _pData(pData), _size(size),
                                                            _width(0), _height(0) {}
};


void doPaddedWindow(cv::InputArray _src, cv::OutputArray _dst);

void doFFT(cv::InputArray _src, cv::InputArray _paddedWindow, cv::OutputArray _dst);

void myPhaseCorrelatePart2(cv::InputArray _FFT1, cv::InputArray _FFT2,
                           PhaseCorrelatePeaks &peaks,
                           PhaseCorrelateDebugFrame *debugFrame = nullptr);

#endif //PROVER_OPTIMIZEDPHASECORRELATE_H
