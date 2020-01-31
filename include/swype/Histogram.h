//
// Created by babay on 10.08.2018.
//

#ifndef PROVER_HISTOGRAM_H
#define PROVER_HISTOGRAM_H


#include <cstring>
#include <opencv2/core/mat.hpp>
#include "swype/common.h"

class Histogram {

public:
    Histogram();

    /**
     * fills histogram data with contents of mat
     * @param mat
     */
    void Fill(const cv::Mat &mat);

    /**
     * calculate RMS-contrast
     * @return contrast
     */
    double RmsContrast() const;

    void Configure(float minLuminanse, float minContrast) {
        _minLuminanse = minLuminanse;
        _minContrast = minContrast;
    }

    bool IsLuminanceLow() {
        return _avg < _minLuminanse;
    }

    bool IsContrastLow() {
        return _contrast < _minContrast;
    }

    float GetAverageLuminance() {
        return _avg;
    }

    float GetContrast() {
        return _contrast;
    }

private:
    float _floatValues[256];
    float _counters[256];

    double _weightSum;

    /**
     * average luminosity; 0-1
     */
    float _avg;

    float _contrast;

    /**
     * minimal luminosity present in image
     */
    int _min;

    /*
     * maximal luminosity present in image
     */
    int _max;

    cv::Mat _window;

    float _minContrast;

    float _minLuminanse;

    /**
     * reset collected data
     */
    inline void Reset() {
        memset(_counters, 0, sizeof(_counters));
    }

    double AvgMinMax(int &min, int &max) const;

    void EnsureWindow(const cv::Mat &frame);

    void CreateContrastWindow(const cv::_OutputArray &_dst, cv::Size winSize, int type);

    static void CalculateGaussianWindow(double *target, int N, double sigma);
};


#endif //PROVER_HISTOGRAM_H
