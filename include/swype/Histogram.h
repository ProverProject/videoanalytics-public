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

    double getAvg() const {
        return _avg255;
    }

    void Configure(float minLuminanse, float minContrast) {
        _minLuminanse = minLuminanse;
        _minContrast = minContrast;
    }

    bool IsLuminanceLow() {
        return _avg255 < _minLuminanse;
    }

    bool IsContrastLow() {
        double contrast = RmsContrast();
        //LOGI_NATIVE("avg: %.2ff, contrast: %.6f", _avg255, contrast);
        return contrast < _minContrast;
    }

private:
    float _floatValues[256];
    float _counters[256];

    double _weightSum;

    /**
     * average luminosity; 0-255
     */
    double _avg255;

    /**
     * minimal luminosity present in image
     */
    int _min;

    /*
     * maximal luminosity present in image
     */
    int _max;
    const double _f255 = 255;

    cv::Mat _window;

    float _minContrast = 0;
    float _minLuminanse = 0;

    /**
     * reset collected data
     */
    inline void Reset() {
        memset(_counters, 0, sizeof(_counters));
    }

    double AvgMinMax(int &min, int &max) const;

    void EnsureWindow(const cv::Mat &frame);

    void CreateContrastWindow(const cv::_OutputArray &_dst, cv::Size winSize, int type);
};


#endif //PROVER_HISTOGRAM_H
