//
// Created by babay on 10.08.2018.
//

#include <vector>
#include <opencv2/imgproc.hpp>
#include "swype/Histogram.h"
#include "swype/GaussianWindowCalc.h"

Histogram::Histogram() {
    for (int i = 0; i < 256; ++i) {
        _floatValues[i] = static_cast<float>(i / _f255);
    }
}

void Histogram::Fill(const cv::Mat &mat) {
    Reset();
    EnsureWindow(mat);
    CV_Assert(mat.depth() == CV_8U);

    int nRows = mat.rows;
    int nCols = mat.cols;

    if (mat.isContinuous() && _window.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    int i, j;
    const uchar *pRow;
    const float *pHannRow;
    for (i = 0; i < nRows; ++i) {
        pRow = mat.ptr<uchar>(i);
        pHannRow = _window.ptr<float>(i);
        for (j = 0; j < nCols; ++j) {
            _counters[pRow[j]] += pHannRow[j];
        }
    }
    _avg255 = AvgMinMax(_min, _max);
}

double Histogram::AvgMinMax(int &min, int &max) const {
    min = -1;
    max = 0;
    float sum = 0;
    for (int i = 0; i < 256; ++i) {
        float lumAmount = _counters[i];
        if (lumAmount > 0) {
            sum += i * lumAmount;
            max = i;
            if (min == -1) {
                min = i;
            }
        }
    }
    return sum / _weightSum;
}

double Histogram::RmsContrast() const {
    double avg = _avg255 / _f255;
    double sum = 0;
    for (int i = _min; i <= _max; ++i) {
        double val = _floatValues[i] - avg;
        sum += _counters[i] * val * val;
    }
    return sqrt(sum / _weightSum);
}

void Histogram::EnsureWindow(const cv::Mat &frame) {
    if (_window.empty() || _window.cols != frame.cols || _window.rows != frame.rows) {
        //cv::createHanningWindow(_window, frame.size(), CV_32F);
        CreateContrastWindow(_window, frame.size(), CV_32F);
        //_window.ones(frame.size(), CV_32F);

        _weightSum = 0;

        int nRows = _window.rows;
        int nCols = _window.cols;

        if (_window.isContinuous()) {
            nCols *= nRows;
            nRows = 1;
        }

        int i, j;
        const float *pHann;
        for (i = 0; i < nRows; ++i) {
            pHann = _window.ptr<float>(i);
            for (j = 0; j < nCols; ++j) {
                _weightSum += pHann[j];
            }
        }
    }
}

void Histogram::CreateContrastWindow(cv::OutputArray _dst, cv::Size winSize, int type) {

    CV_Assert(type == CV_32FC1 || type == CV_64FC1);
    CV_Assert(winSize.width > 1 && winSize.height > 1);

    _dst.create(winSize, type);
    cv::Mat dst = _dst.getMat();

    int rows = dst.rows, cols = dst.cols;
    GaussianWindowCalc calc;

    std::vector<double> wc(cols);
    std::vector<double> wr(rows);

    calc.calculate(wc.data(), cols, cols * 0.1);
    calc.calculate(wr.data(), rows, rows * 0.1);

    if (dst.depth() == CV_32F) {
        for (int i = 0; i < rows; i++) {
            float *dstData = dst.ptr<float>(i);
            double wrValue = wr[i];
            for (int j = 0; j < cols; j++)
                dstData[j] = (float) (wrValue * wc[j]);
        }
    } else {
        for (int i = 0; i < rows; i++) {
            double *dstData = dst.ptr<double>(i);
            double wrValue = wr[i];
            for (int j = 0; j < cols; j++)
                dstData[j] = wrValue * wc[j];
        }
    }
}