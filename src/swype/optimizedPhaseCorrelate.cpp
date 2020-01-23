/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008-2011, William Lucas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <vector>
#include <opencv2/core.hpp>
#include <swype/common.h>
#include "swype/optimizedPhaseCorrelate.h"

#define ERASE_PEAK_SIZE 11

static void magSpectrums(cv::InputArray _src, cv::OutputArray _dst) {
    cv::Mat src = _src.getMat();
    int depth = src.depth(), cn = src.channels(), type = src.type();
    int rows = src.rows, cols = src.cols;
    int j, k;

    CV_Assert(type == CV_32FC1 || type == CV_32FC2 || type == CV_64FC1 || type == CV_64FC2);

    if (src.depth() == CV_32F)
        _dst.create(src.rows, src.cols, CV_32FC1);
    else
        _dst.create(src.rows, src.cols, CV_64FC1);

    cv::Mat dst = _dst.getMat();
    dst.setTo(0);//Mat elements are not equal to zero by default!

    bool is_1d = (rows == 1 || (cols == 1 && src.isContinuous() && dst.isContinuous()));

    if (is_1d)
        cols = cols + rows - 1, rows = 1;

    int ncols = cols * cn;
    int j0 = cn == 1;
    int j1 = ncols - (cols % 2 == 0 && cn == 1);

    if (depth == CV_32F) {
        const float *dataSrc = src.ptr<float>();
        float *dataDst = dst.ptr<float>();

        size_t stepSrc = src.step / sizeof(dataSrc[0]);
        size_t stepDst = dst.step / sizeof(dataDst[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataSrc += cols - 1, dataDst += cols - 1;
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (rows % 2 == 0)
                    dataDst[(rows - 1) * stepDst] =
                            dataSrc[(rows - 1) * stepSrc] * dataSrc[(rows - 1) * stepSrc];

                for (j = 1; j <= rows - 2; j += 2) {
                    dataDst[j * stepDst] = (float) std::sqrt(
                            (double) dataSrc[j * stepSrc] * dataSrc[j * stepSrc] +
                            (double) dataSrc[(j + 1) * stepSrc] * dataSrc[(j + 1) * stepSrc]);
                }

                if (k == 1)
                    dataSrc -= cols - 1, dataDst -= cols - 1;
            }
        }

        for (; rows--; dataSrc += stepSrc, dataDst += stepDst) {
            if (is_1d && cn == 1) {
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (cols % 2 == 0)
                    dataDst[j1] = dataSrc[j1] * dataSrc[j1];
            }

            for (j = j0; j < j1; j += 2) {
                dataDst[j] = (float) std::sqrt((double) dataSrc[j] * dataSrc[j] +
                                               (double) dataSrc[j + 1] * dataSrc[j + 1]);
            }
        }
    } else {
        const double *dataSrc = src.ptr<double>();
        double *dataDst = dst.ptr<double>();

        size_t stepSrc = src.step / sizeof(dataSrc[0]);
        size_t stepDst = dst.step / sizeof(dataDst[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataSrc += cols - 1, dataDst += cols - 1;
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (rows % 2 == 0)
                    dataDst[(rows - 1) * stepDst] =
                            dataSrc[(rows - 1) * stepSrc] * dataSrc[(rows - 1) * stepSrc];

                for (j = 1; j <= rows - 2; j += 2) {
                    dataDst[j * stepDst] = std::sqrt(
                            dataSrc[j * stepSrc] * dataSrc[j * stepSrc] +
                            dataSrc[(j + 1) * stepSrc] * dataSrc[(j + 1) * stepSrc]);
                }

                if (k == 1)
                    dataSrc -= cols - 1, dataDst -= cols - 1;
            }
        }

        for (; rows--; dataSrc += stepSrc, dataDst += stepDst) {
            if (is_1d && cn == 1) {
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (cols % 2 == 0)
                    dataDst[j1] = dataSrc[j1] * dataSrc[j1];
            }

            for (j = j0; j < j1; j += 2) {
                dataDst[j] = std::sqrt(
                        dataSrc[j] * dataSrc[j] + dataSrc[j + 1] * dataSrc[j + 1]);
            }
        }
    }
}

static void
divSpectrums(cv::InputArray _srcA, cv::InputArray _srcB, cv::OutputArray _dst, int flags,
             bool conjB) {
    cv::Mat srcA = _srcA.getMat(), srcB = _srcB.getMat();
    int depth = srcA.depth(), cn = srcA.channels(), type = srcA.type();
    int rows = srcA.rows, cols = srcA.cols;
    int j, k;

    CV_Assert(type == srcB.type() && srcA.size() == srcB.size());
    CV_Assert(type == CV_32FC1 || type == CV_32FC2 || type == CV_64FC1 || type == CV_64FC2);

    _dst.create(srcA.rows, srcA.cols, type);
    cv::Mat dst = _dst.getMat();

    CV_Assert(dst.data != srcA.data); // non-inplace check
    CV_Assert(dst.data != srcB.data); // non-inplace check

    bool is_1d = (flags & cv::DFT_ROWS) || (rows == 1 || (cols == 1 &&
                                                          srcA.isContinuous() &&
                                                          srcB.isContinuous() &&
                                                          dst.isContinuous()));

    if (is_1d && !(flags & cv::DFT_ROWS))
        cols = cols + rows - 1, rows = 1;

    int ncols = cols * cn;
    int j0 = cn == 1;
    int j1 = ncols - (cols % 2 == 0 && cn == 1);

    if (depth == CV_32F) {
        const float *dataA = srcA.ptr<float>();
        const float *dataB = srcB.ptr<float>();
        float *dataC = dst.ptr<float>();
        float eps = FLT_EPSILON; // prevent div0 problems

        size_t stepA = srcA.step / sizeof(dataA[0]);
        size_t stepB = srcB.step / sizeof(dataB[0]);
        size_t stepC = dst.step / sizeof(dataC[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataA += cols - 1, dataB += cols - 1, dataC += cols - 1;
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (rows % 2 == 0)
                    dataC[(rows - 1) * stepC] =
                            dataA[(rows - 1) * stepA] / (dataB[(rows - 1) * stepB] + eps);
                if (!conjB)
                    for (j = 1; j <= rows - 2; j += 2) {
                        double denom = (double) dataB[j * stepB] * dataB[j * stepB] +
                                       (double) dataB[(j + 1) * stepB] *
                                       dataB[(j + 1) * stepB] + (double) eps;

                        double re = (double) dataA[j * stepA] * dataB[j * stepB] +
                                    (double) dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

                        double im = (double) dataA[(j + 1) * stepA] * dataB[j * stepB] -
                                    (double) dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = (float) (re / denom);
                        dataC[(j + 1) * stepC] = (float) (im / denom);
                    }
                else
                    for (j = 1; j <= rows - 2; j += 2) {

                        double denom = (double) dataB[j * stepB] * dataB[j * stepB] +
                                       (double) dataB[(j + 1) * stepB] *
                                       dataB[(j + 1) * stepB] + (double) eps;

                        double re = (double) dataA[j * stepA] * dataB[j * stepB] -
                                    (double) dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

                        double im = (double) dataA[(j + 1) * stepA] * dataB[j * stepB] +
                                    (double) dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = (float) (re / denom);
                        dataC[(j + 1) * stepC] = (float) (im / denom);
                    }
                if (k == 1)
                    dataA -= cols - 1, dataB -= cols - 1, dataC -= cols - 1;
            }
        }

        for (; rows--; dataA += stepA, dataB += stepB, dataC += stepC) {
            if (is_1d && cn == 1) {
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (cols % 2 == 0)
                    dataC[j1] = dataA[j1] / (dataB[j1] + eps);
            }

            if (!conjB)
                for (j = j0; j < j1; j += 2) {
                    double denom = (double) (dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] +
                                             eps);
                    double re = (double) (dataA[j] * dataB[j] + dataA[j + 1] * dataB[j + 1]);
                    double im = (double) (dataA[j + 1] * dataB[j] - dataA[j] * dataB[j + 1]);
                    dataC[j] = (float) (re / denom);
                    dataC[j + 1] = (float) (im / denom);
                }
            else
                for (j = j0; j < j1; j += 2) {
                    double denom = (double) (dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] +
                                             eps);
                    double re = (double) (dataA[j] * dataB[j] - dataA[j + 1] * dataB[j + 1]);
                    double im = (double) (dataA[j + 1] * dataB[j] + dataA[j] * dataB[j + 1]);
                    dataC[j] = (float) (re / denom);
                    dataC[j + 1] = (float) (im / denom);
                }
        }
    } else {
        const double *dataA = srcA.ptr<double>();
        const double *dataB = srcB.ptr<double>();
        double *dataC = dst.ptr<double>();
        double eps = DBL_EPSILON; // prevent div0 problems

        size_t stepA = srcA.step / sizeof(dataA[0]);
        size_t stepB = srcB.step / sizeof(dataB[0]);
        size_t stepC = dst.step / sizeof(dataC[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataA += cols - 1, dataB += cols - 1, dataC += cols - 1;
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (rows % 2 == 0)
                    dataC[(rows - 1) * stepC] =
                            dataA[(rows - 1) * stepA] / (dataB[(rows - 1) * stepB] + eps);
                if (!conjB)
                    for (j = 1; j <= rows - 2; j += 2) {
                        double denom = dataB[j * stepB] * dataB[j * stepB] +
                                       dataB[(j + 1) * stepB] * dataB[(j + 1) * stepB] + eps;

                        double re = dataA[j * stepA] * dataB[j * stepB] +
                                    dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

                        double im = dataA[(j + 1) * stepA] * dataB[j * stepB] -
                                    dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = re / denom;
                        dataC[(j + 1) * stepC] = im / denom;
                    }
                else
                    for (j = 1; j <= rows - 2; j += 2) {
                        double denom = dataB[j * stepB] * dataB[j * stepB] +
                                       dataB[(j + 1) * stepB] * dataB[(j + 1) * stepB] + eps;

                        double re = dataA[j * stepA] * dataB[j * stepB] -
                                    dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

                        double im = dataA[(j + 1) * stepA] * dataB[j * stepB] +
                                    dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = re / denom;
                        dataC[(j + 1) * stepC] = im / denom;
                    }
                if (k == 1)
                    dataA -= cols - 1, dataB -= cols - 1, dataC -= cols - 1;
            }
        }

        for (; rows--; dataA += stepA, dataB += stepB, dataC += stepC) {
            if (is_1d && cn == 1) {
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (cols % 2 == 0)
                    dataC[j1] = dataA[j1] / (dataB[j1] + eps);
            }

            if (!conjB)
                for (j = j0; j < j1; j += 2) {
                    double denom = dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] + eps;
                    double re = dataA[j] * dataB[j] + dataA[j + 1] * dataB[j + 1];
                    double im = dataA[j + 1] * dataB[j] - dataA[j] * dataB[j + 1];
                    dataC[j] = re / denom;
                    dataC[j + 1] = im / denom;
                }
            else
                for (j = j0; j < j1; j += 2) {
                    double denom = dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] + eps;
                    double re = dataA[j] * dataB[j] - dataA[j + 1] * dataB[j + 1];
                    double im = dataA[j + 1] * dataB[j] + dataA[j] * dataB[j + 1];
                    dataC[j] = re / denom;
                    dataC[j + 1] = im / denom;
                }
        }
    }
}

static void fftShift(cv::InputOutputArray _out) {
    cv::Mat out = _out.getMat();

    if (out.rows == 1 && out.cols == 1) {
        // trivially shifted.
        return;
    }

    std::vector<cv::Mat> planes;
    split(out, planes);

    int xMid = out.cols >> 1;
    int yMid = out.rows >> 1;

    bool is_1d = xMid == 0 || yMid == 0;

    if (is_1d) {
        int is_odd = (xMid > 0 && out.cols % 2 == 1) || (yMid > 0 && out.rows % 2 == 1);
        xMid = xMid + yMid;

        for (size_t i = 0; i < planes.size(); i++) {
            cv::Mat tmp;
            cv::Mat half0(planes[i], cv::Rect(0, 0, xMid + is_odd, 1));
            cv::Mat half1(planes[i], cv::Rect(xMid + is_odd, 0, xMid, 1));

            half0.copyTo(tmp);
            half1.copyTo(planes[i](cv::Rect(0, 0, xMid, 1)));
            tmp.copyTo(planes[i](cv::Rect(xMid, 0, xMid + is_odd, 1)));
        }
    } else {
        int isXodd = out.cols % 2 == 1;
        int isYodd = out.rows % 2 == 1;
        for (size_t i = 0; i < planes.size(); i++) {
            // perform quadrant swaps...
            cv::Mat q0(planes[i], cv::Rect(0, 0, xMid + isXodd, yMid + isYodd));
            cv::Mat q1(planes[i], cv::Rect(xMid + isXodd, 0, xMid, yMid + isYodd));
            cv::Mat q2(planes[i], cv::Rect(0, yMid + isYodd, xMid + isXodd, yMid));
            cv::Mat q3(planes[i], cv::Rect(xMid + isXodd, yMid + isYodd, xMid, yMid));

            if (!(isXodd || isYodd)) {
                cv::Mat tmp;
                q0.copyTo(tmp);
                q3.copyTo(q0);
                tmp.copyTo(q3);

                q1.copyTo(tmp);
                q2.copyTo(q1);
                tmp.copyTo(q2);
            } else {
                cv::Mat tmp0, tmp1, tmp2, tmp3;
                q0.copyTo(tmp0);
                q1.copyTo(tmp1);
                q2.copyTo(tmp2);
                q3.copyTo(tmp3);

                tmp0.copyTo(planes[i](cv::Rect(xMid, yMid, xMid + isXodd, yMid + isYodd)));
                tmp3.copyTo(planes[i](cv::Rect(0, 0, xMid, yMid)));

                tmp1.copyTo(planes[i](cv::Rect(0, yMid, xMid, yMid + isYodd)));
                tmp2.copyTo(planes[i](cv::Rect(xMid, 0, xMid + isXodd, yMid)));
            }
        }
    }

    merge(planes, out);
}

static cv::Point2d
weightedCentroid(cv::InputArray _src, const cv::Point &peakLocation, cv::Size weightBoxSize,
                 double *response) {
    cv::Mat src = _src.getMat();

    int type = src.type();
    CV_Assert(type == CV_32FC1 || type == CV_64FC1);

    int minr = peakLocation.y - (weightBoxSize.height >> 1);
    int maxr = peakLocation.y + (weightBoxSize.height >> 1);
    int minc = peakLocation.x - (weightBoxSize.width >> 1);
    int maxc = peakLocation.x + (weightBoxSize.width >> 1);

    cv::Point2d centroid;
    double sumIntensity = 0.0;

    // clamp the values to min and max if needed.
    if (minr < 0) {
        minr = 0;
    }

    if (minc < 0) {
        minc = 0;
    }

    if (maxr > src.rows - 1) {
        maxr = src.rows - 1;
    }

    if (maxc > src.cols - 1) {
        maxc = src.cols - 1;
    }

    if (type == CV_32FC1) {
        const float *dataIn = src.ptr<float>();
        dataIn += minr * src.cols;
        for (int y = minr; y <= maxr; y++) {
            for (int x = minc; x <= maxc; x++) {
                centroid.x += (double) x * dataIn[x];
                centroid.y += (double) y * dataIn[x];
                sumIntensity += (double) dataIn[x];
            }

            dataIn += src.cols;
        }
    } else {
        const double *dataIn = src.ptr<double>();
        dataIn += minr * src.cols;
        for (int y = minr; y <= maxr; y++) {
            for (int x = minc; x <= maxc; x++) {
                centroid.x += (double) x * dataIn[x];
                centroid.y += (double) y * dataIn[x];
                sumIntensity += dataIn[x];
            }

            dataIn += src.cols;
        }
    }

    if (response)
        *response = sumIntensity;

    sumIntensity += DBL_EPSILON; // prevent div0 problems...

    centroid.x /= sumIntensity;
    centroid.y /= sumIntensity;

    return centroid;
}

static void eraseAndStore(cv::InputOutputArray _src, const cv::Point &peakLocation, int size,
                          double *tempStorage) {
    cv::Mat src = _src.getMat();

    int type = src.type();
    CV_Assert(type == CV_64FC1);

    int s = size >> 1;

    int minr = peakLocation.y - s;
    int maxr = peakLocation.y + s;
    int minc = peakLocation.x - s;
    int maxc = peakLocation.x + s;

    // clamp the values to min and max if needed.
    if (minr < 0) {
        minr = 0;
    }

    if (minc < 0) {
        minc = 0;
    }

    if (maxr > src.rows - 1) {
        maxr = src.rows - 1;
    }

    if (maxc > src.cols - 1) {
        maxc = src.cols - 1;
    }


    auto *dataIn = src.ptr<double>();
    dataIn += minr * src.cols;
    for (int y = minr; y <= maxr; y++) {
        for (int x = minc; x <= maxc; x++) {
            *tempStorage = dataIn[x];
            ++tempStorage;
            dataIn[x] = 0.0;
        }
        dataIn += src.cols;
    }
}

static void
restore(cv::InputOutputArray _src, const cv::Point &peakLocation, int size, double *tempStorage) {
    cv::Mat src = _src.getMat();

    int type = src.type();
    CV_Assert(type == CV_64FC1);

    int s = size >> 1;

    int minr = peakLocation.y - s;
    int maxr = peakLocation.y + s;
    int minc = peakLocation.x - s;
    int maxc = peakLocation.x + s;

    // clamp the values to min and max if needed.
    if (minr < 0) {
        minr = 0;
    }

    if (minc < 0) {
        minc = 0;
    }

    if (maxr > src.rows - 1) {
        maxr = src.rows - 1;
    }

    if (maxc > src.cols - 1) {
        maxc = src.cols - 1;
    }

    auto *dataIn = src.ptr<double>();
    dataIn += minr * src.cols;
    for (int y = minr; y <= maxr; y++) {
        for (int x = minc; x <= maxc; x++) {
            dataIn[x] = *tempStorage;
            ++tempStorage;
        }
        dataIn += src.cols;
    }
}

void copyScaled(cv::InputArray _src, double maxValue, PhaseCorrelateDebugFrame *debugFrame) {

    cv::Mat src = _src.getMat();
    int type = src.type();
    CV_Assert(type == CV_32FC1 || type == CV_64FC1);

    if (debugFrame == nullptr || debugFrame->_size < src.cols * src.rows) {
        return;
    }
    debugFrame->_width = src.cols;
    debugFrame->_height = src.rows;

    auto *_dst = reinterpret_cast<uchar *>( debugFrame->_pData);
    int size = src.cols * src.rows;

    double scale = 255 / pow(maxValue, 0.5);

    if (type == CV_32FC1) {
        const float *dataIn = src.ptr<float>();

        for (int i = 0; i < size; ++i) {
            if (*dataIn < 0) {
                *_dst = 0;
            } else {
                *_dst = static_cast<uchar>(pow(*dataIn, 0.5) * scale);
            }
            ++dataIn;
            ++_dst;
        }
    } else if (type == CV_64FC1) {
        const double *dataIn = src.ptr<double>();
        for (int i = 0; i < size; ++i) {
            if (*dataIn < 0) {
                *_dst = 0;
            } else {
                *_dst = static_cast<uchar>(pow(*dataIn, 0.5) * scale);
            }
            ++dataIn;
            ++_dst;
        }
    }
}

void doPaddedWindow(cv::InputArray _src, cv::OutputArray _dst) {
    cv::UMat src = _src.getUMat();

    CV_Assert(src.type() == CV_32FC1 || src.type() == CV_64FC1);

    int M = cv::getOptimalDFTSize(src.rows);
    int N = cv::getOptimalDFTSize(src.cols);

    if (M != src.rows || N != src.cols) {
        copyMakeBorder(src, _dst, 0, M - src.rows, 0, N - src.cols, cv::BORDER_CONSTANT,
                       cv::Scalar::all(0));
    } else {
        src.copyTo(_dst);
    }
}

void doFFT(cv::InputArray _src, cv::InputArray _paddedWindow, cv::OutputArray _dst) {
    cv::UMat src = _src.getUMat();
    cv::UMat paddedWin = _paddedWindow.getUMat();
    CV_Assert(src.type() == CV_32FC1 || src.type() == CV_64FC1);

    int M = cv::getOptimalDFTSize(src.rows);
    int N = cv::getOptimalDFTSize(src.cols);

    cv::UMat padded, paddedWindowed;

    if (M != src.rows || N != src.cols) {
        copyMakeBorder(src, padded, 0, M - src.rows, 0, N - src.cols, cv::BORDER_CONSTANT,
                       cv::Scalar::all(0));
    } else {
        padded = src;
    }

    if (!paddedWin.empty()) {
        CV_Assert(padded.type() == paddedWin.type());
        CV_Assert(padded.size == paddedWin.size);

        // apply window to image before proceeding...
        multiply(paddedWin, padded, paddedWindowed);
    }

    dft(paddedWindowed, _dst, cv::DFT_REAL_OUTPUT);
}

Peak getPeak(cv::Point &peakLoc, cv::InputArray _src) {
    cv::Mat src = _src.getMat();

    // get the phase shift with sub-pixel accuracy, 5x5 window seems about right here...
    Peak result;
    cv::Point2d t = weightedCentroid(src, peakLoc, cv::Size(5, 5), &(result._weightedCentroid));

    result.x = (double) src.cols / 2.0 - t.x;
    result.y = (double) src.rows / 2.0 - t.y;

    const double *dataIn = src.ptr<double>();
    dataIn += peakLoc.y * src.cols + peakLoc.x;
    result._value = *dataIn;
    return result;
}

void myPhaseCorrelatePart2(cv::InputArray _FFT1, cv::InputArray _FFT2,
                           Peak &peak1, Peak *peak2,
                           PhaseCorrelateDebugFrame *debugFrame) {
    cv::UMat FFT1 = _FFT1.getUMat();
    cv::UMat FFT2 = _FFT2.getUMat();

    CV_Assert(FFT1.type() == FFT2.type());
    CV_Assert(FFT1.type() == CV_32FC1 || FFT1.type() == CV_64FC1);
    CV_Assert(FFT1.size == FFT2.size);

    cv::UMat P, Pm, C;

    mulSpectrums(_FFT1, _FFT2, P, 0, true);

    magSpectrums(P, Pm);
    divSpectrums(P, Pm, C, 0, false); // FF* / |FF*| (phase correlation equation completed here...)

    idft(C, C); // gives us the nice peak shift location...

    fftShift(C); // shift the energy to the center of the frame.

    // locate the highest peak
    cv::Point peakLoc;
    minMaxLoc(C, nullptr, nullptr, nullptr, &peakLoc);
    peak1 = getPeak(peakLoc, C);

    if (peak2 != nullptr) {
        double tmp[ERASE_PEAK_SIZE * ERASE_PEAK_SIZE];
        cv::Point peakLoc2;
        eraseAndStore(C, peakLoc, ERASE_PEAK_SIZE, tmp);
        minMaxLoc(C, nullptr, nullptr, nullptr, &peakLoc2);
        restore(C, peakLoc, ERASE_PEAK_SIZE, tmp);
        *peak2 = getPeak(peakLoc2, C);
    }

    if (debugFrame != nullptr) {
        copyScaled(C, peak1._value, debugFrame);
    }
}
