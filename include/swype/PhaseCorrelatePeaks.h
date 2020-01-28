//
// Created by Babay on 22.01.2020.
//

#ifndef PROVER_PHASECORRELATEPEAKS_H
#define PROVER_PHASECORRELATEPEAKS_H

#include "opencvPrecomp.hpp"
#include "settings.h"

class Peak : public cv::Point2d {
public:
    Peak() : cv::Point2d(), _value(0), _weightedCentroid(0) {}

    /**
     * peak value
     */
    double _value;

    /**
     * weighted centroid value
     */
    double _weightedCentroid;
};


class PhaseCorrelatePeaks {
    /**
     * Phase correlation peak
     */
    Peak _peak;

    /**
     * second peak.
     */
    Peak _secondPeak;

public:
    void Set(Peak main, Peak second) {
        _peak = main;
        _secondPeak = second;
    }

    inline bool IsPhaseCorrelateBad() const {
        double wCentrRatio = WeightedCentroidRatio();
        double peakRatio = PeakRatio();
        double  ptc = PeakToCentroid();

        if (wCentrRatio < -0.2 || wCentrRatio > 0.5 || peakRatio > 0.8)
            return true;
        if (fabs(wCentrRatio) < 0.2)
            return false;

        return wCentrRatio < -0.2
                || ptc < 0
                || peakRatio > 0.7
                || fabs(peakRatio * ptc * wCentrRatio) >= 0.03;
    }

    inline double PeakRatio() const {
        return _secondPeak._value / _peak._value;
    }

    inline double WeightedCentroidRatio() const {
        return _secondPeak._weightedCentroid / _peak._weightedCentroid;
    }

    inline double PeakToCentroid() const {
        return _peak._value / _peak._weightedCentroid;
    }

    inline const Peak &getPeak() const {
        return _peak;
    }

    inline const Peak &getSecondPeak() const {
        return _secondPeak;
    }
};


#endif //PROVER_PHASECORRELATEPEAKS_H
