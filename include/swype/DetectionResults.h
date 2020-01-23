//
// Created by Babay on 11.01.2020.
//

#ifndef PROVER_DETECTIONRESULTS_H
#define PROVER_DETECTIONRESULTS_H

#include "optimizedPhaseCorrelate.h"

struct DetectionResults {
    /**
     *  swype detection state; see (@link DetectorState}
     *  filled after detection
     */
    int _state;

    /**
     * only for state==2, index of recognized digit of the swype-code
     * filled after detection
     */
    int _index;

    /**
     * additional message, see {@link Message} in common.h
     * filled after detection
     */
    int _message;

    /**
     * time to fail, in ms
     * when time runs to 0, swype code will be failed
     * if swype-code is finite, it's time to fail for entire swype-code
     * if swype-code is infinite, it's time to fail for current swype step
     */
    int _timeToFailMs;

    /**
     * an array for detector point coordinated. an array of 2. can be null.
     * filled after detection
     */
    float *_point;

    /**
     * an array for shift to previous frame. an array of 2. can be null.
     * filled after detection
     */
    float *_shift;

    /**
     * an array for point's accumulated defect. an array of 2. can be null.
     * filled after detection
     */
    float *_defect;

    /**
     * an array for circle detector coordinates (if array is long enough)
     * filled after detection
     */
    float *_circleCoordinates;

    /**
     * length of circleCoordinates (in floats)
     * filled at initialisation
     */
    const int _circleCoordinatesLength;

    /**
     * actual amount of circle coordinates written
     * filled after detection
     */
    int _actualCircleCoordinates;

    /**
     * frame's luminance. 0-1.
     * filled after detection
     */
    float _luminance;

    /**
     * frame's contrast.
     * filled after detection
     */
    float _contrast;

    /**
     * phase correlation peaks info
     */
    PhaseCorrelatePeaks _peaks;

    PhaseCorrelateDebugFrame *_correlatedFrame;

    /**\
     * @param point - an array for detector point coordinated. an array of 2. can be null.
     * @param shift - an array for shift to previous frame. an array of 2. can be null.
     * @param defect - an array for point's accumulated defect. an array of 2. can be null.
     * @param circleCoordinates - an array for circle detector coordinates (if array is long enough)
     *                                        Here are the coordinates of a polyline that is a candidate to be a circle.
     *                                        The polyline is almost closed (distance between start and end point is small).
     *                                        The polyline might be close to a circle or not.
     *                                        The polyline might be large enough to be a circle or not.
     *                                        The polyline is shifted: it's geometrical center is at (0,0) (sum of all coordinates is 0)
     * @param circleCoordinatesLength - [in] length of circleCoordinates (in floats)
     */
    DetectionResults(float *point, float *shift, float *defect,
                     float *circleCoordinates, int circleCoordinatesLength) :
            _state(DetectorState::WaitingForCode),
            _index(0),
            _message(Message::None),
            _timeToFailMs(0),
            _point(point),
            _shift(shift),
            _defect(defect),
            _circleCoordinates(circleCoordinates),
            _circleCoordinatesLength(circleCoordinatesLength),
            _actualCircleCoordinates(0),
            _luminance(0.0),
            _contrast(0.0),
            _correlatedFrame(nullptr) {
        if (_point != nullptr) {
            _point[0] = 0;
            _point[1] = 0;
        }

        if (_shift != nullptr) {
            _shift[0] = 0;
            _shift[1] = 0;
        }

        if (_defect != nullptr) {
            _defect[0] = 0;
            _defect[1] = 0;
        }
    }

    /**
     * fills _shift value array if it's not null
     * @param shift
     */
    void SetShift(VectorExplained &shift) {
        if (_shift != nullptr) {
            _shift[0] = static_cast<float>(shift.X());
            _shift[1] = static_cast<float>(shift.Y());
        }
    }

    inline bool IsLowLuminance() {
        return _luminance < MIN_BRIGHTNESS_CLIENT;
    }

    inline bool IsContrastLow() {
        return _contrast < MIN_CONTRAST_CLIENT;
    }

    inline bool IsPhaseCorrelateBad() {
        return _peaks.IsPhaseCorrelateBad();
    }
};

#endif //PROVER_DETECTIONRESULTS_H
