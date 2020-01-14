//
// Created by Babay on 11.01.2020.
//

#ifndef PROVER_DETECTION_RESULTS_H
#define PROVER_DETECTION_RESULTS_H

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
    double _luminance;

    /**
     * frame's contrast.
     * filled after detection
     */
    double _contrast;

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
            _point(point),
            _shift(shift),
            _defect(defect),
            _circleCoordinates(circleCoordinates),
            _circleCoordinatesLength(circleCoordinatesLength),
            _actualCircleCoordinates(0),
            _luminance(0.0),
            _contrast(0.0) {
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
};

#endif //PROVER_DETECTION_RESULTS_H
