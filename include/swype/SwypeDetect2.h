//
// Created by babay on 29.07.2019.
//

#ifndef PROVER_GAMESWYPEDETECT_H
#define PROVER_GAMESWYPEDETECT_H


#include "SwypeDetectorBase.h"
#include "SwypeCodeDetectorDelta2.h"

class SwypeDetect2 : public SwypeDetectorBase {

public:
    SwypeDetect2() : SwypeDetectorBase() {}

    virtual ~SwypeDetect2() {}

    /**
     * Processes one frame.
     * If we have no swype code set or if a swype code is complete, return immediatelly;
     * Then we calculate brightness and contrast for each frame (windowed).
     * If brightness or contrast is too low, we reject the frame.
     * Then we calculate shift to previous frame. Normalization: the shift by the smallest screen size normalized to 1.
     * If a circle is still not detected, try to detect a circle movement.
     * If a circle is detected, wait for some time and then start detecting the Swype code.
     * For a swype code detection, we multiply shift by speed (1.5 now) and then add sum it.
     * Then we check the Swype Sum Point.
     * When we reach a point with coordinate (0,1), (1,1) etc in expected direction -- swype step is detected,
     * reset Swype Sum Point, move to next swype step,  of finish a swype code.
     * If a Swype Sum Point gets out of detection area -- swype code failed, wait for next circle.
     * If we wait for too much time entering swype code -- it's failed, wait for next circle.
     * @param frame - [in] frame to process
     * @param timestamp - [in] frame timestamp
     * @param state - [out] swype detection state; see (@link DetectorState}
     * @param index - [out] only for state==2, index of recognized digit of the swype-code
     * @param message - [out] additional message, see #Message
     * @param point - [out] detector point coordinated (cant' be null)
     * @param shift - [out] shift to previous frame. can be null
     * @param defect - [out] point's accumulated defect, can be null
     * @param circleCoordinates - [out] circle detector coordinates (if array is long enough)
     *                                        Here are the coordinates of a polyline that is a candidate to be a circle.
     *                                        The polyline is almost closed (distance between start and end point is small).
     *                                        The polyline might be close to a circle or not.
     *                                        The polyline might be large enough to be a circle or not.
     *                                        The polyline is shifted: it's geometrical center is at (0,0) (sum of all coordinates is 0)
     * @param circleCoordinatesLength - [in] length of circleCoordinates (in floats)
     * @param actualCircleCoordinates - actual amount of circle coordinates written
     */
    void GameProcessMat(const cv::Mat &frame, uint timestamp, int &state, int &index, int &message,
                        float *point, float *shift, float *defect,
                        float *circleCoordinates, int circleCoordinatesLength,
                        int &actualCircleCoordinates);

    void GameProcessFrame(const unsigned char *frame_i,
                          uint timestamp, int &state, int &index, int &message,
                          float *point, float *shift, float *defect,
                          float *circleCoordinates, int circleCoordinatesLength,
                          int &actualCircleCoordinates);

    unsigned int TimeToFailMs();

    void
    init(double sourceAspectRatio, int detectorWidth, int detectorHeight, bool relaxed) override;

    void SetFirstStep(char direction, unsigned int maxDuration);

    void AddNextStep(char direction, unsigned int maxDuration);

    void Reset(bool resetSwypeCode);

    void SetSwypeCode(SwypeCode &code);

private:
    SwypeCodeDetectorDelta2 _swypeDetector{};

    int _frameWidth = 0;
    int _frameHeight = 0;
};


#endif //PROVER_GAMESWYPEDETECT_H
