//
// Created by babay on 29.07.2019.
//

#ifndef PROVER_GAMESWYPEDETECT_H
#define PROVER_GAMESWYPEDETECT_H


#include "SwypeDetectorBase.h"
#include "SwypeCodeDetectorDelta2.h"
#include "DetectionResults.h"

/**
 * detects swype code (fixed-length and infinite)
 *
 * First, initialise detector with Init() method.
 * Detector maintains a state. See {@link DetectorState} for details
 * As long as detector initialised, you can send frames to it.
 * If there is not swype code set, detector remains in state 0 = WaitingForCode
 * and ProcessMat() return immediately.
 *
 * If a frame is too dark or has too low contrast, detector
 * will ignore it and return previous state
 *
 * You can set a swype code.
 * To set a fixed-length swype code, call SetSwypeCode(SwypeCode &);
 * As long as you send frames, detector will try to detect circle and swype code.
 * The detection state will update accordingly.
 * When a circle is detected, state becomes 2 = WaitingToStartSwypeCode for 1400 ms = PAUSE_TO_ST3.
 * Then state is changed to 3 = DetectingSwypeCode.
 * Every time a swype step fully detected, index will increase and message will be set to 5 = SwypeStepFinished;
 *
 * If user fails to enter swype code, the detector will return to state 1 = WaitingForCircle
 *
 *
 * You can use an infinite-length swype code.
 * Detector for infinite swype code keeps track only for current and next swype step.
 * So, first you call Reset(true), then AddNextStep() to set current swype step,
 * and then AddNextStep() to set next swype step.
 *
 * When a first swype step is detected (finished), you'll be notified.
 * At that time (that ProcessMat() call)  next swype step becomes current.
 * You should call AddNextStep() right after first step detected,
 * so the detector will always know current step and next step.
 *
 * You can use SetFirstStep() it is similar to call Reset() and AddNextStep();
 *
 */

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
     * @param result - detection results
     */

    void ProcessMat(const cv::Mat &frame, uint timestamp, DetectionResults &result);

    void ProcessFrame(const unsigned char *frame_i, uint timestamp, DetectionResults &result);

    /**
     * return time to fail, in ms
     * when time runs to 0, swype code will be failed
     * if swype-code is finite, return time to fail for entire swype-code
     * if swype-code is infinite, return time to fail for current swype step
     * @return
     */
    unsigned int TimeToFailMs() const;

    /**
     * initialises detector
     * @param sourceAspectRatio - initial video aspect ratio (source video width/height)
     * @param detectorWidth - detector frame width
     * @param detectorHeight - detector frame height
     */
    void
    init(double sourceAspectRatio, int detectorWidth, int detectorHeight) override;

    /**
     * sets first swype step for infinite swype-code
     * (it is similar to call Reset(true) and AddNextStep(direction, maxDuration)
     *
     * @param direction direction 1 .. 8; 1 is down, 3 is left
     * @param maxDuration - available time for step before fail
     */
    void SetFirstStep(char direction, unsigned int maxDuration);

    /**
     * adds one more step for infinite swype code detection
     *
     * @param direction direction 1 .. 8; 1 is down, 3 is left
     * @param maxDuration - available time for step before fail
     */
    void AddNextStep(char direction, unsigned int maxDuration);

    /**
     * reset
     * @param resetSwypeCode - true to reset swype code
     */
    void Reset(bool resetSwypeCode);

    /**
     * sets fixed-length swype code
     * @param code
     */
    void SetSwypeCode(SwypeCode &code);

private:
    void OnIgnoringFrame(uint timestamp, DetectionResults &result);

    SwypeCodeDetectorDelta2 _swypeDetector{};

    int _frameWidth = 0;
    int _frameHeight = 0;
};


#endif //PROVER_GAMESWYPEDETECT_H
