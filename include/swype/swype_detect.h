// @author Viacheslav Voronin
//
// I. Swype-code domain definition
//
// Swype-code is defined in the domain of 9 virtual points placed in 3x3 grid.
// Points are numbered from 1 to 9 starting from the top left corner (point 1)
// from left to right, then from top downward.
//
// II. Use cases
//
// 1. Approval of video file being captured. In this case we don't know
//    swype-code to be entered. The algorithm just should detect circular
//    motion "okey, prover" and notify about it. Only after that the swype-code
//    will be provided, and the algorithm should start to detect it.
// 2. Verification of swype-code presence in a given file. In this case a
//    swype-code is known and the goal is to find the code in the video stream.
//    The algorithm should detect circular motion "okey, prover" and then
//    search for the code within some predefined amount of time depending on
//    the code length (for example, 2 seconds for each code digit).
//
// It's suggested to implement both use cases in the single algorithm.
//
// III. General information
//
// Input: sequence of video frames (parameters? minimum possible resolution?
// color space etc...) with FPS no less than N (what's the minimum?)
//
// The algorithm behaves like a state machine, state transitions happen after
// processing a successive video frame. The following states are suggested
// (a state may have an associated parameter or set of parameters - given in
// brackets):
// * S0 (no): awaiting for circular motion "okey, prover";
// * S1 (no): circular motion is detected, the algorithm awaits for setting
//   a swype-code (this state is available for the first use-case only, when
//   the swype-code was not set yet; transition to S2 occurs after setting
//   a swype-code);
// * S2 (no): circular motion is detected, we have a swype-code and we are
//   waiting for some time: user should stabilize camera before entering swype-code
// * S3 (index of a last recognized symbol in the code, coordinates of the
//   current point of the image for trajectory visualization): entering
//   a swype-code is in progess.
// * S4 (no): swype-code entering is finished.
//
// IV. Algorithm description
//
// Initialization. The following parameters are provided: frame resolution,
// fps, swype-code (optional).
//
// Algorithm workflow:
// * Start from state S0. Right after recognition of circular motion transit to
//   one of the state:
//   - S1, if the swype-code was not specified at initialization. Immediately
//     after setting a swype-code (separate function that may be called
//     asynchronously) the algorithm transits into state S2 after receiving
//     a successive video frame.
//   - S2 (0), if the swype-code was given at initialization.
// * start to recognize a trajectory, return S2(i) and trajectory point
//   coordinates for visualization with every given video frame.
// * If the trajectory during the input fall into zone of another digit (not
//   the expected one), fall back to state S0. If swype-code was not set at
//   initialization, it should be reset too.
// * If entering a swype-code is not finished within 2*N seconds interval
//   (N denotes number of characters in the swype-code), fall back to state S0
//   and reset the swype-code provided it wasn't set at initialization. Time
//   calculation shall use FPS specified at initialization.
// * If entering a swype-code is finished, transit to state S3.
//
// V. Expected API
//
// /** @brief Initialization
//  *  @param width video frame width (pixels)
//  *  @param height video frame height (pixels)
//  *  @param fps frames per second
//  *  @param swype optional swype-code
//  */
// void init(
//    int         width,
//    int         height,
//    int         fps,
//    std::string swype="");
//
// /** @brief Set swype-code. Only for the case when the swype-code was not set
//  *         via init()
//  *  @param swype swype-code
//  */
// void setSwype(std::string swype);
//
// /** @brief Process single video frame
//  *  @param frame buffer with video frame data
//  *  @param state [out] state S
//  *  @param index [out] only for state==2, index of recognized digit of the
//  *         swype-code
//  *  @param x, y [out] only for state==2, trajectory coordinates for
//  *         visualization
//  */
// void processFrame(
//    const char *frame,
//    int        &state,
//    int        &index,
//    int        &x,
//    int        &message,
//    int        &y);
//
//
//
// /**
//  * @param frame_i frame data, only Luminance channel
//  * @param width_i frame width
//  * @param height_i frame height
//  * @param timestamp timestamp of frame relative to video start
//  * @param state [out] state S
//  * @param index [out] only for state==2, index of recognized digit of the
//  *         swype-code
//  * @param x, y [out] only for state==2, trajectory coordinates for
//  *         visualization
//  * @param debug - some debug data
//  */
//  void processFrame(
//     const unsigned char *frame_i,
//     int width_i,
//     int height_i,
//     uint timestamp,
//     int &state,
//     int &index,
//     int &x,
//     int &y,
//     int &debug);


#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include "opencv2/core/ocl.hpp"
#include "swype/VectorExplained.h"
#include "swype/SwypeStepDetector.h"
#include "swype/CircleDetector.h"
#include "swype/DetectorState.h"
#include "swype/ShiftDetector.h"
#include "swype/SwypeCodeDetectorDelta.h"
#include "swype/Histogram.h"
#include "SwypeDetectorBase.h"

class SwypeDetect : public SwypeDetectorBase {
public:

    SwypeDetect();

    virtual ~SwypeDetect();

    /**
     * set swype code
     * @param swype
     */
    void setSwype(std::string swype);

    /**
     * process one frame, old version
     * @param frame
     * @param timestamp
     * @param state
     * @param index
     * @param x
     * @param y
     * @param debug
     */
    void processMat(const cv::Mat &frame, const uint timestamp, int &state, int &index, int &x,
                    int &y, int &message, int &debug);


    /**
     * same as processMatExt, but receives frame as an array of pixels
     */
    void processFrameExt(const unsigned char *frame_i, int width_i, int height_i, uint timestamp,
                         int &state, int &index, int &message,
                         float *resultCoordinates, int resultCoordinatesLength,
                         int &actualResultCoordinates);

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
     * @param resultCoordinates - [out] output coordinates;
     *                           [0],[1] - (x,y) - Swype Sum Point coordinates
     *                           [2],[3] - (sx,sy) - current frame shift to previous (if array is long enough)
     *                           [4],[5] - (dx,dy) - Swype Sum Point coordinates defect (collected since position reset), (if array is long enough)
     *                           [6],[7],... - coordinates of circle detection (if array is long enough)
     *                                        Here are the coordinates of a polyline that is a candidate to be a circle.
     *                                        The polyline is almost closed (distance between start and end point is small).
     *                                        The polyline might be close to a circle or not.
     *                                        The polyline might be large enough to be a circle or not.
     *                                        The polyline is shifted: it's geometrical center is at (0,0) (sum of all coordinates is 0)

     * @param resultCoordinatesLength - [in] length of resultCoordinates array; should be 2 or more
     * @param actualResultCoordinates - [out] actual amount of result coordinates written
     */
    void processMatExt(const cv::Mat &frame, uint timestamp, int &state, int &index, int &message,
                       float *resultCoordinates, int resultCoordinatesLength,
                       int &actualResultCoordinates);

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
    void processMatExt(const cv::Mat &frame, uint timestamp, int &state, int &index, int &message,
                       float *point, float *shift, float *defect,
                       float *circleCoordinates, int circleCoordinatesLength,
                       int &actualCircleCoordinates);

    /**
     * @return version of Swype-code detector that helps to input swype-code
     */
    int GetSwypeHelperVersion() const {
        return ACTUAL_SWYPE_HELPER_VERSION;
    }

private:

    void DetectCircle(VectorExplained windowedShift, uint timestamp);

    SwypeCode _swypeCode;//we have swype code or we will wait swype code

    SwypeCodeDetectorDelta _detector;
};