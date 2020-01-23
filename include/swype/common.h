#ifndef COMMON_C_
#define COMMON_C_

#define APPNAME "ProverDetector"

#ifdef __ANDROID_API__

#include <android/log.h>
#include <linux/time.h>

#define LOGI_NATIVE(...) ((void)__android_log_print(ANDROID_LOG_INFO, APPNAME, __VA_ARGS__))
#define LOGE_NATIVE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, APPNAME, __VA_ARGS__))

#else

#include <stdio.h>

#define LOGI_NATIVE(fmt, ...) ((void)fprintf(stderr, fmt "\n", ##__VA_ARGS__))
#define LOGE_NATIVE(fmt, ...) ((void)fprintf(stderr, fmt "\n", ##__VA_ARGS__))
//#define LOGI_NATIVE(fmt, ...)
//#define LOGE_NATIVE(fmt, ...)

#endif

#define LOG_VECTORS 1
#define LOG_CIRCLE_DETECTION 2
#define LOG_GENERAL_DETECTION 4
#define LOG_DEBUG_COMPARE_VECTORS 8
#define LOG_ONMIDIR_DETECTOR 16
#define LOG_MASS_DETECTOR 32
#define LOG_MASS_DETECTOR_RESULTS 96


#define JNI_COMMIT_AND_RELEASE 0

#define ACTUAL_SWYPE_HELPER_VERSION 2
#define ACTUAL_SWYPE_DETECTOR_VERSION 2

double deltaTimeMks(const struct timespec &ts2, const struct timespec &ts1);

enum Message {
    /**
     * no message
     */
            None = 0,
    /**
     * Low luminance or low contrast or not enough details or too many periodic details in frame.
     */
            BadPicture = 1,
    /**
     * Swype code failed 'cause we've get out of detection bounds
     */
            SwypeFailOutOfBounds = 2,
    /**
     * Swype code failed 'cause of timeout
     */
            SwypeFailTimeout = 3,

    /**
     * Swype code failed 'cause of timeout
     */
            SwypeStepFinished = 5,
    /**
     * we've got a closed curve, it's round enough, but too small
     */
            CircleTooSmall = 4,
    /**
     * we've got closed curve, it's not round enough
     */
            ClosedCurveNotRoundEnough = 8,
    /**
     * we've got closed curve, it too small and not round enough
     */
            ClosedCurveSmallAndNotRoundEnough = 12,
};

//#define BUILD_COLOR_QUANTUM

#endif

extern unsigned int logLevel;