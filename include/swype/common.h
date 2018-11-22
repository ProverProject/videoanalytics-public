#ifndef COMMON_C_
#define COMMON_C_

#define APPNAME "ProverMVPDetector"

#ifdef __ANDROID_API__

#include <android/log.h>


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

//#define BUILD_COLOR_QUANTUM

#endif

extern int logLevel;