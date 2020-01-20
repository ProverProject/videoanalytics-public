
#include "swype/common.h"
#include <opencv2/core.hpp>

unsigned int logLevel = 0;

double deltaTimeMks(const struct timespec &ts2, const struct timespec &ts1) {
    return (ts2.tv_nsec - ts1.tv_nsec) / 1000.0 + (ts2.tv_sec - ts1.tv_sec) * 1000000;
}
