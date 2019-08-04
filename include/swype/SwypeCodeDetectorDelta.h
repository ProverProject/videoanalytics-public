//
// Created by babay on 21.06.2018.
//

#ifndef SWYPE_SWYPECODEDETECTORDELTA_H
#define SWYPE_SWYPECODEDETECTORDELTA_H


#include <opencv2/core/mat.hpp>
#include "swype/ShiftDetector.h"
#include "swype/SwypeCodeDetector.h"

class SwypeCodeDetectorDelta : public SwypeCodeDetector {
public:

    SwypeCodeDetectorDelta() : SwypeCodeDetector() {};

    void NextFrame(VectorExplained shift);

    void AdvanceStep();

};


#endif //SWYPE_SWYPECODEDETECTORDELTA_H
