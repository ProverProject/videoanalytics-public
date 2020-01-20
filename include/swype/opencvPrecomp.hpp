/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef __OPENCV_PRECOMP_H__
#define __OPENCV_PRECOMP_H__

#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"

#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/hal/hal.hpp"
#include "opencv2/imgproc/hal/hal.hpp"

#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <float.h>
#include <opencv2/core/types_c.h>

#ifdef HAVE_TEGRA_OPTIMIZATION
#include "opencv2/imgproc/imgproc_tegra.hpp"
#else
#endif

#undef   CV_CALC_MIN
#define  CV_CALC_MIN(a, b) if((a) > (b)) (a) = (b)

#undef   CV_CALC_MAX
#define  CV_CALC_MAX(a, b) if((a) < (b)) (a) = (b)

#ifdef HAVE_IPP
static inline IppiInterpolationType ippiGetInterpolation(int inter)
{
    inter &= cv::INTER_MAX;
    return inter == cv::INTER_NEAREST ? ippNearest :
        inter == cv::INTER_LINEAR ? ippLinear :
        inter == cv::INTER_CUBIC ? ippCubic :
        inter == cv::INTER_LANCZOS4 ? ippLanczos :
        inter == cv::INTER_AREA ? ippSuper :
        (IppiInterpolationType)-1;
}
#endif

#include "opencv2/core/sse_utils.hpp"

#endif /*__OPENCV_CV_INTERNAL_H_*/
