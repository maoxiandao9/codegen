/* Copyright 2022-2023 The MathWorks, Inc. */
#ifndef RGBDVSLAM_CORE_API
#define RGBDVSLAM_CORE_API

#ifndef LIBMWVSLAMCORE_API
#    define LIBMWVSLAMCORE_API
#endif

#ifndef EXTERN_C
#  ifdef __cplusplus
#    define EXTERN_C extern "C"
#  else
#    define EXTERN_C extern
#  endif
#endif

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

EXTERN_C LIBMWVSLAMCORE_API
void* RGBDVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const int nrows,
    const int ncols,
    const double scaleFactor,
    const int numLevels,
    const int maxNumPoints,
    const double depthScaleFactor,
    const double depthRangeLow,
    const double depthRangeHigh,
    const int trackFeatureRangeLow,
    const int trackFeatureRangeHigh,
    const int skipMaxFrames,
    const int loopClosureThreshold,
    const bool verbose,
    const char* vocabFile,
    const int threadLevel);

EXTERN_C LIBMWVSLAMCORE_API
void RGBDVisualSLAM_addFrame(void* objPtr,
    const uint8_T* imgColorData,
    const uint16_T* imgDepthData,
    const int nRows,
    const int nCols,
    const bool isCM);

EXTERN_C LIBMWVSLAMCORE_API
bool RGBDVisualSLAM_hasNewKeyFrame(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
void RGBDVisualSLAM_getWorldPoints(void* objPtr, double* xyzPoints);

EXTERN_C LIBMWVSLAMCORE_API
void RGBDVisualSLAM_getCameraPoses(void* objPtr, double* camPoses);

EXTERN_C LIBMWVSLAMCORE_API
void RGBDVisualSLAM_getKeyFrameIDs(void* objPtr, int* keyFrameIDs);

EXTERN_C LIBMWVSLAMCORE_API
int RGBDVisualSLAM_getNumWorldPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
int RGBDVisualSLAM_getNumCameraPoses(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
bool RGBDVisualSLAM_isInitialized(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
bool RGBDVisualSLAM_isLoopRecentlyClosed(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
int RGBDVisualSLAM_getNumTrackedPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
bool RGBDVisualSLAM_isDone(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
void RGBDVisualSLAM_reset(void* objPtr);


#endif // RGBDVSLAM_CORE_API