/* Copyright 2023 The MathWorks, Inc. */
#ifndef STEREOVSLAM_CORE_API
#define STEREOVSLAM_CORE_API

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
void* StereoVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const int nrows,
    const int ncols,
    const double baseline,
    const double scaleFactor,
    const int numLevels,
    const int maxNumPoints,
    const int disparityRangeLow,
    const int disparityRangeHigh,
    const int uniquenessThreshold,
    const int trackFeatureRangeLow,
    const int trackFeatureRangeHigh,
    const int skipMaxFrames,
    const int loopClosureThreshold,
    const bool verbose,
    const char* vocabFile,
    const int threadLevel);

EXTERN_C LIBMWVSLAMCORE_API
void StereoVisualSLAM_addFrame(void* objPtr,
    const uint8_T* I1,
    const uint8_T* I2,
    const int nRows,
    const int nCols,
    const bool isCM);

EXTERN_C LIBMWVSLAMCORE_API
bool StereoVisualSLAM_hasNewKeyFrame(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
void StereoVisualSLAM_getWorldPoints(void* objPtr, double* xyzPoints);

EXTERN_C LIBMWVSLAMCORE_API
void StereoVisualSLAM_getCameraPoses(void* objPtr, double* camPoses);

EXTERN_C LIBMWVSLAMCORE_API
void StereoVisualSLAM_getKeyFrameIDs(void* objPtr, int* keyFrameIDs);

EXTERN_C LIBMWVSLAMCORE_API
int StereoVisualSLAM_getNumWorldPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
int StereoVisualSLAM_getNumCameraPoses(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
bool StereoVisualSLAM_isInitialized(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
bool StereoVisualSLAM_isLoopRecentlyClosed(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
int StereoVisualSLAM_getNumTrackedPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
bool StereoVisualSLAM_isDone(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API
void StereoVisualSLAM_reset(void* objPtr);


#endif // STEREOVSLAM_CORE_API