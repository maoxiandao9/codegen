/* Copyright 2022-2023 The MathWorks, Inc. */
#ifndef VSLAM_CORE_API
#define VSLAM_CORE_API

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
void* MonoVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const int nrows,
    const int ncols,
    const double scaleFactor,
    const int numLevels,
    const int maxNumPoints,
    const int trackFeatureRangeLow,
    const int trackFeatureRangeHigh,
    const int skipMaxFrames,
    const int loopClosureThreshold,
    const bool verbose,
    const char* vocabFile,
    const int threadLevel);

EXTERN_C LIBMWVSLAMCORE_API 
void MonoVisualSLAM_addFrame(void* objPtr,
    const uint8_T* imgData,
    const int nRows,
    const int nCols,
    const bool isCM);

EXTERN_C LIBMWVSLAMCORE_API 
bool MonoVisualSLAM_hasNewKeyFrame(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
void MonoVisualSLAM_getWorldPoints(void* objPtr, double* xyzPoints);

EXTERN_C LIBMWVSLAMCORE_API 
void MonoVisualSLAM_getCameraPoses(void* objPtr, double* camPoses);

EXTERN_C LIBMWVSLAMCORE_API 
void MonoVisualSLAM_getKeyFrameIDs(void* objPtr, int* keyFrameIDs);

EXTERN_C LIBMWVSLAMCORE_API 
int MonoVisualSLAM_getNumWorldPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
int MonoVisualSLAM_getNumCameraPoses(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
bool MonoVisualSLAM_isInitialized(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
bool MonoVisualSLAM_isLoopRecentlyClosed(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
int MonoVisualSLAM_getNumTrackedPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
bool MonoVisualSLAM_isDone(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
void MonoVisualSLAM_reset(void* objPtr);
#endif // VSLAM_CORE_API