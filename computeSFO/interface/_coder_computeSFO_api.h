/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_computeSFO_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-11-28 19:50:32
 */

#ifndef _CODER_COMPUTESFO_API_H
#define _CODER_COMPUTESFO_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
    real_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
real_T computeSFO(emxArray_real_T *buffer);

void computeSFO_api(const mxArray *prhs, const mxArray **plhs);

void computeSFO_atexit(void);

void computeSFO_initialize(void);

void computeSFO_terminate(void);

void computeSFO_xil_shutdown(void);

void computeSFO_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_computeSFO_api.h
 *
 * [EOF]
 */
