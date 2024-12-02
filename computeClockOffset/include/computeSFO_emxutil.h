/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeSFO_emxutil.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-12-02 19:45:56
 */

#ifndef COMPUTESFO_EMXUTIL_H
#define COMPUTESFO_EMXUTIL_H

/* Include Files */
#include "computeSFO_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void emxEnsureCapacity_cint32_T(emxArray_cint32_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_int16_T(emxArray_int16_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_uint16_T(emxArray_uint16_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_uint32_T(emxArray_uint32_T *emxArray, int oldNumel);

extern void emxFree_cint32_T(emxArray_cint32_T **pEmxArray);

extern void emxFree_int16_T(emxArray_int16_T **pEmxArray);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxFree_uint16_T(emxArray_uint16_T **pEmxArray);

extern void emxFree_uint32_T(emxArray_uint32_T **pEmxArray);

extern void emxInit_cint32_T(emxArray_cint32_T **pEmxArray);

extern void emxInit_int16_T(emxArray_int16_T **pEmxArray);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

extern void emxInit_uint16_T(emxArray_uint16_T **pEmxArray, int numDimensions);

extern void emxInit_uint32_T(emxArray_uint32_T **pEmxArray);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeSFO_emxutil.h
 *
 * [EOF]
 */
