/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeSFO_emxutil.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-11-28 19:50:32
 */

/* Include Files */
#include "computeSFO_emxutil.h"
#include "computeSFO_types.h"
#include "rt_nonfinite.h"
#include <stdlib.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_cint32_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_cint32_T(emxArray_cint32_T *emxArray, int oldNumel)
{
    int i;
    int newNumel;
    void *newData;
    if (oldNumel < 0) {
        oldNumel = 0;
    }
    newNumel = 1;
    for (i = 0; i < emxArray->numDimensions; i++) {
        newNumel *= emxArray->size[i];
    }
    if (newNumel > emxArray->allocatedSize) {
        i = emxArray->allocatedSize;
        if (i < 16) {
            i = 16;
        }
        while (i < newNumel) {
            if (i > 1073741823) {
                i = MAX_int32_T;
            } else {
                i *= 2;
            }
        }
        newData = malloc((unsigned int)i * sizeof(cint32_T));
        if (emxArray->data != NULL) {
            memcpy(newData, emxArray->data, sizeof(cint32_T) * (unsigned int)oldNumel);
            if (emxArray->canFreeData) {
                free(emxArray->data);
            }
        }
        emxArray->data = (cint32_T *)newData;
        emxArray->allocatedSize = i;
        emxArray->canFreeData = true;
    }
}

/*
 * Arguments    : emxArray_int16_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_int16_T(emxArray_int16_T *emxArray, int oldNumel)
{
    int i;
    int newNumel;
    void *newData;
    if (oldNumel < 0) {
        oldNumel = 0;
    }
    newNumel = 1;
    for (i = 0; i < emxArray->numDimensions; i++) {
        newNumel *= emxArray->size[i];
    }
    if (newNumel > emxArray->allocatedSize) {
        i = emxArray->allocatedSize;
        if (i < 16) {
            i = 16;
        }
        while (i < newNumel) {
            if (i > 1073741823) {
                i = MAX_int32_T;
            } else {
                i *= 2;
            }
        }
        newData = malloc((unsigned int)i * sizeof(short));
        if (emxArray->data != NULL) {
            memcpy(newData, emxArray->data, sizeof(short) * (unsigned int)oldNumel);
            if (emxArray->canFreeData) {
                free(emxArray->data);
            }
        }
        emxArray->data = (short *)newData;
        emxArray->allocatedSize = i;
        emxArray->canFreeData = true;
    }
}

/*
 * Arguments    : emxArray_real_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
    int i;
    int newNumel;
    void *newData;
    if (oldNumel < 0) {
        oldNumel = 0;
    }
    newNumel = 1;
    for (i = 0; i < emxArray->numDimensions; i++) {
        newNumel *= emxArray->size[i];
    }
    if (newNumel > emxArray->allocatedSize) {
        i = emxArray->allocatedSize;
        if (i < 16) {
            i = 16;
        }
        while (i < newNumel) {
            if (i > 1073741823) {
                i = MAX_int32_T;
            } else {
                i *= 2;
            }
        }
        newData = malloc((unsigned int)i * sizeof(double));
        if (emxArray->data != NULL) {
            memcpy(newData, emxArray->data, sizeof(double) * (unsigned int)oldNumel);
            if (emxArray->canFreeData) {
                free(emxArray->data);
            }
        }
        emxArray->data = (double *)newData;
        emxArray->allocatedSize = i;
        emxArray->canFreeData = true;
    }
}

/*
 * Arguments    : emxArray_uint16_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_uint16_T(emxArray_uint16_T *emxArray, int oldNumel)
{
    int i;
    int newNumel;
    void *newData;
    if (oldNumel < 0) {
        oldNumel = 0;
    }
    newNumel = 1;
    for (i = 0; i < emxArray->numDimensions; i++) {
        newNumel *= emxArray->size[i];
    }
    if (newNumel > emxArray->allocatedSize) {
        i = emxArray->allocatedSize;
        if (i < 16) {
            i = 16;
        }
        while (i < newNumel) {
            if (i > 1073741823) {
                i = MAX_int32_T;
            } else {
                i *= 2;
            }
        }
        newData = malloc((unsigned int)i * sizeof(unsigned short));
        if (emxArray->data != NULL) {
            memcpy(newData, emxArray->data, sizeof(unsigned short) * (unsigned int)oldNumel);
            if (emxArray->canFreeData) {
                free(emxArray->data);
            }
        }
        emxArray->data = (unsigned short *)newData;
        emxArray->allocatedSize = i;
        emxArray->canFreeData = true;
    }
}

/*
 * Arguments    : emxArray_cint32_T **pEmxArray
 * Return Type  : void
 */
void emxFree_cint32_T(emxArray_cint32_T **pEmxArray)
{
    if (*pEmxArray != (emxArray_cint32_T *)NULL) {
        if (((*pEmxArray)->data != (cint32_T *)NULL) && (*pEmxArray)->canFreeData) {
            free((*pEmxArray)->data);
        }
        free((*pEmxArray)->size);
        free(*pEmxArray);
        *pEmxArray = (emxArray_cint32_T *)NULL;
    }
}

/*
 * Arguments    : emxArray_int16_T **pEmxArray
 * Return Type  : void
 */
void emxFree_int16_T(emxArray_int16_T **pEmxArray)
{
    if (*pEmxArray != (emxArray_int16_T *)NULL) {
        if (((*pEmxArray)->data != (short *)NULL) && (*pEmxArray)->canFreeData) {
            free((*pEmxArray)->data);
        }
        free((*pEmxArray)->size);
        free(*pEmxArray);
        *pEmxArray = (emxArray_int16_T *)NULL;
    }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
void emxFree_real_T(emxArray_real_T **pEmxArray)
{
    if (*pEmxArray != (emxArray_real_T *)NULL) {
        if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
            free((*pEmxArray)->data);
        }
        free((*pEmxArray)->size);
        free(*pEmxArray);
        *pEmxArray = (emxArray_real_T *)NULL;
    }
}

/*
 * Arguments    : emxArray_uint16_T **pEmxArray
 * Return Type  : void
 */
void emxFree_uint16_T(emxArray_uint16_T **pEmxArray)
{
    if (*pEmxArray != (emxArray_uint16_T *)NULL) {
        if (((*pEmxArray)->data != (unsigned short *)NULL) && (*pEmxArray)->canFreeData) {
            free((*pEmxArray)->data);
        }
        free((*pEmxArray)->size);
        free(*pEmxArray);
        *pEmxArray = (emxArray_uint16_T *)NULL;
    }
}

/*
 * Arguments    : emxArray_cint32_T **pEmxArray
 * Return Type  : void
 */
void emxInit_cint32_T(emxArray_cint32_T **pEmxArray)
{
    emxArray_cint32_T *emxArray;
    int i;
    *pEmxArray = (emxArray_cint32_T *)malloc(sizeof(emxArray_cint32_T));
    emxArray = *pEmxArray;
    emxArray->data = (cint32_T *)NULL;
    emxArray->numDimensions = 2;
    emxArray->size = (int *)malloc(sizeof(int) * 2U);
    emxArray->allocatedSize = 0;
    emxArray->canFreeData = true;
    for (i = 0; i < 2; i++) {
        emxArray->size[i] = 0;
    }
}

/*
 * Arguments    : emxArray_int16_T **pEmxArray
 * Return Type  : void
 */
void emxInit_int16_T(emxArray_int16_T **pEmxArray)
{
    emxArray_int16_T *emxArray;
    *pEmxArray = (emxArray_int16_T *)malloc(sizeof(emxArray_int16_T));
    emxArray = *pEmxArray;
    emxArray->data = (short *)NULL;
    emxArray->numDimensions = 1;
    emxArray->size = (int *)malloc(sizeof(int));
    emxArray->allocatedSize = 0;
    emxArray->canFreeData = true;
    emxArray->size[0] = 0;
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
    emxArray_real_T *emxArray;
    int i;
    *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
    emxArray = *pEmxArray;
    emxArray->data = (double *)NULL;
    emxArray->numDimensions = numDimensions;
    emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
    emxArray->allocatedSize = 0;
    emxArray->canFreeData = true;
    for (i = 0; i < numDimensions; i++) {
        emxArray->size[i] = 0;
    }
}

/*
 * Arguments    : emxArray_uint16_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_uint16_T(emxArray_uint16_T **pEmxArray, int numDimensions)
{
    emxArray_uint16_T *emxArray;
    int i;
    *pEmxArray = (emxArray_uint16_T *)malloc(sizeof(emxArray_uint16_T));
    emxArray = *pEmxArray;
    emxArray->data = (unsigned short *)NULL;
    emxArray->numDimensions = numDimensions;
    emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
    emxArray->allocatedSize = 0;
    emxArray->canFreeData = true;
    for (i = 0; i < numDimensions; i++) {
        emxArray->size[i] = 0;
    }
}

/*
 * File trailer for computeSFO_emxutil.c
 *
 * [EOF]
 */
