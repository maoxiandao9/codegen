/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_computeSFO_api.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-11-28 19:50:32
 */

/* Include Files */
#include "_coder_computeSFO_api.h"
#include "_coder_computeSFO_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131659U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "computeSFO",                                         /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr, const char_T *identifier,
                             emxArray_real_T *y);

static const mxArray *emlrt_marshallOut(const real_T u);

static void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel);

static void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y)
{
    c_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
    emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                emxArray_real_T *ret
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret)
{
    static const int32_T dims = -1;
    int32_T i;
    int32_T i1;
    boolean_T b = true;
    emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U, (const void *)&dims, &b, &i);
    ret->allocatedSize = i;
    i1 = ret->size[0];
    ret->size[0] = i;
    emxEnsureCapacity_real_T(ret, i1);
    ret->data = (real_T *)emlrtMxGetData(src);
    ret->canFreeData = false;
    emlrtDestroyArray(&src);
}

/*
 * Arguments    : const void *r
 * Return Type  : void
 */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
    emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr, const char_T *identifier, emxArray_real_T *y)
{
    emlrtMsgIdentifier thisId;
    thisId.fIdentifier = (const char_T *)identifier;
    thisId.fParent = NULL;
    thisId.bParentIsCell = false;
    b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
    emlrtDestroyArray(&nullptr);
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u)
{
    const mxArray *m;
    const mxArray *y;
    y = NULL;
    m = emlrtCreateDoubleScalar(u);
    emlrtAssign(&y, m);
    return y;
}

/*
 * Arguments    : emxArray_real_T *emxArray
 *                int32_T oldNumel
 * Return Type  : void
 */
static void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel)
{
    int32_T i;
    int32_T newNumel;
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
        newData = emlrtMallocMex((uint32_T)i * sizeof(real_T));
        if (emxArray->data != NULL) {
            memcpy(newData, emxArray->data, sizeof(real_T) * (uint32_T)oldNumel);
            if (emxArray->canFreeData) {
                emlrtFreeMex(emxArray->data);
            }
        }
        emxArray->data = (real_T *)newData;
        emxArray->allocatedSize = i;
        emxArray->canFreeData = true;
    }
}

/*
 * Arguments    : const emlrtStack *sp
 *                emxArray_real_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray)
{
    if (*pEmxArray != (emxArray_real_T *)NULL) {
        if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
            emlrtFreeMex((*pEmxArray)->data);
        }
        emlrtFreeMex((*pEmxArray)->size);
        emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
        emlrtFreeEmxArray(*pEmxArray);
        *pEmxArray = (emxArray_real_T *)NULL;
    }
}

/*
 * Arguments    : const emlrtStack *sp
 *                emxArray_real_T **pEmxArray
 * Return Type  : void
 */
static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray)
{
    emxArray_real_T *emxArray;
    *pEmxArray = (emxArray_real_T *)emlrtMallocEmxArray(sizeof(emxArray_real_T));
    emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray, (void *)&emxFree_real_T, NULL, NULL,
                                        NULL);
    emxArray = *pEmxArray;
    emxArray->data = (real_T *)NULL;
    emxArray->numDimensions = 1;
    emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T));
    emxArray->allocatedSize = 0;
    emxArray->canFreeData = true;
    emxArray->size[0] = 0;
}

/*
 * Arguments    : const mxArray *prhs
 *                const mxArray **plhs
 * Return Type  : void
 */
void computeSFO_api(const mxArray *prhs, const mxArray **plhs)
{
    emlrtStack st = {
        NULL, /* site */
        NULL, /* tls */
        NULL  /* prev */
    };
    emxArray_real_T *buffer;
    real_T clockOffset;
    st.tls = emlrtRootTLSGlobal;
    emlrtHeapReferenceStackEnterFcnR2012b(&st);
    /* Marshall function inputs */
    emxInit_real_T(&st, &buffer);
    buffer->canFreeData = false;
    emlrt_marshallIn(&st, emlrtAlias(prhs), "buffer", buffer);
    /* Invoke the target function */
    clockOffset = computeSFO(buffer);
    emxFree_real_T(&st, &buffer);
    /* Marshall function outputs */
    *plhs = emlrt_marshallOut(clockOffset);
    emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void computeSFO_atexit(void)
{
    emlrtStack st = {
        NULL, /* site */
        NULL, /* tls */
        NULL  /* prev */
    };
    mexFunctionCreateRootTLS();
    st.tls = emlrtRootTLSGlobal;
    emlrtPushHeapReferenceStackR2021a(&st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    computeSFO_xil_terminate();
    computeSFO_xil_shutdown();
    emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void computeSFO_initialize(void)
{
    emlrtStack st = {
        NULL, /* site */
        NULL, /* tls */
        NULL  /* prev */
    };
    mexFunctionCreateRootTLS();
    st.tls = emlrtRootTLSGlobal;
    emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void computeSFO_terminate(void)
{
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_computeSFO_api.c
 *
 * [EOF]
 */
