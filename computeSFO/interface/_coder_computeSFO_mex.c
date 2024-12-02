/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_computeSFO_mex.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-11-28 19:50:32
 */

/* Include Files */
#include "_coder_computeSFO_mex.h"
#include "_coder_computeSFO_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray *prhs[])
{
    mexAtExit(&computeSFO_atexit);
    computeSFO_initialize();
    unsafe_computeSFO_mexFunction(nlhs, plhs, nrhs, prhs);
    computeSFO_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
    emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1, NULL, "UTF-8", true);
    return emlrtRootTLSGlobal;
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[1]
 * Return Type  : void
 */
void unsafe_computeSFO_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[1])
{
    emlrtStack st = {
        NULL, /* site */
        NULL, /* tls */
        NULL  /* prev */
    };
    const mxArray *outputs;
    st.tls = emlrtRootTLSGlobal;
    /* Check for proper number of arguments. */
    if (nrhs != 1) {
        emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4, 10, "computeSFO");
    }
    if (nlhs > 1) {
        emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 10, "computeSFO");
    }
    /* Call the function. */
    computeSFO_api(prhs[0], &outputs);
    /* Copy over outputs to the caller. */
    emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * File trailer for _coder_computeSFO_mex.c
 *
 * [EOF]
 */
