/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_computeSFO_info.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-11-28 19:50:32
 */

/* Include Files */
#include "_coder_computeSFO_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
    const mxArray *nameCaptureInfo;
    const char_T *data[5] = {
        "789ce555cb4ec240149d1a3426062531f107dc8a6581224b444b0822d51636c6c4d25e6c6367a6e94b74e5d29dfe823b7fc49ff12b044b"
        "e9c3342569525f7773"
        "e7e4ccf49cb937738b9876974108ad232f1ed6bc5c9ce1d22c2fa168c47926b68f896e47cba81039e7f34fb32c5362c3d8f6009130cc4f"
        "2a146b4422b6786700",
        "32c1a2ba0bca2733d27410350c42189c4e11e642d41c4ca9e9baa9827c23381899aa1538d4c3605e8fc784fb1616acc776423d4a31fee2"
        "f89255290656c20e76"
        "58992ad00232c9d8706c10b8de2e8ef81a67f4b599e2cbe76549971d5db28157250b049d1a10f17195d1c74aa20f8f51a833d421d07bce"
        "a8b793a817e593faf1",
        "b51c5e63d2eab1b1a0bf780ef6af7af76c57aff3d4f3e3bfe8657d575b097aa5180f72a53ea86b35b747786e60ef59e7cabdcb053ef814"
        "9d341f2801e7f5fd9f"
        "3a378d8cbee2ffafb82f9fef136d444ddc27b7a664a0ef9b97af19f55a897a513ed6075e93a92003012bb42c771be249e3b02c52aa0fe9"
        "9855c06523759af42a",
        "af77ffe21ce53a47dfde3bc53cf5fcf8eb73745fa876a5e6b05d33a1d36ad807cd8aab9c1dfffe39fa01b004a1e1", ""};
    nameCaptureInfo = NULL;
    emlrtNameCaptureMxArrayR2016a(&data[0], 2968U, &nameCaptureInfo);
    return nameCaptureInfo;
}

/*
 * Arguments    : void
 * Return Type  : mxArray *
 */
mxArray *emlrtMexFcnProperties(void)
{
    mxArray *xEntryPoints;
    mxArray *xInputs;
    mxArray *xResult;
    const char_T *propFieldName[9] = {
        "Version", "ResolvedFunctions",      "Checksum", "EntryPoints", "CoverageInfo", "IsPolymorphic", "PropertyList",
        "UUID",    "ClassEntryPointIsHandle"};
    const char_T *epFieldName[8] = {"QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
                                    "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
    xEntryPoints = emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
    xInputs = emlrtCreateLogicalMatrix(1, 1);
    emlrtSetField(xEntryPoints, 0, "QualifiedName", emlrtMxCreateString("computeSFO"));
    emlrtSetField(xEntryPoints, 0, "NumberOfInputs", emlrtMxCreateDoubleScalar(1.0));
    emlrtSetField(xEntryPoints, 0, "NumberOfOutputs", emlrtMxCreateDoubleScalar(1.0));
    emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
    emlrtSetField(xEntryPoints, 0, "ResolvedFilePath", emlrtMxCreateString("/home/amumu/codeGen/computeSFO.m"));
    emlrtSetField(xEntryPoints, 0, "TimeStamp", emlrtMxCreateDoubleScalar(739584.82674768521));
    emlrtSetField(xEntryPoints, 0, "Constructor", emlrtMxCreateLogicalScalar(false));
    emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
    xResult = emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
    emlrtSetField(xResult, 0, "Version", emlrtMxCreateString("24.2.0.2773142 (R2024b) Update 2"));
    emlrtSetField(xResult, 0, "ResolvedFunctions", (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
    emlrtSetField(xResult, 0, "Checksum", emlrtMxCreateString("KTrsKLJV8Br7L3P5FEw3CG"));
    emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
    return xResult;
}

/*
 * File trailer for _coder_computeSFO_info.c
 *
 * [EOF]
 */
