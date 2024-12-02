/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_computeSFO_info.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-12-02 19:45:56
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
        "789cdd554d4fc24010dd1a3426462531f1e0d5ab100e108337402026208d2d78108d653ba4d5ee6eed97c0c9a3378dbfc29fe3dd3fa3d0"
        "42694d034993a2ce65"
        "f6e5edf6bdce64671177d6e410423bc88da72d376f7b38ede535148c30cf85f671c1ed681da502e7a6fc8b9731a3160c2c175089c0eca4"
        "cc884a256a89431d90",
        "0126d31c90274c5fd540540908f3e07c8c486d8e9a8131355e5714c0f7824d90a198be436d1ecceaf11cf1bfa925eb7118518f7488bfaa"
        "5e9f9e747983dd01b6"
        "cc2e6632d4817e67a2db1608b55696047c0d62fada5be06bca6349c3b62659c02b920982c67408f8b88de96323d287cbc8ccee69e0ebbd"
        "c6d43b8ad40bf251fd",
        "f8590eb7318beab1bba4bf70f6f76f4eb250f8e492d44307ca4da27a5eac4a2feebdda8fd04b8778c0b962a7a81e3b2dcad73a56c1bc90"
        "474ecdf7c12fd059e4"
        "0345e0a4beff5be7a61ed357f8fd0afb9af26daaf69941daf4d19074b4ba79f91e53af1ea917e4437de055cc040c14e6979966496c94ca"
        "199131adc7065d199c",
        "6ea04edfbd4aeade67cbc9ced1870ffa96a4de34fefb1cb52bb94b326a354a7d6a3784ba991fe58d61f5efcfd12fa18aa591", ""};
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
    emlrtSetField(xEntryPoints, 0, "ResolvedFilePath", emlrtMxCreateString("D:\\Projects\\codeGen\\computeSFO.m"));
    emlrtSetField(xEntryPoints, 0, "TimeStamp", emlrtMxCreateDoubleScalar(739588.81307870371));
    emlrtSetField(xEntryPoints, 0, "Constructor", emlrtMxCreateLogicalScalar(false));
    emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
    xResult = emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
    emlrtSetField(xResult, 0, "Version", emlrtMxCreateString("24.2.0.2773142 (R2024b) Update 2"));
    emlrtSetField(xResult, 0, "ResolvedFunctions", (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
    emlrtSetField(xResult, 0, "Checksum", emlrtMxCreateString("Y7PCtX7EBF5mukSVzQfLrH"));
    emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
    return xResult;
}

/*
 * File trailer for _coder_computeSFO_info.c
 *
 * [EOF]
 */
