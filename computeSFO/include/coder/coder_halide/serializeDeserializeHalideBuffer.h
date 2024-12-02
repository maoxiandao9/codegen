// Copyright 2023 The MathWorks, Inc.

#ifndef MATLAB_HALIDE_CONVERTER
#define MATLAB_HALIDE_CONVERTER

#include <stdlib.h>

#ifndef HALIDE_CODEGEN
#include "rtwtypes.h"
#endif

#ifdef HALIDE_CODEGEN
#include "HalideRuntime.h"
#endif

#ifdef _MSC_VER
#pragma warning (disable: 4200)
#pragma warning (disable: 4100)
#endif

halide_buffer_t matlabArrayToHalideBuffer_size_numDim(const void* wrapee, const int* size, const int dim,
const uint8_t halide_type_t_code, const uint8_t halide_type_t_bits, const uint16_t halide_type_t_lanes);

halide_buffer_t matlabArrayToHalideBuffer_min_size_stride_numDim(const void* wrapee, const int* min, const int* size, const int* stride, const int dim,
const uint8_t halide_type_t_code, const uint8_t halide_type_t_bits, const uint16_t halide_type_t_lanes);

void deallocateHalideBuffer(halide_buffer_t* wrapee);

#endif