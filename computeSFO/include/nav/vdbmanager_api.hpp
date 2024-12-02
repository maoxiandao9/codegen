/* Copyright 2023 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for meshtsdf.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#ifndef VDBMANAGER_CODEGEN_API
#define VDBMANAGER_CODEGEN_API

#ifdef BUILDING_LIBMWVDBVOLUMECODEGEN
#include "vdbvolumecodegen/vdbvolumecodegen_util.hpp"
#else
#include "vdbvolumecodegen_util.hpp"
#endif

EXTERN_C VDBMANAGER_EXPORT_API void* vdbmanager_initialize(const double resolution,
                                                           const double truncDist,
                                                           const bool fillInterior,
                                                           const bool fastSweep);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_cleanup(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getNumMesh(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getTruncDist(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getResolution(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getFillInterior(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getSignedDistanceMode(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_getActiveBoundingBox(void* mgr, double* data);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_getID(void* mgr, double* idSet);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getNumActiveVoxel(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getNumActiveVoxelInMesh(void* mgr, double meshID);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_addMesh(void* mgr,
                                                         const double id,
                                                         const double* const pose,
                                                         const double nVert,
                                                         const double* const vertices,
                                                         const double nFace,
                                                         const double* const faces);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_updatePose(void* mgr,
                                                            const double id,
                                                            const double* const pose);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_removeMeshes(void* mgr,
                                                          const double* const id,
                                                          const double numID);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_getActiveVoxelFrom(void* mgr, double id, double* ctr, double* dist, double* size);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_getPoseFrom(void* mgr, double id, double* pose);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_distance(void* mgr,
                                                        const double* const pts,
                                                        const double numPts,
                                                        const double interpMethod,
                                                        double* const distData);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_gradient(void* mgr,
                                                        const double* const pts,
                                                        const double numPts,
                                                        const double interpMethod,
                                                        double* const gradData);

EXTERN_C VDBMANAGER_EXPORT_API double vdbmanager_getSerializeSize(void* mgr);

EXTERN_C VDBMANAGER_EXPORT_API void vdbmanager_serialize(void* mgr, char* const data);

EXTERN_C VDBMANAGER_EXPORT_API void* vdbmanager_deserialize(const char* const data,
                                                            const double bufferLength);

#endif // VDBMANAGER_CODEGEN_API
