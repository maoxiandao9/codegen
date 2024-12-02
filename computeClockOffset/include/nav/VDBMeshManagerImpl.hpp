// Copyright 2023 The MathWorks, Inc.

/**
 * This file contains the wrapper for the vdbfusion library.
 */

#ifndef VDBMESHMANAGER_IMPL
#define VDBMESHMANAGER_IMPL

#include "vdbvolumebuiltin/vdbvolume_mcosutils.hpp"
#include "vdbvolumebuiltin/vdbvolumebuiltin_util.hpp"

namespace nav {
// Forward declarations
class VDBManager;

/**
 * @brief Wraps the VDBVolume class from vdbfusion and exposes some tools from openvdb
 */
class VDBVOLUMEBUILTIN_EXPORT_CLASS VDBMeshManagerImpl {
  public:
    VDBMeshManagerImpl(const double resolution,
                       const double truncDist,
                       const bool fillInterior,
                       const bool fastSweep);

    // Property Getters
    double getNumMesh(void) const;

    double getTruncDist(void) const;

    double getResolution(void) const;

    double getNumActiveVoxel(void) const;

    double getFillInterior(void) const;

    double getSignedDistanceMode(void) const;

    mxArrayProxy getActiveBoundingBox(void) const;

    mxArrayProxy getID(void) const;

    // Exposed Methods
    mxArrayProxy addMeshes(const mxArrayProxy& meshStruct);

    mxArrayProxy updatePoses(const mxArrayProxy& poseStruct);

    void removeMeshes(const mxArrayProxy& idSet);

    mxArrayProxy getActiveVoxels(void) const;

    mxArrayProxy getPoses(void) const;

    mxArrayProxy distance(const mxArrayProxy& pts, const size_t interpMethod);

    mxArrayProxy gradient(const mxArrayProxy& pts, const size_t interpMethod);

    mxArrayProxy serialize(void);

    void deserialize(const mxArrayProxy& serializedMgrData);

    std::shared_ptr<void> getManager(void) {
        return m_VDBManager;
    }

  private:
    std::shared_ptr<void> m_VDBManager;

    double addMesh(const double id,
            const mxArrayProxy& pose,
            const mxArrayProxy& vertices,
            const mxArrayProxy& faces);
};

} // namespace nav

#endif
