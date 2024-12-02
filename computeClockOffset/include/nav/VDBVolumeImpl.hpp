// Copyright 2023 The MathWorks, Inc.

/**
 * This file contains the wrapper for the vdbfusion library.
 */

#ifndef VDBVOLUME_IMPL
#define VDBVOLUME_IMPL

#include "matrix.h"
#include "mcos_factory/mi.hpp"
#include "sl_services_mi/slsv_mcos.hpp"
#include "vdbvolumebuiltin/vdbvolumebuiltin_util.hpp"
#include "vdbvolumebuiltin/vdbvolume_mcosutils.hpp"

// Forward declarations
namespace vdbfusion {
class VDBVolume;
}

namespace nav {
/**
 * @brief Wraps the VDBVolume class from vdbfusion and exposes some tools from openvdb
 */
class VDBVOLUMEBUILTIN_EXPORT_CLASS VDBVolumeImpl {
  public:
    /**
     * @brief Creates VDBVolumeImpl object
     * @param resolution Resolution of the octree leaf nodes (in meters)
     */
    VDBVolumeImpl(double resolution, double truncDist, bool fullTracing);

    /**
     * @brief Get resolution of VDBVolume
     * @return Resolution of octree leaf nodes (in cells/meter)
     */
    double getResolution(void) const;

    /**
     * @brief Get TruncationDistance of VDBVolume
     * @return TruncationDistance of VDBVolume (in meters)
     */
    double getTruncDist(void) const;

    /**
     * @brief Get NumActiveVoxel of VDBVolume
     * @return NumActiveVoxel of VDBVolume
     */
    double getNumActiveVoxel(void) const;

    /**
     * @brief Get FullTracing option of VDBVolume
     * @return Boolean indicating whether raycast traces full positive distance in space (true), or
     * only within +/- TruncationDistance of the target
     */
    double getFullTracing(void) const;

    /**
     * @brief Get ActiveBoundingBox
     * @return A 2x3 matrix corresponding to the min/max vertices of the bounding box containing all
     * active voxels
     */
    mxArrayProxy getActiveBoundingBox(void) const;

    void insertPointCloud(const mxArrayProxy& mOrigin, const mxArrayProxy& mPts);

    mxArrayProxy getActiveVoxels(void);

    mxArrayProxy createMesh(bool fillHoles, float minWeight);

    mxArrayProxy distance(const mxArrayProxy& pts, const size_t interpType);

    mxArrayProxy gradient(const mxArrayProxy& pts, const size_t interpType);

    mxArrayProxy serialize(void);

    void deserialize(const mxArrayProxy& serializedData);

    std::shared_ptr<vdbfusion::VDBVolume> getVDB(void) {
        return m_vdbPtr;
    }

  private:
    std::shared_ptr<vdbfusion::VDBVolume> m_vdbPtr;
};

} // namespace nav

#endif
