// Copyright 2023 The MathWorks, Inc.
#ifndef VDBMANAGER_IMPL
#define VDBMANAGER_IMPL

#include <map>
#include <unordered_set>
#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/math/Transform.h>
#include <openvdb/math/Vec3.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/math/Stencils.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/FastSweeping.h>
#include <openvdb/tools/VolumeToSpheres.h>
#include <openvdb/util/NullInterrupter.h>
#include <openvdb/openvdb.h>
#include <openvdb/io/Stream.h>

#ifdef BUILDING_LIBMWVDBVOLUMECODEGEN
#include "vdbvolumecodegen/NavMeshAdapter.hpp"
#else
#include "NavMeshAdapter.hpp"
#endif

using DistGridType = openvdb::FloatGrid;
using PolyIdxGridType = openvdb::Int32Grid;

namespace nav {
class VDBManager {
  public:
    VDBManager(const double resolution,
               const double truncDist,
               const bool fillInterior,
               const bool fastSweep) {
        m_resolution = static_cast<float>(resolution);
        m_truncDist = static_cast<float>(truncDist);
        m_fillInterior = fillInterior;
        m_fastSweep = fastSweep;
        m_numActiveVoxel = 0u;
        recomputeLimits();
    }

    template <class Mesh>
    DistGridType::Ptr discretizeMesh(const Mesh& mesh) const;

    double addMesh(const size_t id,
                   const double* const pose,
                   const size_t nVert,
                   const double* const vertices,
                   const size_t nFace,
                   const double* const faces);

    void removeMesh(const std::vector<size_t>& idSet);

    double applyPose(const size_t id, const double* const pose);

    void tallyActiveVoxel(void);

    void distance(const std::vector<openvdb::Vec3d>& pts,
                  double* dists,
                  const size_t interpMethod) const;

    void gradient(const std::vector<openvdb::Vec3d>& pts,
                  double* grads,
                  const size_t interpMethod) const;

    void recomputeLimits(void);

    void getActiveVoxelFrom(const size_t id, double* ctrs, double* vals, double* sizes) const;

    void getPoseFrom(size_t idSet, double* poses) const;

    void getID(double* idSet) const;

    std::string serialize(void);

    static VDBManager deserialize(std::string& vdbManagerString);

    float getRes(void) const {
        return m_resolution;
    };

    float getTruncDist(void) const {
        return m_truncDist;
    };

    bool getFillInterior(void) const {
        return m_fillInterior;
    };

    bool getFastSweep(void) const {
        return m_fastSweep;
    };

    size_t getNumMesh(void) const {
        return m_vdbSet.size();
    };

    size_t getNumActiveVoxel(void) const {
        return m_numActiveVoxel;
    };

    size_t getNumActiveVoxel(const size_t id) const {
        if (m_vdbSet.find(id) != m_vdbSet.end()) {
            return m_vdbSet.at(id)->activeVoxelCount();
        }
        return 0u;
    }

    void getMapWorldLimits(double* lims) const {
        lims[0] = m_mapWorldLimits.min().x();
        lims[2] = m_mapWorldLimits.min().y();
        lims[4] = m_mapWorldLimits.min().z();
        lims[1] = m_mapWorldLimits.max().x();
        lims[3] = m_mapWorldLimits.max().y();
        lims[5] = m_mapWorldLimits.max().z();
    };

  private:
    float m_resolution;
    float m_truncDist;
    bool m_fillInterior;
    bool m_fastSweep;
    size_t m_numActiveVoxel;
    openvdb::BBoxd m_mapWorldLimits;
    std::map<size_t, DistGridType::Ptr> m_vdbSet;
};
} // namespace nav

#endif /* VDBMANAGER_IMPL */
