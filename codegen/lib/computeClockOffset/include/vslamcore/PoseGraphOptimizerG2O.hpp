/* Copyright 2023 The MathWorks, Inc. */

/**
 * @file
 * Header for pose graph optimization
 */

#ifndef POSE_GRAPH_OPTIMIZER_G2O_HPP
#define POSE_GRAPH_OPTIMIZER_G2O_HPP

#include <algorithm>

#include "g2o/config.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/core/sparse_optimizer.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/sim3/sim3.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

namespace vision {
    namespace vslam {

        /**
         * @brief Optimize 3-D similarity pose graph
         *
         * @param[in] Nodes Absolute node poses to be optimized, specified as a vector of double vectors.
         *      Each double vector holds an 8-element SIM3 pose.
         * @param[in] Edges Relative edge constraints, specified as a vector of double vectors.
         *      Each double vector holds 10 doubles: a 2-element edge id and 8-element SIM3 pose.
         * @param[out] OptimNodes Optimized absolute node poses, specified as a vector of double vectors.
         *      Each double vector holds an 8-element SIM3 pose.
         * @param[in] functionTolerance functional tolerance, specified as a double value.
         * @param[in] maxIterations maximum number of iterations, specified as an integer value.
         */
        void PoseGraphOptimizerSimilarity(const std::vector<std::vector<double>>& nodes,
            const std::vector<std::vector<double>>& edges,
            const std::vector<std::vector<double>>& infoMats,
            std::vector<std::vector<double>>& solvedNodes,
            double functionTolerance = 1e-16,
            int maxIterations = 100);

         /**
         * @brief Optimize 3-D rigid pose graph
         *
         * @param[in] Nodes Absolute node poses to be optimized, specified as a vector of double vectors.
         *      Each double vector holds an 7-element SE3 pose.
         * @param[in] Edges Relative edge constraints, specified as a vector of double vectors.
         *      Each double vector holds 9 doubles: a 2-element edge id and 7-element SE3 pose.
         * @param[out] OptimNodes Optimized absolute node poses, specified as a vector of double vectors.
         *      Each double vector holds an 7-element SE3 pose.
         * @param[in] functionTolerance functional tolerance, specified as a double value.
         * @param[in] maxIterations maximum number of iterations, specified as an integer value.
         */
        void PoseGraphOptimizerRigid(const std::vector<std::vector<double>>& nodes,
            const std::vector<std::vector<double>>& edges,
            const std::vector<std::vector<double>>& infoMats,
            std::vector<std::vector<double>>& solvedNodes,
            double functionTolerance = 1e-16,
            int maxIterations = 100);

    } // end namesapce vslam
} // end namespace vision

#endif /* POSE_GRAPH_OPTIMIZER_G2O_HPP */