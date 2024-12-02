////////////////////////////////////////////////////////////////////////////////
// Perform bundle adjustment over camera poses and 3-D world points
// 
// Copyright 2022-2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef BUNDLEADJUSTMENT_HPP
#define BUNDLEADJUSTMENT_HPP

#include <iostream>
#include <stdint.h>
#include <cstdlib>
#include <string>
#include <cmath> //pow, sqrt
#include <numeric>
#include <algorithm>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "Configuration.hpp"
#include "converter.hpp"

namespace vision {
    namespace vslam {
        class MapPointSet;
        class KeyFrameSet;

        /**
        * @brief Refine camera poses and 3-D world point locations using g2o. 
        *
        * @param[in] mpSet a MapPointSet object containing 3-D world points.
        * @param[in] kfSet a KeyFrameSet object containing camera poses.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d
        * @param[in] viewIds Id of camera poses to be refined in kfSet, specified as a vector integers.
        * @param[in] viewIds Id of cameras to be fixed during the optimization, specified as a vector integers.
        * @param[in] maxIter maximum number of iterations, specified as an integer.
        * @param[in] abortBA Flag used to terminate BA.
        */
        void bundleAdjustment(
            MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const cv::Matx33d& intrinsics,
            const std::vector<int>& viewIds,
            const std::vector<int>& fixedViewIds,
            const Configuration& config,
            bool * abortBA = nullptr);

#ifdef CERES_ON
        /**
        * @brief Refine camera poses and 3-D world point locations using Ceres. 
        *
        * @param[in] mpSet a MapPointSet object containing 3-D world points.
        * @param[in] kfSet a KeyFrameSet object containing camera poses.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d
        * @param[in] viewIds Id of camera poses to be refined in kfSet, specified as a vector integers.
        * @param[in] fixedViewIds Id of cameras to be fixed during the optimization, specified as a vector integers.
        * @param[in] config contains the configuration of the optimizer.
        * @param[in] maxIter maximum number of iterations, specified as an integer.
        * @param[in] abortBA Flag used to terminate BA.
        */
        void bundleAdjustmentCeres(
            MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const cv::Matx33d& intrinsics,
            const std::vector<int>& viewIds,
            const std::vector<int>& fixedViewIds,
            const Configuration& config,
            const int& maxIter = 10,
            bool * abortBA = nullptr);
#endif
    }// namespace vslam
}// namespace vision
#endif //BUNDLEADJUSTMENT_HPP
