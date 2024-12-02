///////////////////////////////////////////////////////////////////////////
// Convert a depth image to a point cloud
// 
// Copyright 2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#ifndef PCFROMDEPTH_HPP
#define PCFROMDEPTH_HPP

#include <vector>

#include "opencv2/core.hpp"
#include "Configuration.hpp"

namespace vision {
    namespace vslam {
        /**
        * @brief Convert a depth image using camera intrinsics, into a point cloud. 
        *
        * @param[in] depthImage depth image, specified as a cv::Mat.
        * @param[in] imagePoints image points of interest, specified as an M-element vector of cv::KeyPoint.
        * @param[in] intrinsics camera intrinsics parameters, specified as a 3-by-3 matrix.
        * @param[in] config configuration parameters of the SLAM system.
        * @param[in/out] xyzPoints the output point cloud, returned as an M-element 3-D world coordinates with 
        *       its origin centered at the camera.
        * @param[in/out] validIndex a vector containing sorted indices of features whose corresponding 3-D world points are
        *       within the desired depth range.
        */

        void pcfromdepth(
            cv::Mat& depthImage,
            const std::vector<cv::KeyPoint>& imagePoints,
            const cv::Matx33d& intrinsics,
            const Configuration& config,
            std::vector<cv::Vec3d>& xyzPoints,
            std::vector<int>& validIndex);
    }// namespace vslam
}// namespace vision
#endif //PCFROMDEPTH_HPP

