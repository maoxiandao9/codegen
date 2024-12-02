////////////////////////////////////////////////////////////////////////////////
// Estimate the current camera extrinsics by tracking the previous key frame
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef TRACKLASTKEYFRAME_HPP
#define TRACKLASTKEYFRAME_HPP

#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"
#include "Configuration.hpp"

namespace vision {
    namespace vslam {
        class KeyFrameSet;
        class MapPointSet;

        /**
        * @brief Estimate the current camera extrinsics by tracking the previous key frame
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in] currFeatures ORB features of the current frame, specified as an N-by-32 cv::Mat.
        * @param[in] currPoints Key points corresponding to currFeatures, specified as an M-element vector of cv::KeyPoint.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d.
        * @param[out] currExtrinsics_R rotation matrix from world coordinate to camera coordinate, returned as a cv::Matx33d.
        * @param[out] currExtrinsics_t translation vector from world coordinate to camera coordinate, returned as a cv::Vec3d.
        * @param[out] trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        * @param[out] trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        * @param[in] config configuration parameters of the system.
        */

        void trackLastKeyFrame(MapPointSet& mpSet,
            KeyFrameSet& kfSet, 
            const cv::Mat& currFeatures,
            const std::vector<cv::KeyPoint>& currPoints,
            const cv::Matx33d& intrinsics,
            cv::Matx33d& currExtrinsics_R,
            cv::Vec3d& currExtrinsics_t,
            std::vector<int>& trackedMapPointIds,
            std::vector<int>& trackedFeatureIndices,
            const Configuration& config);
    }// namespace vslam
}// namespace vision
#endif //TRACKLASTKEYFRAME