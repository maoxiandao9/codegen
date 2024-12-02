///////////////////////////////////////////////////////////////////////////
// Add a new key frame to the data management object
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef ADDNEWKEYFRAME_HPP
#define ADDNEWKEYFRAME_HPP

#include <vector>

#include "Configuration.hpp"

#include "opencv2/core.hpp"

namespace vision {
    namespace vslam {
        class KeyFrameSet;
        class MapPointSet;

        /**
        * @brief Add a new key frame and update 3-D to 2-D correspondences
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in/out] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in] currFeatures ORB features of the current frame, specified as an N-by-32 cv::Mat.
        * @param[in] currPoints Key points corresponding to currFeatures, specified as an M-element vector of cv::KeyPoint.
        * @param[in] currExtrinsics_R rotation matrix from world coordinate to camera coordinate, specified as a cv::Matx33d
        * @param[in] currExtrinsics_t translation vector from world coordinate to camera coordinate, specified as a cv::Vec3d
        * @param[in] trackedMapPointIds IDs of tracked map points observed in local key frames, specified as a vector of integers.
        * @param[in] trackedFeatureIndices Indices of tracked feature points in the current frame, specified as a vector of integers.
        * @param[in] localKeyFrameIds Ids of local key frames that are within distance of 2 to the current frame, specified as a vector of integers.
        * @param[in] currKeyFrameId current key frame Id, specified as an integer.
        * @param[in] config configuration parameters of the SLAM system.
        */

        void addNewKeyFrame(
            MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const cv::Mat& currFeatures,
            const std::vector<cv::KeyPoint>& currPoints,
            const cv::Matx33d& currExtrinsics_R,
            const cv::Vec3d& currExtrinsics_t,
            const std::vector<int>& trackedMapPointIds,
            const std::vector<int>& trackedFeatureIndices,
            const std::vector<int>& localKeyFrameIds,
            const int currKeyFrameId,
            const Configuration& config);
    }// namespace vslam
}// namespace vision
#endif //ADDNEWKEYFRAME_HPP

