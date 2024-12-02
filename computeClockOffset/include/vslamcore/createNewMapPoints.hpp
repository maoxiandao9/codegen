///////////////////////////////////////////////////////////////////////////
// Create new map points
// 
// Copyright 2022-23 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#ifndef CREATENEWMAPPOINTS_HPP
#define CREATENEWMAPPOINTS_HPP

#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"
#include "Configuration.hpp"
#include "matchFeatures.hpp"
#include "bundleAdjustment.hpp"
#include "computeReprojectionErrors.hpp"
#include "NodeIDGenerator.hpp"

namespace vision {
    namespace vslam {
        class KeyFrameSet;
        class MapPointSet;
        
        /**
        * @brief Get translation from camera center to 3-D world point
        *
        * @param[in] worldPoint, a 3-D world point represented as a 3-by-1 cv::Mat
        * @param[in] pose, the position of camera center in world coordinate, specifiec as a cv::Vec<3,T>.
        * @param[out] camToPoint, the XYZ translation from camera center to world point, specified as cv::Vec<3,T>.
        */
        template<typename T>
        void getCamToPoint(const cv::Mat& worldPoint, const cv::Vec<T,3>& pose, cv::Vec<T,3>& camToPoint) {
            camToPoint[0] = worldPoint.at<T>(0,0) - pose[0];
            camToPoint[1] = worldPoint.at<T>(1,0) - pose[1];
            camToPoint[2] = worldPoint.at<T>(2,0) - pose[2];
        }

        /**
        * @brief Create new map points using triangulation for monocular visual SLAM
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in/out] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in] currFeatures ORB features of the current frame, specified as an N-by-32 cv::Mat.
        * @param[in] currPoints Key points corresponding to currFeatures, specified as an M-element vector of cv::KeyPoint.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d.
        * @param[in] currExtrinsics_R rotation matrix from world coordinate to camera coordinate, specified as a cv::Matx33d.
        * @param[in] currExtrinsics_t translation vector from world coordinate to camera coordinate, specified as a cv::Vec3d.
        * @param[out] recentMapPointIds IDs of newly created map points, returned as a vector of integers.
        * @param[in/out] trackedFeatureIndices Indices of tracked feature points in the current key frame, specified as a vector of integers.
        * @param[out] refinedViewIds Id of camera poses to be refined in kfSet, specified as a vector integers.
        * @param[out] fixedViewIds Id of cameras to be fixed during the optimization, specified as a vector integers.
        * @param[in] currKeyFrameId current key frame Id, specified as an integer.
        * @param[in] config configuration parameters of the system.
        */

        void createNewMapPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const cv::Mat& currFeatures,
            const std::vector<cv::KeyPoint>& currPoints,
            const cv::Matx33d& intrinsics,
            const cv::Matx33d& currExtrinsics_R,
            const cv::Vec3d& currExtrinsics_t,
            std::vector<int>& recentMapPointIds,
            std::vector<int>& trackedFeatureIndices,
            std::vector<int>& refinedViewIds,
            std::vector<int>& fixedViewIds,
            const int currKeyFrameId,
            const Configuration& config,
            NodeIDGenerator& generator);

        /**
        * @brief Create new map points using triangulation for stereo and RGB-D visual SLAM
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in/out] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in] currFeatures ORB features of the current frame, specified as an N-by-32 cv::Mat.
        * @param[in] currPoints Key points corresponding to currFeatures, specified as an M-element vector of cv::KeyPoint.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d.
        * @param[in] currExtrinsics_R rotation matrix from world coordinate to camera coordinate, specified as a cv::Matx33d.
        * @param[in] currExtrinsics_t translation vector from world coordinate to camera coordinate, specified as a cv::Vec3d.
        * @param[in/out] trackedFeatureIndices Indices of tracked feature points in the current key frame, specified as a vector of integers.
        * @param[in/out] 2-D to 3-D correspondence between feature points and map points created using stereo methods for the current key frame, 
        *           specified as a map between feature indices and map point IDs.
        * @param[out] refinedViewIds Id of camera poses to be refined in kfSet, specified as a vector integers.
        * @param[out] fixedViewIds Id of cameras to be fixed during the optimization, specified as a vector integers.
        * @param[in] currKeyFrameId current key frame Id, specified as an integer.
        * @param[in] config configuration parameters of the system.
        */

        void createNewMapPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const cv::Mat& currFeatures,
            const std::vector<cv::KeyPoint>& currPoints,
            const cv::Matx33d& intrinsics,
            const cv::Matx33d& currExtrinsics_R,
            const cv::Vec3d& currExtrinsics_t,
            std::vector<int>& trackedFeatureIndices,
            std::unordered_map<int, int>& stereoCorrespondence,
            std::vector<int>& refinedViewIds,
            std::vector<int>& fixedViewIds,
            const int currKeyFrameId,
            const Configuration& config);
    }// namespace vslam
}// namespace vision
#endif //CREATENEWMAPPOINTS_HPP