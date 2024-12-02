////////////////////////////////////////////////////////////////////////////////
// Perform map initialization using homography or fundamental matrix
//
// Copyright 2022-2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef INITIALIZEMAP_HPP
#define INITIALIZEMAP_HPP

#include "MapPointSet.hpp"
#include "KeyFrameSet.hpp"
#include "StereoReconstructor.hpp"
#include "correctLoop.hpp"
#include "matchFeatures.hpp"
#include "reconstructFromHomography.hpp"
#include "reconstructFromFundamentalMatrix.hpp"
#include "bundleAdjustment.hpp"
#include "geometry.hpp"
#include "NodeIDGenerator.hpp"

namespace vision {
    namespace vslam {

        class MapPointSet;
        class KeyFrameSet;
        class StereoReconstructor;

        /**
        * @brief Perform map initialization for monocular vSLAM using homography or fundamental matrix between previous view and current view
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in/out] database instance to detect loop closure and store inverted image index.
        * @param[in] currFeatures, ORB features of the current frame, specified as an M-by-32 cv::Mat.
        * @param[in] currPoints, Key points corresponding to currFeatures, specified as an M-element vector of cv::KeyPoint.
        * @param[in] intrinsics, intrinsic parameters of the camera, represented as a cv::Matx33d.
        * @param[in/out] currKeyFrameId, an integer representing the ID of the current key frame.
        * @param[out] trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        * @param[out] trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return boolean indicating if map initialization was successful.
        */
        bool initializeMapMono(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            const cv::Mat& currFeatures,
            const std::vector<cv::KeyPoint>& currPoints,
            const cv::Matx33d& intrinsics,
            int& currKeyFrameId,
            std::vector<int>& trackedMapPointIds,
            std::vector<int>& trackedFeatureIndices,
            const Configuration& config,
            NodeIDGenerator& generator);

        /**
        * @brief Perform map initialization for stereo vSLAM using disparity reconstruction
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in/out] database instance to detect loop closure and store inverted image index.
        * @param[in] reconstructor StereoReconstructor object used to construct 3-D world points.
        * @param[in] frameL, current left frame, specified as a cv::Mat.
        * @param[in] frameR, current right frame, specified as a cv::Mat.
        * @param[in] featuresL, ORB features of the current left frame, specified as an M-by-32 cv::Mat.
        * @param[in] featuresR, ORB features of the current right frame, specified as an M-by-32 cv::Mat.
        * @param[in] pointsL, Key points corresponding to featuresL, specified as an M-element vector of cv::KeyPoint.
        * @param[in] pointsR, Key points corresponding to featuresR, specified as an M-element vector of cv::KeyPoint.
        * @param[in] intrinsics, intrinsic parameters of the camera, represented as a cv::Matx33d.
        * @param[out] trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        * @param[out] trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return boolean indicating if map initialization was successful.
        */
        bool initializeMapStereo(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            const StereoReconstructor& reconstructor,
            const cv::Mat& frameL,
            const cv::Mat& frameR,
            const cv::Mat& featuresL,
            const cv::Mat& featuresR,
            const std::vector<cv::KeyPoint>& pointsL,
            const std::vector<cv::KeyPoint>& pointsR,
            const cv::Matx33d& intrinsics,
            std::vector<int>& trackedMapPointIds,
            std::vector<int>& trackedFeatureIndices,
            const Configuration& config,
            NodeIDGenerator& generator);

        /**
        * @brief Perform map initialization for vSLAM using RGB-D camera
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in/out] database instance to detect loop closure and store inverted image index.
        * @param[in] currFeatures, ORB features of the current color image, specified as an M-by-32 cv::Mat.
        * @param[in] currPoints, Key points corresponding to features, specified as an M-element vector of cv::KeyPoint.
        * @param[in] intrinsics, intrinsic parameters of the camera, represented as a cv::Matx33d.
        * @param[in] xyzPoints, 3-D world points computed from the depth map, represented as a vector of cv::Vec3d.
        * @param[in] validIndex, indices of features corresponding to xyzPoints, represented as a vector of integers.
        * @param[out] trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        * @param[out] trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return boolean indicating if map initialization was successful.
        */
        bool initializeMapRGBD(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            const cv::Mat& currFeatures,
            const std::vector<cv::KeyPoint>& currPoints,
            const cv::Matx33d& intrinsics,
            const std::vector<cv::Vec3d>& xyzPoints,
            const std::vector<int>& validIndex,
            std::vector<int>& trackedMapPointIds,
            std::vector<int>& trackedFeatureIndices,
            const Configuration& config,
            NodeIDGenerator& generator);
    }
}
#endif // INITIALIZEMAP_HPP
