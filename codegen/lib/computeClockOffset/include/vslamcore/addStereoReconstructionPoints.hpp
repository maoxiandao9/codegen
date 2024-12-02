///////////////////////////////////////////////////////////////////////////////////
// Perform stereo reconstruction from disparity and update map with new 3-D points.
//
// Copyright 2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////////////

#ifndef ADDSTEREORECONSTRUCTIONPOINTS_HPP
#define ADDSTEREORECONSTRUCTIONPOINTS_HPP

#include "MapPointSet.hpp"
#include "KeyFrameSet.hpp"
#include "StereoReconstructor.hpp"
#include "matchFeatures.hpp"
#include "NodeIDGenerator.hpp"

namespace vision {
    namespace vslam {
        
        /**
        * @brief Identify untracked indices in matchedPairs
        *
        * @param[in] matchedPairs, indices of matched features, specified as a vector of pairs<T,T> sorted least to greatest.
        * @param[in] trackedFeatureIndices, Indices of tracked feature points, specified as a vector<T> sorted least to greatest.
        * @return vector<bool> with TRUE values indicating untracked indices in matchedPairs.
        */
        template<class T>
        std::vector<bool> findUntrackedFeatures(const std::vector<std::pair<T,T>>& matchedPairs, const std::vector<T>& trackedFeatureIndices) {
            const size_t numPairs = matchedPairs.size(), numTracked = trackedFeatureIndices.size();
            std::vector<bool> isUntrackedIdx(numPairs, true);

            for (size_t i = 0, j = 0; i < numPairs && j < numTracked;) {
                if (matchedPairs[i].first < trackedFeatureIndices[j])
                    ++i;
                else {
                    if (matchedPairs[i].first == trackedFeatureIndices[j])
                        isUntrackedIdx[i++] = false;
                    ++j;
                }
            }

            return isUntrackedIdx;
        }

        class MapPointSet;
        class KeyFrameSet;
        class StereoReconstructor;

        /**
        * @brief Perform stereo reconstruction from disparity and update map with new 3-D points
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in] reconstructor StereoReconstructor object used to construct 3-D world points.
        * @param[in] frameL, current left frame, specified as a cv::Mat.
        * @param[in] frameR, current right frame, specified as a cv::Mat.
        * @param[in] featuresL, ORB features of the current left frame, specified as an M-by-32 cv::Mat.
        * @param[in] featuresR, ORB features of the current right frame, specified as an M-by-32 cv::Mat.
        * @param[in] pointsL, Key points corresponding to featuresL, specified as an M-element vector of cv::KeyPoint.
        * @param[in] pointsR, Key points corresponding to featuresR, specified as an M-element vector of cv::KeyPoint.
        * @param[in/out] trackedFeatureIndices, Indices of tracked feature points in the current key frame, specified as a vector of integers.
        * @param[in] currKeyFrameId, the Id of the current key frame, represented as an integer.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return map linking 2-D image point indices in the current frame to 3-D world point IDs.
        */
        std::unordered_map<int, int> addStereoReconstructionPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const StereoReconstructor& reconstructor,
            const cv::Mat& frameL,
            const cv::Mat& frameR,
            const cv::Mat& featuresL,
            const cv::Mat& featuresR,
            const std::vector<cv::KeyPoint>& pointsL,
            const std::vector<cv::KeyPoint>& pointsR,
            std::vector<int>& trackedFeatureIndices,
            const int currKeyFrameId,
            const Configuration& config,
            NodeIDGenerator& generator);
    }
}
#endif // ADDSTEREORECONSTRUCTIONPOINTS_HPP
