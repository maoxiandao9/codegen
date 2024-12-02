///////////////////////////////////////////////////////////////////////////
// Find valid matched feature pairs for stereo images
// 
// Copyright 2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef STEREORECONSTRUCTOR_HPP
#define STEREORECONSTRUCTOR_HPP

#include "Configuration.hpp"

#include <vector>
#include "opencv2/opencv.hpp"

namespace vision {
    namespace vslam {
        class StereoReconstructor {

        public:
            StereoReconstructor() = delete;

            /**
            * @brief Constructor 
            *
            * @param[in] intrinsics Camera intrinsics matrix specified as a cv::Matx33d.
            * @param[in] baseline Distance between stereo cameras specified as a double.
            * @param[in] config Configuration struct
            */
            StereoReconstructor(const cv::Matx33d& intrinsics, const double baseline, const Configuration& config);

            /**
            * @brief Destructor
            */
            ~StereoReconstructor();

            /**
            * @brief Get valid feature pairs and 3-D world points from stereo images
            *
            * @param[in] frameL Left stereo image, specified as a cv::Mat. The image is gray and assumed undistorted.
            * @param[in] frameR Right stereo image, specified as a cv::Mat. The image is gray and assumed undistorted.
            * @param[in] pointsL Key points of the extracted ORB features for frameL, specified as a vector of cv::KeyPoint.
            * @param[in] pointsR Key points of the extracted ORB features for frameR, specified as a vector of cv::KeyPoint.
            * @param[out] matchedPairs Indices of matched features, returned as a vector of pairs.
            * @param[out] xyzPoints 3-D world points in the left camera coordinate constructed from stereo images, specified as vector of cv::Vec3d.
            * @param[in] config Configuration struct
            */
            void reconstruct(
                const cv::Mat& frameL,
                const cv::Mat& frameR,
                const std::vector<cv::KeyPoint>& pointsL,
                const std::vector<cv::KeyPoint>& pointsR,
                std::vector<std::pair<int, int>>& matchedPairs,
                std::vector<cv::Vec3d>& xyzPoints,
                const Configuration& config) const;

        private:
            cv::Ptr<cv::StereoSGBM> sgbm;
            int minDisparity;
            int maxDisparity;
            float maxDepthFactor;
            cv::Matx44d reprojectionMatrix;
            
            const int blockSize = 15;
            const int P1 = 8*blockSize*blockSize;
            const int P2 = 32*blockSize*blockSize;
            const int disp12MaxDiff = -1; // disable maximum allowed difference in left-right disparity check
            const int preFilterCap = 31; // floor(ContrastThreshold*63)
        };
    }// namespace vslam
}// namespace vision
#endif //STEREORECONSTRUCTOR_HPP