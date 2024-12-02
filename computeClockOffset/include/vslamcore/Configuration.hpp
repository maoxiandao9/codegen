#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <utility>

namespace vision {
    namespace vslam {
        struct Configuration {
            // Camera setup
            bool isStereo = false;

            // Image size
            int nRows = 480;
            int nCols = 640;

            // Map initialization
            int minNumWorldPoints = 100;
            double maxReprojErrorMapInit = 2.4474; //std::sqrt(5.99)
            double minCosParallaxInit = 0.9998; // cosd(1)
            double maxRatioHomography = 0.75;

            // Tracking last key frame
            float maxReprojError = 4.f;
            int minNumPnPPoints = 10;

            // Tracking local map
            int minNumPointWeakFrame = 100;
            int minNumMatches = 20;
            int numSkippedFrames = 20;

            // Add new key frames
            int minNumMatchesNewConnection = 5;

            // Features
            int numFeatures = 1000;
            float scaleFactor = 1.2f;
            int numLevels = 8;
            int matchThreshold = 100;
            float maxRatio = 0.9f;
            float maxRatioInRadius = 0.99f;
            int minNumMatchesLoop = 50;

            bool verbose = false;

            // Creating new map points
            float maxRatioMapping = 0.9f;
            int minNumMatchesMapping = 5;
            int minNumMatchesBA = 10;
            int maxNumFixedViewsBA = 5;
            int maxNumIterationsBA = 10;
            double maxCeresRE = 5.99;

            double minSinParallax = 0.0175; // sind(1)
            double maxCosParallax = 0.9986; // cosd(3)

            // Pose graph optimization
            int minNumMatchesPGO = 20;
            int optimizationInterval = 20;

            // Stereo reconstruction
            int minDisparity = 0;
            int maxDisparity = 48; // disparityRange = [minDisparity, maxDisparity]
            float maxDepthFactor = 200.f;
            int uniquenessThreshold = 15;

            // Depth image
            double depthScaleFactor = 5000;
            std::pair<float, float> depthRange = std::make_pair<float, float>(0.1f, 5.f);
        };
    }// namespace vslam
}// namespace vision
#endif //CONFIGURATION_HPP
