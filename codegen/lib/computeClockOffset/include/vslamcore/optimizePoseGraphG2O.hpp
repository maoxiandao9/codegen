////////////////////////////////////////////////////////////////////////////////
// Pose graph optimization functions
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef OPTIMIZEPOSEGRAPH_G2O_HPP
#define OPTIMIZEPOSEGRAPH_G2O_HPP

#include "PoseGraphOptimizerG2O.hpp"
#include "Connection.hpp"

namespace vision {
    namespace vslam {
        /**
        * @brief Optimize a similarity pose graph in the monocular visual SLAM workflow
        *
        * @param[in, out] viewIdsAndPoses view Ids and view poses. View poses are updated after optimization.
        * @param[in] odometryConnections odometry connections, specified as a vector of pointers to Connection,
        *            which contains the relative orientation R and relative translation t. The relative pose of 
        *            an odometry connection is represented as a 3-D rigid transform:
        *            | R, t |
        *            | 0, 1 |
        * @param[in] loopConnections loop closure connections, specified as a vector of pairs. Each pair contains
        *            a pointer to Connection, which contains the relative orientation R and relative translation t,  
        *            and the corresponding scale s. The relative pose of a loop closure connection is represented 
        *            as a 3-D similarity transform:
        *            | s*R, t |
        *            | 0,   1 |
        * @param[out] optimScales, optimized scales associated with view poses, returned as a vector of double values.        
        *             These values are used to update the map after similarity pose graph optimization in the monocular 
        *             visual SLAM workflow
        * @param[in] minNumMatches, minimum number of matched features points between two views for them to be 
        *            counted as connected views, specified as an integer.
        * @param[in] maxIter, maximum number of iterations in optimization, specified as an integer.
        * @param[in] funcTol, tolerance of the optimization cost function, specified as a double value.
        */
        void optimizePoseGraph(
            std::pair<std::vector<int>, std::vector<cv::Matx44d>>& viewIdsAndPoses,
            const std::vector<std::shared_ptr<Connection>>& odometryConnections,
            const std::vector<std::pair<std::shared_ptr<Connection>, double>>& loopConnections,
            std::vector<double>& optimScales,
            const int minNumMatches = 30,
            const int maxIter = 100,
            const double funcTol = 1e-15);

        /**
        * @brief Optimize a rigid pose graph in the stereo visual SLAM workflow
        *
        * @param[in, out] viewIdsAndPoses view Ids and view poses. View poses are updated after optimization.
        * @param[in] odometryConnections odometry connections, specified as a vector of pointers to Connection,
        *            which contains the relative orientation R and relative translation t. The relative pose of 
        *            an odometry connection is represented as a 3-D rigid transform:
        *            | R, t |
        *            | 0, 1 |
        * @param[in] loopConnections loop closure connections, specified as a vector of pointers to Connection,
        *            which contains the relative orientation R and relative translation t. The relative pose of 
        *            a loop closure connection is represented as a 3-D rigid transform:
        *            | R, t |
        *            | 0, 1 |
        * @param[in] minNumMatches, minimum number of matched features points between two views for them to be 
        *            counted as connected views, specified as an integer.
        * @param[in] maxIter, maximum number of iterations in optimization, specified as an integer.
        * @param[in] funcTol, tolerance of the optimization cost function, specified as a double value.
        */
        void optimizePoseGraph(
            std::pair<std::vector<int>, std::vector<cv::Matx44d>>& viewIdsAndPoses,
            const std::vector<std::shared_ptr<Connection>>& odometryConnections,
            const std::vector<std::shared_ptr<Connection>>& loopConnections,
            const int minNumMatches = 30,
            const int maxIter = 100,
            const double funcTol = 1e-15);
    }
}
#endif // OPTIMIZEPOSEGRAPH_G2O_HPP