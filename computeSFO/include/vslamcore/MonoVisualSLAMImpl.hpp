////////////////////////////////////////////////////////////////////////////////
//  MonoVisualSLAMImpl.hpp
//
//  MonoVisualSLAMImpl class header file.
//
//  Copyright 2022-23 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef MONOVISUALSLAMIMPL_HPP
#define MONOVISUALSLAMIMPL_HPP

#include <fstream>
#include <algorithm>
#include <string>
#include <numeric>
#include <cassert>
#include <cmath>

#include "libmwvslamcore_util.hpp"

// utility functions
#include "detectAndExtractFeatures.hpp"
#include "converter.hpp"
#include "NodeIDGenerator.hpp"

// Modules
#include "createNewMapPoints.hpp"
#include "addNewKeyFrame.hpp"
#include "trackLocalMap.hpp"
#include "trackLastKeyFrame.hpp"
#include "initializeMap.hpp"

// Thread management
#include <mutex>
#include <chrono>
#include <thread>
#include "SafeQueue.hpp"
#include "ThreadManager.hpp"
#include "SafeX.hpp"

namespace vision {

    namespace vslam {

        // This class will be used for data transfer between threads
        struct QueueInstance {
            public:
                int currFrameIndex;
                int currKeyFrameId;
                std::vector<cv::KeyPoint> currPoints;
                cv::Mat currFeatures;
                cv::Matx33d currExtrinsics_R;
                cv::Vec3d currExtrinsics_t;
                cv::Matx33d lastPose_R;
                cv::Vec3d lastPose_t;
                std::vector<int> trackedMapPointIds;
                std::vector<int> trackedFeatureIndices;
                std::vector<int> localKeyFrameIds;
        };

        // Enum used for concurrency level
        enum class VSlamConcurrency : int {
            DEBUG,      // Single thread
            THREAD_L1,  // One separate thread for Tracking, Mapping, Loop closing
            THREAD_L2   // Three threads, one each for Tracking, Mapping, Loop closing
        };

        class LIBMWVSLAMCORE_API MonoVisualSLAMImpl {

        public:
            MonoVisualSLAMImpl() = delete;
            ~MonoVisualSLAMImpl();

            /**
            * @brief Constructor
            */
            MonoVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const Configuration& config,
                const char* vocabFile,
                const int threadLevel = 2);

             /**
             * @brief Add an image frame to the system and check if it is a key frame
             *
             * @param[in] img current frame represented as a pointer to cv::Mat
             *
             */
             void addFrame(const cv::Ptr<cv::Mat> img);

             /**
             * @brief Get the map points
             *
             * @return [X, Y, Z] map points represented as a vector of cv::Vec3d
             *
             */
            std::vector<cv::Vec3d> getWorldPoints();

             /**
             * @brief Get the camera poses of key frames
             *
             * @return a pair of vectors containing view Ids and camera poses of the key frames
             *
             */
             std::pair<std::vector<int>, std::vector<cv::Matx44d>> getCameraPoses();

             /**
             * @brief Get the key frame IDs
             *
             * @return a vector of key frame IDs
             *
             */
             std::vector<int> getKeyFrameIDs();

             /**
             * @brief Get the number of tracked feature points/map points in the current frame
             *
             * @return number of points as an integer
             *
             */
            int getNumTrackedPoints();

            /**
             * @brief Check whether a new key frame is added
             *
             * @return flag indicating if new key frame is added
             *
             */
            bool hasNewKeyFrame();

            /**
             * @brief Check whether the visual SLAM object is done 
             *        processing all added frames
             *
             * @return flag indicating all frames are processed
             *
             */
            bool isDone();

             /**
             * @brief Check if the map has been initialized
             *
             * @return flag indicating if map initialization is successful
             *
             */
            bool isMapInitialized() const;

             /**
             * @brief Check if pose graph optimization was performed recently
             * @param[in] flag indicating if the status will be reset after the call
             *
             * @return flag indicating if the loop was closed recently
             *
             */
            bool isLoopRecentlyClosed(bool reset=true);

            /**
             * @brief Closes all internal processes and clears all data
             *
             */
            void reset();

        private:

            /**
             * @brief Function for tracking thread
             *
             */
            void trackingModule();

            /**
             * @brief Function for mapping thread
             *
             */
            void mappingModule();

            /**
             * @brief Function for loop closing thread
             *
             */
            void loopClosingModule();

            /**
             * @brief Reset variables and initialize threads
             *
             * @param[in] createThreads set true to create threads
             * @param[in] resetData set true to reset internal data structures
             */
            void initialize(const bool createThreads, const bool resetData);

        private:

             /**
             * @brief Remove recently-created map points that are observed by less than 3 views.
             *  The purpose is to keep only map points with strong observability.
             *
             */
            void cullRecentMapPoints();

            /**
            * Configuration of the system
            */
            struct Configuration config;

            /**
            * Vocabulary file path
            */
            std::string vocabularyFilePath;
            
            /**
            * ID generator for poses and points
            */
            NodeIDGenerator generator;

            /**
            * Camera intrinsics matrix
            */
            cv::Matx33d intrinsics;

            /**
            * Flag indicating if map initialization is successful
            */
            bool mapInitialized{ false };

            /**
            * Flag indicating if the loop was closed recently
            */
            SafeX<bool> loopRecentlyClosed;

            /**
            * Verbose output display
            */
            bool verboseDisplay{ false };

            /**
            * Key frames
            */
            std::unique_ptr<KeyFrameSet> keyFrames;

            /**
            * Map points
            */
            std::unique_ptr<MapPointSet> mapPoints;

            /**
            * Loop closure detection database
            */
            std::unique_ptr<LoopClosureDatabase> loopDatabase;

            /**
            * Map between key frame view Id and frame index
            */
            std::vector<int> keyFrameId2Index;

            /**
            * IDs of map points created recently using triangulation
            */
            std::vector<int> recentMapPointIds;

            /**
            * Indices and IDs used in the tracking process
            */
            int currFrameIndex{ 0 }, lastKeyFrameIndex{ 0 };

            /**
            * Number of key frames passed since last pose graph optimization
            */
            int numKeyFramesSinceLastLoop{ 0 };

            /**
            * Thread manager storing all threads
            */
            ThreadManager threadManager;

            /**
            * Queue to store extracted keypoints and feature pairs
            */
            SafeQueue< QueueInstance > trackingQueue;

            /**
            * Queue to store mapping artifacts
            */
            SafeQueue< QueueInstance > mappingQueue;

            /**
            * Queue to store loop closing artifacts
            */
            SafeQueue< QueueInstance > loopClosingQueue;

            /**
            * Integer identifier for tracking thread
            */
            int trackingThreadIdx;

            /**
            * Integer identifier for mapping thread
            */
            int mappingThreadIdx;

            /**
            * Integer identifier for loop closure thread
            */
            int loopClosingThreadIdx;

            /**
            * This flag controls the life of running threads. 
            * If set to true, all threads will terminate
            */
            SafeX<bool> stopFlag;

            /**
            * Number of tracked points used for computing status of vslam
            */
            SafeX<int> numTrackedPoints;

            /**
            * This flag indicates whether tracking thread is processing
            */
            SafeX<bool> isTrackingInProgress;

            /**
            * This flag indicates whether mapping thread is processing
            */
            SafeX<bool> isMappingInProgress;

            /**
            * This flag indicates whether loop closure thread is processing
            */
            SafeX<bool> isLoopClosingInProgress;

            /**
            * This flag indicates whether loop detection was initiated
            */
            SafeX<bool> isLoopCorrectionInitiated;

            /**
            * This flag indicates whether a new key frame is added
            */
            SafeX<bool> isNewKeyFrameAdded;

            /**
            * This flag indicates whether we run in single thread or not
            */
            VSlamConcurrency threadConcurrency;

            /**
            * Flag used to terminate BA when loop is detected
            */
            bool abortBA;
        }; // class MonoVisualSLAMImpl
    }
} // namespace vision
#endif // MONOVISUALSLAMIMPL_HPP