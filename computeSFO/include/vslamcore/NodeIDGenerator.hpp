////////////////////////////////////////////////////////////////////////////////
// ID generator class, used to generate pointIds and viewIds
// 
// Copyright 2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#ifndef IDGENERATOR_HPP
#define IDGENERATOR_HPP


#include <unordered_map>
#include <utility>
#include <vector>

namespace vision {
    namespace vslam {
        enum class NodeType  { POINT_XYZ, POSE_SE3, VEL3, IMU_BIAS };

        struct NodeID {
            int identifier;
            NodeType type;
        };

        class NodeIDGenerator {

        private:
            int counter;
            std::vector<NodeID> allIDs;

        public:
            NodeIDGenerator();
            NodeID newIDs(const NodeType type);
            std::vector<NodeID> newIDs(const NodeType type, const int N);
        };
    }// namespace vslam
}// namespace vision
#endif //IDGENERATOR_HPP
