#pragma once
#include <moveit/collision_detection/collision_common.h>

namespace collision_detection::acl {

    MOVEIT_CLASS_FORWARD(CollisionCallback);

    class CollisionCallback
    {
    public:
        virtual ~CollisionCallback() = default;

        virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const moveit::core::RobotState &state) = 0;
        virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const moveit::core::RobotState &state) = 0;
        virtual std::vector<moveit::core::RobotState> getUnstuckPath(const moveit::core::RobotState& startState) = 0;
    };
}
