#pragma once

#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_acl/collision_env_acl.h>

namespace collision_detection
{

MOVEIT_CLASS_FORWARD(CollisionDetectorAllocatorACL);  // Defines CollisionDetectorAllocatorACLPtr, ConstPtr, WeakPtr... etc

/** \brief An allocator for ACL collision detectors */
class CollisionDetectorAllocatorACL
  : public CollisionDetectorAllocatorTemplate<CollisionEnvACL, CollisionDetectorAllocatorACL>
  , public collision_detection::acl::CollisionCallback
  , public std::enable_shared_from_this<collision_detection::acl::CollisionCallback>
{
public:
  static const std::string NAME;  // defined in collision_env_acl.cpp
  CollisionEnvPtr allocateEnv(const WorldPtr& world, const moveit::core::RobotModelConstPtr& robot_model) const override;

  CollisionEnvPtr allocateEnv(const moveit::core::RobotModelConstPtr& robot_model) const override;

//  const std::string& getName() const override;
//
  void setCollisionCallback(const collision_detection::acl::CollisionCallbackPtr& collisionCallback) {
      collisionCallback_ = collisionCallback;
  }

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state) override {
    assert(collisionCallback_ && "No collision callback is set!");
    collisionCallback_->checkRobotCollision(req, res, state);
  }

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state) override {
      assert(collisionCallback_ && "No collision callback is set!");
      collisionCallback_->checkSelfCollision(req, res, state);
  }

  std::vector<moveit::core::RobotState> getUnstuckPath(const moveit::core::RobotState& startState) override {
      assert(collisionCallback_ && "No collision callback is set!");
      return collisionCallback_->getUnstuckPath(startState);
  }

private:
  acl::CollisionCallbackPtr collisionCallback_;
};
}  // namespace collision_detection
