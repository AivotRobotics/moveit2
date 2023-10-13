#pragma once

#include <moveit/collision_detection/collision_env.h>

#include "moveit/collision_detection_acl/collision_callback.h"

namespace collision_detection
{
/** \brief ACL implementation of the CollisionEnv */
class CollisionEnvACL : public CollisionEnv
{
public:
  CollisionEnvACL() = delete;

  CollisionEnvACL(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world,
                  const collision_detection::acl::CollisionCallbackPtr& collisionCallback = {});

  CollisionEnvACL(const moveit::core::RobotModelConstPtr& model,
                  const collision_detection::acl::CollisionCallbackPtr& collisionCallback = {});

  CollisionEnvACL(const CollisionEnvACL& other, const WorldPtr& world);

  ~CollisionEnvACL() override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                           const moveit::core::RobotState& state) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                           const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2) const override;

  std::vector<moveit::core::RobotState> getUnstuckPath(const moveit::core::RobotState& startState) const;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const moveit::core::RobotState& state) const override;

  void distanceRobot(const DistanceRequest& req, DistanceResult& res,
                     const moveit::core::RobotState& state) const override;

  void setWorld(const WorldPtr& world) override;

protected:

  /** \brief Bundles the different checkSelfCollision functions into a single function */
  void checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;

  /** \brief Bundles the different checkRobotCollision functions into a single function */
  void checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                 const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;
private:
  /** @brief The callback to perform collision checks */
  acl::CollisionCallbackPtr collisionCallback_;
};
}  // namespace collision_detection
