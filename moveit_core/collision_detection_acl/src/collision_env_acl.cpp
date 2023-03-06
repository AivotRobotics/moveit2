#include <moveit/collision_detection_acl/collision_env_acl.h>
#include <moveit/collision_detection_acl/collision_detector_allocator_acl.h>

namespace collision_detection
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_collision_detection_acl.collision_env_acl");
const std::string CollisionDetectorAllocatorACL::NAME("ACL");

CollisionEnvACL::CollisionEnvACL(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, const collision_detection::acl::CollisionCallbackPtr& collisionCallback)
  : CollisionEnvFCL(model, world)
  , collisionCallback_(collisionCallback)
{}

CollisionEnvACL::CollisionEnvACL(const moveit::core::RobotModelConstPtr& model, const collision_detection::acl::CollisionCallbackPtr& collisionCallback)
    : CollisionEnvFCL(model)
    , collisionCallback_(collisionCallback)
{}

CollisionEnvACL::~CollisionEnvACL()
{}

CollisionEnvACL::CollisionEnvACL(const CollisionEnvACL& other, const WorldPtr& world)
  : CollisionEnvFCL(other, world)
  , collisionCallback_(other.collisionCallback_)
{}

void CollisionEnvACL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvACL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionEnvACL::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state,
                                               const AllowedCollisionMatrix* acm) const
{
  moveit::tools::Profiler::ScopedBlock sblock("CollisionEnvACL::checkSelfCollision");

  assert(collisionCallback_ && "Self collision callback must be set!");
  collisionCallback_->checkSelfCollision(req, res, state);
}

void CollisionEnvACL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                          const moveit::core::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvACL:: checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                          const moveit::core::RobotState& state,
                                          const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, state, &acm);
}

void CollisionEnvACL::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                          const moveit::core::RobotState& /*state1*/,
                                          const moveit::core::RobotState& /*state2*/) const
{
  RCLCPP_ERROR(LOGGER, "Continuous collision not implemented");
}

void CollisionEnvACL::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                          const moveit::core::RobotState& /*state1*/,
                                          const moveit::core::RobotState& /*state2*/,
                                          const AllowedCollisionMatrix& /*acm*/) const
{
  RCLCPP_ERROR(LOGGER, "Not implemented");
}

void CollisionEnvACL::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                const moveit::core::RobotState& state,
                                                const AllowedCollisionMatrix* acm) const
{
  moveit::tools::Profiler::ScopedBlock sblock("CollisionEnvACL::checkRobotCollision");

  assert(collisionCallback_ && "Robot collision callback must be set!");
  collisionCallback_->checkRobotCollision(req, res, state);

  if (req.distance)
  {
    assert(false);
  }
}

std::vector<moveit::core::RobotState> CollisionEnvACL::getUnstuckPath(const moveit::core::RobotState& startState) const
{
    moveit::tools::Profiler::ScopedBlock sblock("CollisionEnvACL::getUnstuckPath");

    assert(collisionCallback_ && "Robot collision callback must be set!");
    return collisionCallback_->getUnstuckPath(startState);
}

void CollisionEnvACL::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                   const moveit::core::RobotState& state) const
{
  CollisionEnvFCL::distanceSelf(req, res, state);
}

void CollisionEnvACL::distanceRobot(const DistanceRequest& req, DistanceResult& res,
                                    const moveit::core::RobotState& state) const
{
  assert(false);
}

void CollisionEnvACL::setWorld(const WorldPtr& world)
{
  // no-op
}

//const std::string& CollisionDetectorAllocatorACL::getName() const
//{
//  return NAME;
//}
//
CollisionEnvPtr CollisionDetectorAllocatorACL::allocateEnv(const WorldPtr& world, const moveit::core::RobotModelConstPtr& robot_model) const {
    return CollisionEnvPtr(new CollisionEnvACL(robot_model, world, const_cast<CollisionDetectorAllocatorACL*>(this)->shared_from_this()));
}

CollisionEnvPtr CollisionDetectorAllocatorACL::allocateEnv(const moveit::core::RobotModelConstPtr& robot_model) const {
    return CollisionEnvPtr(new CollisionEnvACL(robot_model, const_cast<CollisionDetectorAllocatorACL*>(this)->shared_from_this()));
}
}  // namespace collision_detection
