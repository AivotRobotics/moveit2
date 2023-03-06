#include <moveit/collision_detection_acl/collision_detector_acl_plugin_loader.h>
#include <pluginlib/class_list_macros.hpp>

namespace collision_detection
{
bool CollisionDetectorACLPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene) const
{
  scene->allocateCollisionDetector(CollisionDetectorAllocatorACL::create());
  return true;
}
}  // namespace collision_detection

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionDetectorACLPluginLoader, collision_detection::CollisionPlugin)
