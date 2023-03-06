#pragma once

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/collision_detection_acl/collision_detector_allocator_acl.h>

namespace collision_detection
{
class CollisionDetectorACLPluginLoader : public CollisionPlugin
{
public:
  bool initialize(const planning_scene::PlanningScenePtr& scene) const override;
};
}  // namespace collision_detection
