/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sergio Villanueva */

#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <aivot_msgs/srv/get_arm_position.hpp>
#include <aivot_msgs/srv/get_arm_pose.hpp>
#include <aivot_msgs/srv/get_gripper_position.hpp>
#include <aivot_msgs/srv/modify_scene_object.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/callback_group.hpp>
#include <isaac_ros_cumotion_interfaces/action/attach_object.hpp>
#include <string>

#define INV_HAND_IDX -1
#define RT_HAND_IDX 0
#define LF_HAND_IDX 1

namespace move_group
{
class MoveGroupManipulationAgentService : public MoveGroupCapability
{
public:
MoveGroupManipulationAgentService();

  void initialize() override;

private:

  std::string StrLowerCase (const std::string & value);
  bool HasSubStrI (const std::string & value, const std::string & query);
  int ArmIdx (const std::string & name);

  rclcpp::Service<aivot_msgs::srv::GetArmPosition>::SharedPtr get_arm_position_service_;
  rclcpp::Service<aivot_msgs::srv::GetArmPose>::SharedPtr get_arm_pose_service_;
  rclcpp::Service<aivot_msgs::srv::GetGripperPosition>::SharedPtr get_gripper_position_service_;
  rclcpp::Service<aivot_msgs::srv::ModifySceneObject>::SharedPtr modify_scene_object_service_;
  rclcpp_action::Client<isaac_ros_cumotion_interfaces::action::AttachObject>::SharedPtr attach_object_action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_action_client_;
};
}  // namespace move_group
