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

#include "manipulation_agent_capability.h"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/move_group/capability_names.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define MAX_FLOAT std::numeric_limits<float>::max()

namespace move_group
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_move_group_default_capabilities.manipulation_agent_capability");

MoveGroupManipulationAgentService::MoveGroupManipulationAgentService() : MoveGroupCapability("ManipulationAgentService")
{
}

void MoveGroupManipulationAgentService::initialize()
{
  get_arm_position_service_ = context_->moveit_cpp_->getNode()->create_service<aivot_msgs::srv::GetArmPosition>(
      "get_arm_position", [this](const std::shared_ptr<rmw_request_id_t>& request_header,
                                 const std::shared_ptr<aivot_msgs::srv::GetArmPosition::Request>& req,
                                 const std::shared_ptr<aivot_msgs::srv::GetArmPosition::Response>& res) {
        // Get arm position service logic
        RCLCPP_INFO(LOGGER, "Received request for arm position");
        context_->planning_scene_monitor_->updateFrameTransforms();

        moveit::core::RobotState start_state =
            planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
        if (const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(req->arm_name))
        {
          std::vector<double> joint_values;
          start_state.copyJointGroupPositions(jmg, joint_values);
          res->position.resize(joint_values.size());
          for (std::size_t i = 0; i < joint_values.size(); ++i)
          {
            res->position[i] = joint_values[i];
          }
          RCLCPP_INFO(LOGGER, "Arm position retrieved successfully with joint positions: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                      res->position[0], res->position[1], res->position[2],
                      res->position[3], res->position[4], res->position[5]);
        }
        else
        {
          RCLCPP_ERROR(LOGGER, "Invalid group name provided for arm position");
        }
      });

    get_arm_pose_service_ = context_->moveit_cpp_->getNode()->create_service<aivot_msgs::srv::GetArmPose>(
    "get_arm_pose", [this]([[maybe_unused]] const std::shared_ptr<rmw_request_id_t>& request_header,
                            const std::shared_ptr<aivot_msgs::srv::GetArmPose::Request>& req,
                            const std::shared_ptr<aivot_msgs::srv::GetArmPose::Response>& res) {
        RCLCPP_INFO(LOGGER, "Received request for arm pose");
        context_->planning_scene_monitor_->updateFrameTransforms();
        moveit::core::RobotState start_state =
            planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();

        if (const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(req->arm_name))
        {
            const std::string& link_name = req->link_name;
            if (start_state.getLinkModel(link_name))
            {
                const Eigen::Isometry3d& link_transform = start_state.getGlobalLinkTransform(link_name);

                Eigen::Isometry3d pose = link_transform;
                
                if (req->link_offset.x != MAX_FLOAT && req->link_offset.y != MAX_FLOAT && req->link_offset.z != MAX_FLOAT) {
                    Eigen::Vector3d offset(req->link_offset.x, req->link_offset.y, req->link_offset.z);
                    pose.translation() += pose.linear() * offset;
                    RCLCPP_INFO(LOGGER, "Arm pose retrieved WITH LINK OFFSET");
                }

                tf2::Transform tf2_transform;
                tf2_transform.setOrigin(tf2::Vector3(pose.translation().x(),
                                                     pose.translation().y(),
                                                     pose.translation().z()));

                Eigen::Matrix3d rotation_matrix = pose.rotation();
                tf2::Matrix3x3 tf2_rotation(
                    rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                    rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                    rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
                );

                tf2::Quaternion tf2_quaternion;
                tf2_rotation.getRotation(tf2_quaternion);
                tf2_transform.setRotation(tf2_quaternion);
                geometry_msgs::msg::Pose pose_msg;
                tf2::toMsg(tf2_transform, pose_msg);

                res->position = pose_msg.position;

                tf2::Matrix3x3 mat(tf2::Quaternion(pose_msg.orientation.x,
                                                    pose_msg.orientation.y,
                                                    pose_msg.orientation.z,
                                                    pose_msg.orientation.w));
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);
                res->angle.x = roll;
                res->angle.y = pitch;
                res->angle.z = yaw;

                res->base_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();

                RCLCPP_INFO(LOGGER, "Arm pose retrieved successfully for link: %s", link_name.c_str());
                RCLCPP_INFO(LOGGER, "Pose: position=(%.2f, %.2f, %.2f), orientation=(%.2f, %.2f, %.2f, %.2f)",
                            res->position.x, res->position.y, res->position.z,
                            res->angle.x, res->angle.y, res->angle.z);
            }
            else
            {
                RCLCPP_ERROR(LOGGER, "Invalid link name provided for arm pose");
            }
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Invalid group name provided for arm pose");
        }
    });

    get_gripper_position_service_ = context_->moveit_cpp_->getNode()->create_service<aivot_msgs::srv::GetGripperPosition>(
        "get_gripper_position", [this](const std::shared_ptr<rmw_request_id_t>& request_header,
                                       const std::shared_ptr<aivot_msgs::srv::GetGripperPosition::Request>& req,
                                       const std::shared_ptr<aivot_msgs::srv::GetGripperPosition::Response>& res) {
        // Get gripper position service logic
        RCLCPP_INFO(LOGGER, "Received request for gripper position");

        context_->planning_scene_monitor_->updateFrameTransforms();
        moveit::core::RobotState start_state =
            planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();

        if (!req->finger_joints.empty()) {
            const std::string& joint_name = req->finger_joints[0];

            if (const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(req->arm_name))
            {
                if (start_state.getJointModel(joint_name) != nullptr)
                {
                    RCLCPP_INFO(LOGGER, "Retrieving position for joint '%s'", joint_name.c_str());
                    res->position = start_state.getJointPositions(joint_name)[0];
                    RCLCPP_INFO(LOGGER, "Position for joint '%s' retrieved successfully", joint_name.c_str());
                    RCLCPP_INFO(LOGGER, "Gripper position: %.2f", res->position);
                }
                else
                {
                    RCLCPP_ERROR(LOGGER, "Joint '%s' not found in the robot model", joint_name.c_str());
                    return;
                }
            }
            else
            {
                RCLCPP_ERROR(LOGGER, "Invalid group name provided for gripper position");
            }
        } else {
            RCLCPP_ERROR(LOGGER, "No finger joints provided in the request");
        }
    });
}  
}// namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupManipulationAgentService, move_group::MoveGroupCapability)
