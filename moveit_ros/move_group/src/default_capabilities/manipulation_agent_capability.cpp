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
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/callback_group.hpp>
#include <algorithm>
#include <cctype>


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

    auto node = context_->moveit_cpp_->getNode();
    callback_group_action_client_ = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Gazebo
    gz_attach_pub_ = node->create_publisher<std_msgs::msg::Empty>(
        "/gz/attach_box1", rclcpp::QoS(1));

    gz_detach_pub_ = node->create_publisher<std_msgs::msg::Empty>(
        "/gz/detach_box1", rclcpp::QoS(1));

    attach_object_action_client_ = rclcpp_action::create_client<isaac_ros_cumotion_interfaces::action::AttachObject>(
        node, "/attach_object", callback_group_action_client_);

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
                RCLCPP_INFO(LOGGER, "Pose: position=(%.2f, %.2f, %.2f), orientation=(%.2f, %.2f, %.2f)",
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

    modify_scene_object_service_ = context_->moveit_cpp_->getNode()->create_service<aivot_msgs::srv::ModifySceneObject>(
        "modify_scene_object", [this, node](const std::shared_ptr<rmw_request_id_t>& request_header,
                                      const std::shared_ptr<aivot_msgs::srv::ModifySceneObject::Request>& req,
                                      const std::shared_ptr<aivot_msgs::srv::ModifySceneObject::Response>& res) {
        // Modify scene object service logic
        RCLCPP_INFO(LOGGER, "Received request to modify scene object: %s", req->object_id.name.c_str());

        int prevArmIdx = ArmIdx (req->prev_arm);
        int newArmIdx = ArmIdx (req->new_arm);
        isaac_ros_cumotion_interfaces::action::AttachObject::Goal goal;

        // Configuration of object to be attached, including its shape (sphere, cuboid, mesh), pose and scale.
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = context_->moveit_cpp_->getNode()->now();
        marker.ns = req->object_id.name;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.frame_locked = true;

        const auto & oc = req->object_cuboid;
        // Extract the 4×4 matrix (row-major):
        double rowmajor[16];
        for (int i = 0; i < 16; ++i) {
            rowmajor[i] = oc.affine[i];
        }
        Eigen::Isometry3d world_T_obj;
        world_T_obj.linear() = Eigen::Matrix3d{
            {rowmajor[0], rowmajor[1], rowmajor[2]},
            {rowmajor[4], rowmajor[5], rowmajor[6]},
            {rowmajor[8], rowmajor[9], rowmajor[10]}
        };
        world_T_obj.translation() = Eigen::Vector3d{rowmajor[3], rowmajor[7], rowmajor[11]};

        // Retrieve attachment frame from params (loaded from object_attachment_params.yaml)
        std::string attachment_frame;
        auto node_local = context_->moveit_cpp_->getNode();
        if (!node_local->get_parameter("object_attachment_gripper_frame_name", attachment_frame)) {
            // fallback: infer from arm name
            attachment_frame = HasSubStrI(req->new_arm, "left") ? "LEFTgripper_tcp" : "RIGHTgripper_tcp";
            RCLCPP_WARN(LOGGER, "Parameter object_attachment_gripper_frame_name not set, using %s", attachment_frame.c_str());
        }

        Eigen::Isometry3d gripper_T_obj = world_T_obj;
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(context_->planning_scene_monitor_);
            const moveit::core::RobotState& state = scene->getCurrentState();
            if (state.getRobotModel()->hasLinkModel(attachment_frame)) {
                Eigen::Isometry3d world_T_gripper = state.getGlobalLinkTransform(attachment_frame);
                gripper_T_obj = world_T_gripper.inverse() * world_T_obj;
            } else {
                RCLCPP_WARN(LOGGER, "Attachment frame %s not found in robot model, using world pose", attachment_frame.c_str());
            }
        }

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = gripper_T_obj.translation().x();
        pose_msg.position.y = gripper_T_obj.translation().y();
        pose_msg.position.z = gripper_T_obj.translation().z();
        Eigen::Quaterniond q(gripper_T_obj.linear());
        q.normalize();
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        pose_msg.orientation.w = q.w();
        marker.pose = pose_msg;

        // Scale from cuboid dims (max - min per axis)
        marker.scale.x = oc.cuboid[3] - oc.cuboid[0];
        marker.scale.y = oc.cuboid[4] - oc.cuboid[1];
        marker.scale.z = oc.cuboid[5] - oc.cuboid[2];

        goal.object_config = marker;
        // Compute the inscribed‐sphere radius = ½ × (smallest cuboid dimension)
        double min_edge = std::min({ marker.scale.x, marker.scale.y, marker.scale.z });
        goal.fallback_radius = min_edge * 0.5;

        // TODO (sergio): Add logic to select the action server based on the arm index
        
        if (prevArmIdx == INV_HAND_IDX && newArmIdx != INV_HAND_IDX) {
            // Remove the object from the scene and attach it to the new arm
            RCLCPP_INFO(LOGGER, "Removing object '%s' from the scene", req->object_id.name.c_str());
            goal.attach_object = true;           
    
        } else if (prevArmIdx != INV_HAND_IDX && newArmIdx == INV_HAND_IDX) {
            // Detach the object from the previous arm and add it to the scene
            RCLCPP_INFO(LOGGER, "Detaching object '%s' from the previous arm '%s'", req->object_id.name.c_str(), req->prev_arm.c_str());
            goal.attach_object = false;

        } else {
            // Not handled case: both arms are either valid or invalid
            RCLCPP_ERROR(LOGGER, "Invalid arm configuration: both previous and new arms are either valid or invalid");
            return;
        }

        // Call the action server to attach the object to the new arm
        if (!attach_object_action_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(LOGGER, "Action server for attaching object is not available");
            return;
        }
        auto future_goal_handle = attach_object_action_client_->async_send_goal(goal);
        if (future_goal_handle.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
        {
            auto goal_handle_ = future_goal_handle.get();
            if (!goal_handle_)
            {
                throw std::runtime_error("Goal was rejected by the action server");
            }
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Attach object goal call time out...");
            return;
        }

        auto goal_handle = future_goal_handle.get();

        // Wait for the result
        auto future_result = attach_object_action_client_->async_get_result(goal_handle);
        if (future_result.wait_for(std::chrono::seconds(30)) == std::future_status::ready)
        {
            auto goal_handle_ = future_goal_handle.get();
            if (!goal_handle_)
            {
                throw std::runtime_error("Goal was rejected by the Object Attachment action server");
            }
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Object Attachment goal execution time out...");
            return;
        }
        
        auto pub_empty = [](auto & pub) {
            std_msgs::msg::Empty msg;
            pub->publish(msg);
        };

        // The final result
        auto result = future_result.get();
        
        if (result.result->outcome.find("attached") != std::string::npos)
        {
            pub_empty(gz_attach_pub_);
            RCLCPP_INFO(LOGGER, "Object '%s' attached successfully to arm '%s'", req->object_id.name.c_str(), req->new_arm.c_str());
            res->error_info.description = result.result->outcome;
        }
        else if (result.result->outcome.find("Detached") != std::string::npos)
        {
            pub_empty(gz_detach_pub_);
            RCLCPP_INFO(LOGGER, "Object '%s' detached successfully from arm '%s'", req->object_id.name.c_str(), req->prev_arm.c_str());
            res->error_info.description = result.result->outcome;
        }
        else if (result.result->outcome.find("failed") != std::string::npos)
        {
            RCLCPP_ERROR(LOGGER, "Failed to modify object '%s'", req->object_id.name.c_str());
            res->error_info.description = result.result->outcome;
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Unexpected outcome for object '%s'", req->object_id.name.c_str());
            res->error_info.description = result.result->outcome;
        }
    });
}

int MoveGroupManipulationAgentService::ArmIdx (const std::string & name)
{
    if (HasSubStrI (name, "left")) {
        return LF_HAND_IDX;
    } else if (HasSubStrI (name, "right")) {
        return RT_HAND_IDX;
    }
    return INV_HAND_IDX;
}
bool MoveGroupManipulationAgentService::HasSubStrI (const std::string & value, const std::string & query)
{
    return (StrLowerCase(value).find(StrLowerCase(query)) != std::string::npos);

}

std::string MoveGroupManipulationAgentService::StrLowerCase (const std::string & value)
{
    std::string result = value;
    std::transform (result.begin (), result.end (), result.begin (), 
        [](unsigned char c) { return std::tolower (c); });
    return result;
}

}// namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupManipulationAgentService, move_group::MoveGroupCapability)
