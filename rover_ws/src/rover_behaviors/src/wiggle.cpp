// Wiggle recovery behavior for Mars Rover - Created for wiggle_integration, Dec 2025
// Based on Nav2 BackUp behavior pattern with oscillating movement
// https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/plugins/back_up.cpp
// https://docs.nav2.org/plugin_tutorials/docs/writing_new_behavior_plugin.html

// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <chrono>
#include <memory>
#include <utility>

#include "rover_behaviors/wiggle.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace rover_behaviors
{

    Wiggle::Wiggle()
        : TimedBehavior<BackUpAction>(),
          feedback_(std::make_shared<BackUpAction::Feedback>()),
          min_linear_vel_(0.0),
          max_linear_vel_(0.0),
          linear_acc_lim_(0.0),
          simulate_ahead_time_(0.0),
          wiggle_cycles_(3),
          current_cycle_(0),
          moving_forward_(false),
          wiggle_distance_(0.3)
    {
    }

    Wiggle::~Wiggle() = default;

    void Wiggle::onConfigure()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node"};
        }

        nav2_util::declare_parameter_if_not_declared(
            node,
            "simulate_ahead_time", rclcpp::ParameterValue(2.0));
        node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

        nav2_util::declare_parameter_if_not_declared(
            node,
            "max_linear_vel", rclcpp::ParameterValue(0.5));
        node->get_parameter("max_linear_vel", max_linear_vel_);

        nav2_util::declare_parameter_if_not_declared(
            node,
            "min_linear_vel", rclcpp::ParameterValue(0.1));
        node->get_parameter("min_linear_vel", min_linear_vel_);

        nav2_util::declare_parameter_if_not_declared(
            node,
            "linear_acc_lim", rclcpp::ParameterValue(2.5));
        node->get_parameter("linear_acc_lim", linear_acc_lim_);

        nav2_util::declare_parameter_if_not_declared(
            node,
            "wiggle_cycles", rclcpp::ParameterValue(3));
        node->get_parameter("wiggle_cycles", wiggle_cycles_);

        nav2_util::declare_parameter_if_not_declared(
            node,
            "wiggle_distance", rclcpp::ParameterValue(0.3));
        node->get_parameter("wiggle_distance", wiggle_distance_);
    }

    nav2_behaviors::Status Wiggle::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
    {
        if (!nav2_util::getCurrentPose(
                initial_pose_, *tf_, global_frame_, robot_base_frame_,
                transform_tolerance_))
        {
            RCLCPP_ERROR(logger_, "Current robot pose is not available.");
            return nav2_behaviors::Status::FAILED;
        }

        // Initialize wiggle state
        current_cycle_ = 0;
        moving_forward_ = false; // Start with backward movement

        // Wiggle behavior uses parameter-based distances rather than the BackUpAction target
        // The time_allowance is used to prevent infinite wiggle attempts
        command_time_allowance_ = command->time_allowance;
        end_time_ = this->clock_->now() + command_time_allowance_;

        RCLCPP_INFO(
            logger_, "Starting wiggle recovery behavior with %d cycles, %0.2fm per wiggle.",
            wiggle_cycles_, wiggle_distance_);

        return nav2_behaviors::Status::SUCCEEDED;
    }

    nav2_behaviors::Status Wiggle::onCycleUpdate()
    {
        rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
        if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0)
        {
            stopRobot();
            RCLCPP_WARN(
                logger_,
                "Exceeded time allowance before completing wiggle - Exiting Wiggle");
            return nav2_behaviors::Status::FAILED;
        }

        // Check if we've completed all wiggle cycles
        if (current_cycle_ >= wiggle_cycles_)
        {
            stopRobot();
            RCLCPP_INFO(logger_, "Completed %d wiggle cycles successfully.", wiggle_cycles_);
            return nav2_behaviors::Status::SUCCEEDED;
        }

        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(
                current_pose, *tf_, global_frame_, robot_base_frame_,
                transform_tolerance_))
        {
            RCLCPP_ERROR(logger_, "Current robot pose is not available.");
            return nav2_behaviors::Status::FAILED;
        }

        // Calculate distance traveled in current wiggle direction
        double dx = current_pose.pose.position.x - initial_pose_.pose.position.x;
        double dy = current_pose.pose.position.y - initial_pose_.pose.position.y;
        double current_distance = std::sqrt(dx * dx + dy * dy);

        // Check if we've completed the current wiggle movement
        if (current_distance >= wiggle_distance_)
        {
            // Switch direction and reset reference pose
            // This automatically resets distance calculation on next cycle
            moving_forward_ = !moving_forward_;
            initial_pose_ = current_pose;
            
            // Increment cycle counter when completing a forward-backward pair
            if (moving_forward_)
            {
                current_cycle_++;
                RCLCPP_INFO(logger_, "Completed wiggle cycle %d/%d", current_cycle_, wiggle_cycles_);
            }
        }

        // Calculate velocity based on remaining distance in current direction
        double remaining_distance = std::max(0.0, wiggle_distance_ - current_distance);
        double vel = std::sqrt(2 * linear_acc_lim_ * remaining_distance);
        vel = std::min(std::max(vel, min_linear_vel_), max_linear_vel_);

        // Create velocity command
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        
        // Set velocity direction (negative for backward, positive for forward)
        if (moving_forward_)
        {
            cmd_vel->linear.x = vel;
        }
        else
        {
            cmd_vel->linear.x = -vel;
        }

        /**
         * IMPORTANT! Similar to CustomSpin, we disable collision checking here
         * as it was causing false positives in real-world testing.
         * The wiggle behavior is specifically designed for recovery when stuck,
         * so collision avoidance would defeat its purpose.
         */

        // Publish velocity command
        vel_pub_->publish(std::move(cmd_vel));

        // Update and publish feedback
        feedback_->distance_traveled = static_cast<float>(current_distance);
        action_server_->publish_feedback(feedback_);

        return nav2_behaviors::Status::RUNNING;
    }

} // namespace rover_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_behaviors::Wiggle, nav2_core::Behavior)
