// Wiggle recovery behavior for Mars Rover - Created for wiggle_integration, Dec 2025
// Based on Nav2 BackUp behavior pattern
// https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/include/nav2_behaviors/plugins/back_up.hpp
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

#ifndef ROVER_BEHAVIORS__PLUGINS__WIGGLE_HPP_
#define ROVER_BEHAVIORS__PLUGINS__WIGGLE_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"

namespace rover_behaviors
{
    using BackUpAction = nav2_msgs::action::BackUp;

    /**
     * @class rover_behaviors::Wiggle
     * @brief A recovery behavior that performs oscillating movements to help get unstuck
     * 
     * This behavior alternates between forward and backward movements in a wiggling pattern
     * to help the rover get unstuck or improve localization. It disables collision checking
     * similar to CustomSpin to avoid false positives.
     */
    class Wiggle : public nav2_behaviors::TimedBehavior<BackUpAction>
    {
    public:
        /**
         * @brief A constructor for rover_behaviors::Wiggle
         */
        Wiggle();
        ~Wiggle();

        /**
         * @brief Initialization to run behavior
         * @param command Goal to execute
         * @return Status of behavior
         */
        nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

        /**
         * @brief Configuration of behavior action
         */
        void onConfigure() override;

        /**
         * @brief Loop function to run behavior
         * @return Status of behavior
         */
        nav2_behaviors::Status onCycleUpdate() override;

    protected:
        BackUpAction::Feedback::SharedPtr feedback_;

        double min_linear_vel_;
        double max_linear_vel_;
        double linear_acc_lim_;
        double simulate_ahead_time_;
        
        // Wiggle-specific parameters
        int wiggle_cycles_;
        int current_cycle_;
        bool moving_forward_;
        double wiggle_distance_;
        double distance_traveled_;
        
        rclcpp::Duration command_time_allowance_{0, 0};
        rclcpp::Time end_time_;
        geometry_msgs::msg::PoseStamped initial_pose_;
    };

} // namespace rover_behaviors

#endif // ROVER_BEHAVIORS__PLUGINS__WIGGLE_HPP_
