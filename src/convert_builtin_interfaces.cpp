// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "ros1_bridge/convert_builtin_interfaces.hpp"

namespace ros1_bridge
{

template<>
void
convert_1_to_2(
  const ros::Duration & ros1_type,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  ros2_msg.sec = ros1_type.sec;
  ros2_msg.nanosec = ros1_type.nsec;
}

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  ros::Duration & ros1_type)
{
  ros1_type.sec = ros2_msg.sec;
  ros1_type.nsec = ros2_msg.nanosec;
}


template<>
void
convert_1_to_2(
  const ros::Time & ros1_type,
  builtin_interfaces::msg::Time & ros2_msg)
{
  ros2_msg.sec = ros1_type.sec;
  ros2_msg.nanosec = ros1_type.nsec;
}

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  ros::Time & ros1_type)
{
  ros1_type.sec = ros2_msg.sec;
  ros1_type.nsec = ros2_msg.nanosec;
}


template<>
void
convert_1_to_2(
  const rosgraph_msgs::Log & ros1_msg,
  rcl_interfaces::msg::Log & ros2_msg)
{
  ros1_bridge::convert_1_to_2(ros1_msg.header.stamp, ros2_msg.stamp);

  // Explicitly map the values of the log level as they differ between
  // ROS1 and ROS2.
  switch (ros1_msg.level) {
    case rosgraph_msgs::Log::DEBUG:
      ros2_msg.level = rcl_interfaces::msg::Log::DEBUG;
      break;

    case rosgraph_msgs::Log::INFO:
      ros2_msg.level = rcl_interfaces::msg::Log::INFO;
      break;

    case rosgraph_msgs::Log::WARN:
      ros2_msg.level = rcl_interfaces::msg::Log::WARN;
      break;

    case rosgraph_msgs::Log::ERROR:
      ros2_msg.level = rcl_interfaces::msg::Log::ERROR;
      break;

    case rosgraph_msgs::Log::FATAL:
      ros2_msg.level = rcl_interfaces::msg::Log::FATAL;
      break;

    default:
      ros2_msg.level = ros1_msg.level;
      break;
  }

  ros2_msg.name = ros1_msg.name;
  ros2_msg.msg = ros1_msg.msg;
  ros2_msg.file = ros1_msg.file;
  ros2_msg.function = ros1_msg.function;
  ros2_msg.line = ros1_msg.line;
}

template<>
void
convert_2_to_1(
  const rcl_interfaces::msg::Log & ros2_msg,
  rosgraph_msgs::Log & ros1_msg)
{
  ros1_bridge::convert_2_to_1(ros2_msg.stamp, ros1_msg.header.stamp);

  // Explicitly map the values of the log level as they differ between
  // ROS1 and ROS2.
  switch (ros2_msg.level) {
    case rcl_interfaces::msg::Log::DEBUG:
      ros1_msg.level = rosgraph_msgs::Log::DEBUG;
      break;

    case rcl_interfaces::msg::Log::INFO:
      ros1_msg.level = rosgraph_msgs::Log::INFO;
      break;

    case rcl_interfaces::msg::Log::WARN:
      ros1_msg.level = rosgraph_msgs::Log::WARN;
      break;

    case rcl_interfaces::msg::Log::ERROR:
      ros1_msg.level = rosgraph_msgs::Log::ERROR;
      break;

    case rcl_interfaces::msg::Log::FATAL:
      ros1_msg.level = rosgraph_msgs::Log::FATAL;
      break;

    default:
      ros1_msg.level = ros1_msg.level;
      break;
  }

  ros1_msg.name = ros2_msg.name;
  ros1_msg.msg = ros2_msg.msg;
  ros1_msg.file = ros2_msg.file;
  ros1_msg.function = ros2_msg.function;
  ros1_msg.line = ros2_msg.line;
}

}  // namespace ros1_bridge
