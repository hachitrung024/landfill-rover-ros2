// Copyright 2026 Landfill Rover Maintainers
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

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/message_filter.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointcloudTransformNode : public rclcpp::Node
{
public:
  PointcloudTransformNode()
  : Node("pointcloud_transform_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/zed/zed_node/point_cloud/cloud_registered");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/lr/point_cloud/cloud_in_map");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.5);

    if (input_topic_.empty()) {
      throw std::invalid_argument("Parameter 'input_topic' must not be empty");
    }
    if (output_topic_.empty()) {
      throw std::invalid_argument("Parameter 'output_topic' must not be empty");
    }
    if (target_frame_.empty()) {
      throw std::invalid_argument("Parameter 'target_frame' must not be empty");
    }
    if (transform_timeout_sec_ < 0.0) {
      throw std::invalid_argument("Parameter 'transform_timeout_sec' must be non-negative");
    }

    const auto input_qos = rclcpp::SensorDataQoS();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto transform_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(transform_timeout_sec_));

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);
    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      tf_buffer_, target_frame_, 30,
      get_node_logging_interface(), get_node_clock_interface(),
      transform_timeout);
    tf_filter_->registerCallback(
      std::bind(&PointcloudTransformNode::pointcloud_callback, this, std::placeholders::_1));

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, input_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        tf_filter_->add(message);
      });

    RCLCPP_INFO(
      get_logger(), "Transforming PointCloud2: %s -> %s (target frame: %s)",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring PointCloud2 with an empty frame_id");
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, message->header.frame_id,
        rclcpp::Time(message->header.stamp));

      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(*message, transformed, transform);
      transformed.header.stamp = message->header.stamp;
      transformed.header.frame_id = target_frame_;
      publisher_->publish(transformed);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot transform point cloud from '%s' to '%s': %s",
        message->header.frame_id.c_str(), target_frame_.c_str(), exception.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double transform_timeout_sec_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<PointcloudTransformNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_transform_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
