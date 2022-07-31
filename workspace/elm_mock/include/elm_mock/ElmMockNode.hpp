/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace elm_mock {
class ElmMockNode : public rclcpp::Node {
 public:
  ElmMockNode();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_input_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_output_pub_;

  void inputImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace elm_mock