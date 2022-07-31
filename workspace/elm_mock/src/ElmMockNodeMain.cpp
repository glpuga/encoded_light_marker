/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

// ros2
#include <elm_mock/ElmMockNode.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elm_mock::ElmMockNode>());
  rclcpp::shutdown();
  return 0;
}