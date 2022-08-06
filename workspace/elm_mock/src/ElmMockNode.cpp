/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

// project
#include <elm_mock/ElmMockNode.hpp>

namespace elm_mock {

ElmMockNode::ElmMockNode() : Node("elm_mock_node") {
  image_input_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "~/input/image_raw", 10,
      std::bind(&ElmMockNode::inputImageCallback, this, std::placeholders::_1));

  image_output_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("~/output/img_output", 1);
}

void ElmMockNode::inputImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  image_output_pub_->publish(*msg);
}

} // namespace elm_mock