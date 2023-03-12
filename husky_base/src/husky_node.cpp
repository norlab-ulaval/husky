#include "husky_base/husky_hardware.hpp"
#include "husky_msgs/msg/husky_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>

class HuskyNode : public rclcpp::Node
{
public:
  HuskyNode() : Node("husky_node"), husky()
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&HuskyNode::diagnostics_callback, this));
  }

private:
  void diagnostics_callback()
  {
    auto node = std::make_shared<rclcpp::Node>(shared_from_this());
    husky_msgs::msg::HuskyStatus husky_status_msg = husky.updateDiagnostics(node);
    diagnostic_publisher_->publish(husky_status_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  // TODO: Create a constructor for husky
  husky_base::HuskyHardware husky;
  rclcpp::Publisher<husky_msgs::msg::HuskyStatus>::SharedPtr diagnostic_publisher_;
};

using namespace husky_base;

/**
 * Diagnostics loop for Husky, not realtime safe
 */
void diagnosticLoop(husky_base::HuskyHardware& husky, std::shared_ptr<rclcpp::Node> node)
{
  husky.updateDiagnostics(node);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<HuskyNode>();

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }
}
