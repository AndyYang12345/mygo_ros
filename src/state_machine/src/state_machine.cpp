#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "state_machine/robot_state_machine_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStateMachineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
