#include <rclcpp/rclcpp.hpp>

class ExplorerNode : public rclcpp::Node {
public:
  ExplorerNode() : Node("explorer_node") {
    RCLCPP_INFO(this->get_logger(), "Custom explorer node started");
    // Placeholder: could publish frontier info etc.
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExplorerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
