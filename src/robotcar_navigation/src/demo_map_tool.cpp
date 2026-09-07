#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DemoMapTool : public rclcpp::Node
{
public:
  DemoMapTool()
  : Node("demo_map_tool")
  {
    nav_pub_ = create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);
    result_sub_ = create_subscription<std_msgs::msg::String>(
      "/waterplus/navi_result", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_WARN(get_logger(), "[NavResultCallback] received result: %s", msg->data.c_str());
      });
    timer_ = create_wall_timer(1s, [this]() {
      if (sent_) {
        return;
      }
      std_msgs::msg::String msg;
      msg.data = "1";
      nav_pub_->publish(msg);
      sent_ = true;
      RCLCPP_INFO(get_logger(), "Published navigation command to waypoint: %s", msg.data.c_str());
    });
  }

private:
  bool sent_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoMapTool>());
  rclcpp::shutdown();
  return 0;
}
