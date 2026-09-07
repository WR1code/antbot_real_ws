#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotcar_navigation/nav2_client_utils.hpp"
#include "robotcar_navigation/srv/save_waypoints.hpp"
#include "std_srvs/srv/empty.hpp"

#define COLOR_GREEN "\033[32m"
#define COLOR_RESET "\033[0m"

namespace
{

void set_raw_mode()
{
  termios tty;
  tcgetattr(STDIN_FILENO, &tty);
  tty.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

void reset_raw_mode()
{
  termios tty;
  tcgetattr(STDIN_FILENO, &tty);
  tty.c_lflag |= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

bool manage_detect_action(
  const std::string & package_share,
  const std::string & save_file,
  const std::string & waypoint_numbers,
  const std::string & mode)
{
  const std::string script_path = package_share + "/scripts/manage_detect_action.py";
  std::stringstream command;
  command << "python3 \"" << script_path << "\""
          << " --input \"" << save_file << "\""
          << " --output \"" << save_file << "\""
          << " --waypoints \"" << waypoint_numbers << "\""
          << " --mode " << mode;
  return std::system(command.str().c_str()) == 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wp_saver");

  const std::string package_share =
    ament_index_cpp::get_package_share_directory("robotcar_navigation");
  const std::string default_save_file = package_share + "/config/waypoints.xml";
  const std::string save_file =
    node->declare_parameter<std::string>("save_file", default_save_file);

  auto save_client = node->create_client<robotcar_navigation::srv::SaveWaypoints>(
    "waterplus/save_waypoints");
  auto reload_client = node->create_client<std_srvs::srv::Empty>(
    "waterplus/reload_waypoints");

  RCLCPP_INFO(node->get_logger(), COLOR_GREEN "--- Waypoint save/action tool ---" COLOR_RESET);
  RCLCPP_INFO(node->get_logger(), COLOR_GREEN " [space]: save waypoints" COLOR_RESET);
  RCLCPP_INFO(node->get_logger(), COLOR_GREEN " [q]: add detect action" COLOR_RESET);
  RCLCPP_INFO(node->get_logger(), COLOR_GREEN " [d]: remove detect action" COLOR_RESET);
  RCLCPP_INFO(node->get_logger(), COLOR_GREEN " [x]: exit" COLOR_RESET);

  set_raw_mode();
  char key = 0;
  while (rclcpp::ok()) {
    const int n = read(STDIN_FILENO, &key, 1);
    if (n <= 0) {
      rclcpp::spin_some(node);
      usleep(10000);
      continue;
    }

    auto save_request = std::make_shared<robotcar_navigation::srv::SaveWaypoints::Request>();
    save_request->filename = save_file;
    robotcar_navigation::srv::SaveWaypoints::Response::SharedPtr save_response;

    if (key == ' ') {
      if (robotcar_navigation::call_service<robotcar_navigation::srv::SaveWaypoints>(
          node, save_client, save_request, save_response))
      {
        RCLCPP_INFO(node->get_logger(), "Saved waypoints to: %s", save_file.c_str());
      }
    } else if (key == 'q' || key == 'd') {
      reset_raw_mode();
      const std::string mode = key == 'q' ? "add" : "remove";
      std::cout << "\nInput waypoint names/numbers, e.g. 1,2,6: ";
      std::string input;
      std::getline(std::cin, input);
      if (!input.empty() &&
        robotcar_navigation::call_service<robotcar_navigation::srv::SaveWaypoints>(
          node, save_client, save_request, save_response) &&
        manage_detect_action(package_share, save_file, input, mode))
      {
        auto reload_request = std::make_shared<std_srvs::srv::Empty::Request>();
        std_srvs::srv::Empty::Response::SharedPtr reload_response;
        robotcar_navigation::call_service<std_srvs::srv::Empty>(
          node, reload_client, reload_request, reload_response);
      }
      set_raw_mode();
    } else if (key == 'x') {
      break;
    }
  }

  reset_raw_mode();
  rclcpp::shutdown();
  return 0;
}
