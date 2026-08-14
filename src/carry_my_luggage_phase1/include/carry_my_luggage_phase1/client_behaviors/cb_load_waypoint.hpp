#pragma once

#include <chrono>
#include <fstream>
#include <future>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <putrabot_interfaces/srv/save_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <yaml-cpp/yaml.h>

namespace carry_my_luggage_phase1::client_behaviors
{
class CbLoadWaypoint : public smacc2::SmaccAsyncClientBehavior
{
public:
  explicit CbLoadWaypoint(const std::string & yaml_file_path)
  : yaml_file_path_(yaml_file_path)
  {
    // Keep constructor lightweight; ROS logger/node can be unavailable during static configuration.
  }

  void onEntry() override
  {
    using namespace std::chrono_literals;

    if (!loadWaypointsFromFile()) {
      postFailureEvent();
      return;
    }

    if (!load_ok_) {
      RCLCPP_WARN(getLogger(), "Waypoint file is invalid or empty.");
      postFailureEvent();
      return;
    }

    auto node = getNode();
    auto client = node->create_client<putrabot_interfaces::srv::SaveWaypoints>("waypoints/save");

    if (!client->wait_for_service(2s)) {
      RCLCPP_WARN(getLogger(), "waypoints/save service is not available.");
      postFailureEvent();
      return;
    }

    auto request = std::make_shared<putrabot_interfaces::srv::SaveWaypoints::Request>();
    request->goals = goals_;
    request->waypoint_ids = waypoint_ids_;

    auto future = client->async_send_request(request);
    const auto start = std::chrono::steady_clock::now();

    while (!isShutdownRequested() && rclcpp::ok() &&
      future.wait_for(100ms) != std::future_status::ready)
    {
      if ((std::chrono::steady_clock::now() - start) > 5s) {
        RCLCPP_WARN(getLogger(), "Timed out waiting for waypoints/save response.");
        postFailureEvent();
        return;
      }
    }

    if (!rclcpp::ok() || isShutdownRequested()) {
      postFailureEvent();
      return;
    }

    const auto response = future.get();
    if (response != nullptr && response->success) {
      RCLCPP_INFO(getLogger(), "Saved %zu waypoints via service.", goals_.size());
      postSuccessEvent();
      return;
    }

    const auto message = response != nullptr ? response->message : std::string("null response");
    RCLCPP_WARN(getLogger(), "Failed to save waypoints: %s", message.c_str());
    postFailureEvent();
  }

private:
  bool loadWaypointsFromFile()
  {
    goals_.clear();
    waypoint_ids_.clear();
    load_ok_ = false;

    RCLCPP_INFO_STREAM(getLogger(), "Loading waypoint file: " << yaml_file_path_);

    std::ifstream ifs(yaml_file_path_);
    if (!ifs.good()) {
      RCLCPP_ERROR_STREAM(getLogger(), "Waypoints file not found: " << yaml_file_path_);
      return false;
    }

    try {
      const YAML::Node root = YAML::Load(ifs);
      const YAML::Node waypoints_node = root["waypoints"];
      if (!waypoints_node || !waypoints_node.IsSequence()) {
        RCLCPP_ERROR_STREAM(
          getLogger(),
          "Invalid waypoint YAML format: missing 'waypoints' sequence in " << yaml_file_path_);
        return false;
      }

      for (size_t i = 0; i < waypoints_node.size(); ++i) {
        const auto wp_node = waypoints_node[i];
        const auto pose_node = wp_node["pose"];

        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = wp_node["frame_id"].as<std::string>();
        goal.pose.position.x = pose_node["position"]["x"].as<double>();
        goal.pose.position.y = pose_node["position"]["y"].as<double>();
        goal.pose.position.z = pose_node["position"]["z"].as<double>();
        goal.pose.orientation.x = pose_node["orientation"]["x"].as<double>();
        goal.pose.orientation.y = pose_node["orientation"]["y"].as<double>();
        goal.pose.orientation.z = pose_node["orientation"]["z"].as<double>();
        goal.pose.orientation.w = pose_node["orientation"]["w"].as<double>();

        goals_.push_back(goal);
        waypoint_ids_.push_back(wp_node["id"].as<std::string>());
      }

      load_ok_ = !goals_.empty();
      RCLCPP_INFO(getLogger(), "Parsed %zu waypoints.", goals_.size());
      return load_ok_;
    } catch (const YAML::Exception & ex) {
      RCLCPP_ERROR_STREAM(getLogger(), "Failed to parse waypoint YAML: " << ex.what());
      return false;
    }
  }

  std::string yaml_file_path_;
  bool load_ok_{false};
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  std::vector<std::string> waypoint_ids_;
};
}  // namespace carry_my_luggage_phase1::client_behaviors
