#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <putrabot_interfaces/srv/get_waypoints.hpp>
#include <putrabot_interfaces/srv/save_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>

namespace carry_my_luggage_phase1
{
class WaypointStoreNode : public rclcpp::Node
{
public:
  WaypointStoreNode()
  : rclcpp::Node("waypoint_store")
  {
    save_service_ = create_service<putrabot_interfaces::srv::SaveWaypoints>(
      "waypoints/save",
      std::bind(&WaypointStoreNode::handleSave, this, std::placeholders::_1, std::placeholders::_2));

    get_service_ = create_service<putrabot_interfaces::srv::GetWaypoints>(
      "waypoints/get",
      std::bind(&WaypointStoreNode::handleGet, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Waypoint store ready: services 'waypoints/save' and 'waypoints/get'.");
  }

private:
  void handleSave(
    const std::shared_ptr<putrabot_interfaces::srv::SaveWaypoints::Request> request,
    std::shared_ptr<putrabot_interfaces::srv::SaveWaypoints::Response> response)
  {
    if (request->goals.size() != request->waypoint_ids.size()) {
      response->success = false;
      response->message = "goals and waypoint_ids size mismatch";
      RCLCPP_WARN(get_logger(), "SaveWaypoints rejected: %zu goals vs %zu ids",
        request->goals.size(), request->waypoint_ids.size());
      return;
    }

    goals_ = request->goals;
    waypoint_ids_ = request->waypoint_ids;

    response->success = true;
    response->message = "Saved " + std::to_string(goals_.size()) + " goals";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void handleGet(
    const std::shared_ptr<putrabot_interfaces::srv::GetWaypoints::Request> request,
    std::shared_ptr<putrabot_interfaces::srv::GetWaypoints::Response> response)
  {
    response->success = false;
    response->message = "Waypoint not found";

    // Find the goal that matches the requested waypoint id.
    for (size_t i = 0; i < waypoint_ids_.size(); ++i) {
      if (waypoint_ids_[i] == request->waypoint_id) {
        response->goal = goals_[i];
        response->success = true;
        response->message = "Found waypoint '" + request->waypoint_id + "'";
        break;
      }
    }

    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  std::vector<std::string> waypoint_ids_;

  rclcpp::Service<putrabot_interfaces::srv::SaveWaypoints>::SharedPtr save_service_;
  rclcpp::Service<putrabot_interfaces::srv::GetWaypoints>::SharedPtr get_service_;
};
}  // namespace carry_my_luggage_phase1

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<carry_my_luggage_phase1::WaypointStoreNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
