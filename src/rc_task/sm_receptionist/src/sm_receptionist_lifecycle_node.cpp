#include <memory>

#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sm_receptionist
{
class SmReceptionistLifecycleNode : public nav2_util::LifecycleNode
{
public:
  using CallbackReturn = nav2_util::CallbackReturn;

  SmReceptionistLifecycleNode()
  : nav2_util::LifecycleNode("sm_receptionist_lifecycle")
  {
    declare_parameter<bool>("dry_run", true);
    declare_parameter<int>("target_guest_count", 2);
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(
      get_logger(),
      "Configured SMACC2 receptionist lifecycle: dry_run=%s target_guest_count=%ld",
      get_parameter("dry_run").as_bool() ? "true" : "false",
      get_parameter("target_guest_count").as_int());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    createBond();
    RCLCPP_INFO(get_logger(), "Activated SMACC2 receptionist lifecycle manager.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    RCLCPP_INFO(get_logger(), "Deactivated SMACC2 receptionist lifecycle manager.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    RCLCPP_INFO(get_logger(), "Cleaned up SMACC2 receptionist lifecycle manager.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    RCLCPP_INFO(get_logger(), "Shut down SMACC2 receptionist lifecycle manager.");
    return CallbackReturn::SUCCESS;
  }
};
}  // namespace sm_receptionist

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sm_receptionist::SmReceptionistLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
