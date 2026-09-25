#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace robot_yasmin_tasks {

class SkillAvailabilityResource final : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Special function: Preserve the lifecycle capability marker node name.
  SkillAvailabilityResource()
      : rclcpp_lifecycle::LifecycleNode("skill_availability_resource") {}

  // Special function: Report successful configuration without allocating resources.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    return CallbackReturn::SUCCESS;
  }

  // Special function: Report successful activation as the capability-ready signal.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    return CallbackReturn::SUCCESS;
  }

  // Special function: Report successful deactivation without side effects.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    return CallbackReturn::SUCCESS;
  }

  // Special function: Report successful cleanup without side effects.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    return CallbackReturn::SUCCESS;
  }

  // Special function: Report successful shutdown without side effects.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    return CallbackReturn::SUCCESS;
  }
};

}  // namespace robot_yasmin_tasks

int main(int argc, char **argv) {
  // Section: Preserve lifecycle service responsiveness with a multithreaded executor.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::SkillAvailabilityResource>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
