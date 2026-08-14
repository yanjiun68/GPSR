#pragma once

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <string>

namespace carry_my_luggage_phase1::client_behaviors
{
class CbNavigateToWaypoint : public smacc2::SmaccAsyncClientBehavior
{
public:
  explicit CbNavigateToWaypoint(std::string waypoint_id = "");
  explicit CbNavigateToWaypoint(size_t waypoint_index);

  void onEntry() override;

private:
  bool use_index_{false};
  size_t waypoint_index_{0};
  std::string waypoint_id_;
};
}  // namespace carry_my_luggage_phase1::client_behaviors
