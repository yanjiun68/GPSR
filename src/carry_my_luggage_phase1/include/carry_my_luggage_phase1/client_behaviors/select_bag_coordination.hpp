#pragma once

#include <atomic>
#include <memory>

namespace carry_my_luggage_phase1::client_behaviors::detail
{
using SelectBagPeerFailureFlag = std::shared_ptr<std::atomic_bool>;

inline constexpr char kSelectBagPeerFailureFlagKey[] = "st_select_bag.peer_failure_flag";
}  // namespace carry_my_luggage_phase1::client_behaviors::detail
