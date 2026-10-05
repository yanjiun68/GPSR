#ifndef ROBOT_STATE_CACHE__CACHE_STATUS_HPP_
#define ROBOT_STATE_CACHE__CACHE_STATUS_HPP_

// Section: Standard strings provide stable diagnostic names without a ROS
// dependency.
#include <string>

namespace robot_state_cache {

// Section: Freshness states distinguish absence from usable, aging, and expired
// held data.
enum class CacheStatus {
  EMPTY,
  VALID,
  STALE,
  EXPIRED,
};

inline std::string to_string(const CacheStatus status) {
  // Special function: Return stable uppercase state labels for logs, tests, and
  // ROS diagnostics.
  switch (status) {
  case CacheStatus::EMPTY:
    return "EMPTY";
  case CacheStatus::VALID:
    return "VALID";
  case CacheStatus::STALE:
    return "STALE";
  case CacheStatus::EXPIRED:
    return "EXPIRED";
  }
  // Section: Defensive fallback protects diagnostics if a corrupted enum value
  // is observed.
  return "UNKNOWN";
}

} // namespace robot_state_cache

#endif // ROBOT_STATE_CACHE__CACHE_STATUS_HPP_
