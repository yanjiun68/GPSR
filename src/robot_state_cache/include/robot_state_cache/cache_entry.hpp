#ifndef ROBOT_STATE_CACHE__CACHE_ENTRY_HPP_
#define ROBOT_STATE_CACHE__CACHE_ENTRY_HPP_

// Section: A steady clock gives monotonic age calculations across system-clock
// corrections.
#include <chrono>
#include <cstdint>

namespace robot_state_cache {

template <typename T> struct CacheEntry {
  // Section: The accepted payload is retained independently from its freshness
  // state.
  T value;
  // Section: The monotonic acceptance timestamp drives deterministic age
  // evaluation.
  std::chrono::steady_clock::time_point timestamp;
  // Section: Sequence starts at one and changes only when input validation
  // accepts an update.
  std::uint64_t sequence{0U};
  // Section: Valid distinguishes populated entries from logical empty storage.
  bool valid{false};
};

} // namespace robot_state_cache

#endif // ROBOT_STATE_CACHE__CACHE_ENTRY_HPP_
