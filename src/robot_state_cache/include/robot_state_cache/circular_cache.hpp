#ifndef ROBOT_STATE_CACHE__CIRCULAR_CACHE_HPP_
#define ROBOT_STATE_CACHE__CIRCULAR_CACHE_HPP_

// Section: Standard containers and synchronization implement bounded,
// thread-safe snapshots.
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "robot_state_cache/cache_entry.hpp"
#include "robot_state_cache/cache_status.hpp"

namespace robot_state_cache {

template <typename T> struct CacheSnapshot {
  // Section: One atomic view prevents timer consumers from combining values
  // from different updates.
  std::optional<CacheEntry<T>> latest;
  CacheStatus status{CacheStatus::EMPTY};
  std::optional<std::chrono::duration<double>> age;
  std::size_t history_count{0U};
  std::size_t capacity{0U};
};

template <typename T> class CircularCache {
public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using Entry = CacheEntry<T>;
  using Validator = std::function<bool(const T &)>;

  explicit CircularCache(const std::size_t capacity,
                         Validator validator = Validator{})
      : buffer_(capacity), capacity_(capacity),
        validator_(std::move(validator)) {
    // Special function: Preallocate every logical slot once and reject a
    // non-functional capacity.
    if (capacity_ == 0U) {
      throw std::invalid_argument(
          "CircularCache capacity must be greater than zero.");
    }
  }

  std::optional<Entry>
  update(const T &value, const Clock::time_point timestamp = Clock::now()) {
    // Special function: Validate first so a rejected observation cannot alter
    // history or held state.
    std::lock_guard<std::mutex> lock(mutex_);
    if (validator_ && !validator_(value)) {
      return std::nullopt;
    }
    Entry entry{value, timestamp, next_sequence_, true};
    ++next_sequence_;
    buffer_[write_index_] = entry;
    write_index_ = (write_index_ + 1U) % capacity_;
    valid_count_ = std::min(valid_count_ + 1U, capacity_);
    return entry;
  }

  std::optional<Entry> get_latest() const {
    // Special function: Copy the held value under lock so callers never retain
    // internal storage references.
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_unlocked();
  }

  std::optional<Entry> get_latest_valid(const Clock::time_point now,
                                        const Duration stale_timeout,
                                        const Duration expire_timeout) const {
    // Special function: Return a value only while its freshness state is
    // strictly VALID.
    const auto view = snapshot(now, stale_timeout, expire_timeout);
    if (view.status != CacheStatus::VALID) {
      return std::nullopt;
    }
    return view.latest;
  }

  std::vector<Entry> get_history() const {
    // Special function: Reconstruct logical history in documented
    // oldest-to-newest order.
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Entry> history;
    history.reserve(valid_count_);
    const std::size_t oldest_index =
        valid_count_ == capacity_ ? write_index_ : 0U;
    for (std::size_t offset = 0U; offset < valid_count_; ++offset) {
      const std::size_t index = (oldest_index + offset) % capacity_;
      if (buffer_[index].has_value() && buffer_[index]->valid) {
        history.push_back(*buffer_[index]);
      }
    }
    return history;
  }

  std::optional<Entry> get_by_sequence(const std::uint64_t sequence) const {
    // Special function: Search only populated slots while preserving the
    // bounded O(capacity) cost.
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &entry : buffer_) {
      if (entry.has_value() && entry->valid && entry->sequence == sequence) {
        return entry;
      }
    }
    return std::nullopt;
  }

  std::size_t get_history_count() const {
    // Special function: Report logical population without exposing the physical
    // write index.
    std::lock_guard<std::mutex> lock(mutex_);
    return valid_count_;
  }

  std::size_t size() const {
    // Special function: Preserve the conventional container alias for history
    // count.
    return get_history_count();
  }

  std::size_t capacity() const noexcept {
    // Special function: Capacity never changes after construction and therefore
    // needs no lock.
    return capacity_;
  }

  bool empty() const {
    // Special function: Distinguish an uninitialized cache from a stored
    // default-valued payload.
    return get_history_count() == 0U;
  }

  CacheStatus status(const Clock::time_point now, const Duration stale_timeout,
                     const Duration expire_timeout) const {
    // Special function: Evaluate freshness using the same atomic snapshot used
    // by ROS publication.
    return snapshot(now, stale_timeout, expire_timeout).status;
  }

  std::optional<std::chrono::duration<double>>
  age(const Clock::time_point now) const {
    // Special function: Return no age for EMPTY and clamp a future test
    // timestamp to zero.
    std::lock_guard<std::mutex> lock(mutex_);
    const auto latest = latest_unlocked();
    if (!latest.has_value()) {
      return std::nullopt;
    }
    return std::chrono::duration<double>(
        std::max(Duration::zero(), now - latest->timestamp));
  }

  CacheSnapshot<T> snapshot(const Clock::time_point now,
                            const Duration stale_timeout,
                            const Duration expire_timeout) const {
    // Special function: Copy held data and freshness metadata under one lock
    // for race-free consumers.
    validate_timeouts(stale_timeout, expire_timeout);
    std::lock_guard<std::mutex> lock(mutex_);
    CacheSnapshot<T> view;
    view.capacity = capacity_;
    view.history_count = valid_count_;
    view.latest = latest_unlocked();
    if (!view.latest.has_value()) {
      return view;
    }
    const Duration bounded_age =
        std::max(Duration::zero(), now - view.latest->timestamp);
    view.age = std::chrono::duration<double>(bounded_age);
    // Section: Equality at stale enters STALE; equality at expiration enters
    // EXPIRED.
    if (bounded_age < stale_timeout) {
      view.status = CacheStatus::VALID;
    } else if (bounded_age < expire_timeout) {
      view.status = CacheStatus::STALE;
    } else {
      view.status = CacheStatus::EXPIRED;
    }
    return view;
  }

  void clear() {
    // Special function: Reset logical contents and indices without changing
    // configured capacity or sequence history.
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &entry : buffer_) {
      entry.reset();
    }
    write_index_ = 0U;
    valid_count_ = 0U;
  }

private:
  static void validate_timeouts(const Duration stale_timeout,
                                const Duration expire_timeout) {
    // Special function: Reject negative or reversed thresholds before
    // evaluating cache state.
    if (stale_timeout < Duration::zero()) {
      throw std::invalid_argument("stale timeout must be non-negative.");
    }
    if (expire_timeout < stale_timeout) {
      throw std::invalid_argument(
          "expire timeout must be greater than or equal to stale timeout.");
    }
  }

  std::optional<Entry> latest_unlocked() const {
    // Special function: Locate the slot immediately preceding the next write
    // position.
    if (valid_count_ == 0U) {
      return std::nullopt;
    }
    const std::size_t latest_index =
        (write_index_ + capacity_ - 1U) % capacity_;
    return buffer_[latest_index];
  }

  // Section: Physical storage is allocated to final capacity once and never
  // resized by normal updates.
  std::vector<std::optional<Entry>> buffer_;
  const std::size_t capacity_;
  Validator validator_;
  // Section: Mutable indices and sequence state are protected for multithreaded
  // executors and consumers.
  mutable std::mutex mutex_;
  std::size_t write_index_{0U};
  std::size_t valid_count_{0U};
  std::uint64_t next_sequence_{1U};
};

} // namespace robot_state_cache

#endif // ROBOT_STATE_CACHE__CIRCULAR_CACHE_HPP_
