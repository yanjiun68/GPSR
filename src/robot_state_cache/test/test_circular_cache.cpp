// Section: GTest validates deterministic ring behavior and freshness without
// requiring ROS middleware.
#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "robot_state_cache/circular_cache.hpp"

namespace {

using Cache = robot_state_cache::CircularCache<std::string>;
using Status = robot_state_cache::CacheStatus;
using namespace std::chrono_literals;

TEST(CircularCache, ConstructionIsEmptyWithFixedCapacity) {
  // Section: No default value may be mistaken for an accepted observation.
  Cache cache(5U);
  EXPECT_EQ(cache.capacity(), 5U);
  EXPECT_EQ(cache.size(), 0U);
  EXPECT_TRUE(cache.empty());
  EXPECT_FALSE(cache.get_latest().has_value());
  EXPECT_EQ(cache.status(Cache::Clock::now(), 500ms, 2s), Status::EMPTY);
}

TEST(CircularCache, RejectsZeroCapacity) {
  // Section: A zero-slot cache cannot satisfy deterministic overwrite
  // semantics.
  EXPECT_THROW(Cache(0U), std::invalid_argument);
}

TEST(CircularCache, SequentialInsertionPreservesOldestToNewestOrder) {
  // Section: Partial buffers begin logically at physical slot zero.
  Cache cache(5U);
  cache.update("A");
  cache.update("B");
  cache.update("C");
  const auto history = cache.get_history();
  ASSERT_EQ(history.size(), 3U);
  EXPECT_EQ(history[0].value, "A");
  EXPECT_EQ(history[1].value, "B");
  EXPECT_EQ(history[2].value, "C");
  ASSERT_TRUE(cache.get_latest().has_value());
  EXPECT_EQ(cache.get_latest()->value, "C");
}

TEST(CircularCache, FullBufferContainsEveryEntry) {
  // Section: Filling exactly to capacity must not overwrite or reorder an
  // entry.
  Cache cache(3U);
  cache.update("A");
  cache.update("B");
  cache.update("C");
  const auto history = cache.get_history();
  ASSERT_EQ(history.size(), 3U);
  EXPECT_EQ(history[0].value, "A");
  EXPECT_EQ(history[1].value, "B");
  EXPECT_EQ(history[2].value, "C");
}

TEST(CircularCache, WraparoundOverwritesOldestFirst) {
  // Section: A B C D E at capacity three must expose C D E while capacity
  // remains fixed.
  Cache cache(3U);
  for (const auto *value : {"A", "B", "C", "D", "E"}) {
    cache.update(value);
  }
  const auto history = cache.get_history();
  ASSERT_EQ(history.size(), 3U);
  EXPECT_EQ(cache.capacity(), 3U);
  EXPECT_EQ(history[0].value, "C");
  EXPECT_EQ(history[1].value, "D");
  EXPECT_EQ(history[2].value, "E");
  EXPECT_EQ(cache.get_latest()->value, "E");
}

TEST(CircularCache, CapacityFiveScenarioMatchesRequiredProgression) {
  // Section: Verify the specified A-through-G progression at every overwrite
  // boundary.
  Cache cache(5U);
  std::vector<std::string> expected;
  for (const auto *value : {"A", "B", "C", "D", "E", "F", "G"}) {
    cache.update(value);
    expected.push_back(value);
    if (expected.size() > 5U) {
      expected.erase(expected.begin());
    }
    const auto history = cache.get_history();
    ASSERT_EQ(history.size(), expected.size());
    for (std::size_t index = 0U; index < expected.size(); ++index) {
      EXPECT_EQ(history[index].value, expected[index]);
    }
    EXPECT_EQ(cache.get_latest()->value, value);
    EXPECT_EQ(cache.capacity(), 5U);
  }
}

TEST(CircularCache, SequenceAdvancesOnlyForAcceptedUpdates) {
  // Section: The validator rejection path must retain the previous held value
  // and sequence.
  Cache cache(3U, [](const std::string &value) { return !value.empty(); });
  const auto first = cache.update("A");
  const auto rejected = cache.update("");
  const auto second = cache.update("B");
  ASSERT_TRUE(first.has_value());
  EXPECT_FALSE(rejected.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->sequence, 1U);
  EXPECT_EQ(second->sequence, 2U);
  EXPECT_EQ(cache.get_history_count(), 2U);
  EXPECT_EQ(cache.get_latest()->value, "B");
  EXPECT_EQ(cache.get_by_sequence(1U)->value, "A");
  EXPECT_FALSE(cache.get_by_sequence(3U).has_value());
}

TEST(CircularCache, ClearKeepsCapacityAndMonotonicSequence) {
  // Section: Clear removes logical contents but does not resize or reuse an
  // accepted sequence number.
  Cache cache(2U);
  EXPECT_EQ(cache.update("A")->sequence, 1U);
  cache.clear();
  EXPECT_TRUE(cache.empty());
  EXPECT_EQ(cache.capacity(), 2U);
  EXPECT_EQ(cache.status(Cache::Clock::now(), 500ms, 2s), Status::EMPTY);
  EXPECT_EQ(cache.update("B")->sequence, 2U);
}

TEST(CircularCache, FreshnessBoundariesAndRecoveryAreExact) {
  // Section: Equality at thresholds follows VALID -> STALE -> EXPIRED exactly
  // as documented.
  Cache cache(2U);
  const auto origin = Cache::Clock::time_point(10s);
  cache.update("A", origin);
  EXPECT_EQ(cache.status(origin + 499ms, 500ms, 2s), Status::VALID);
  EXPECT_EQ(cache.status(origin + 500ms, 500ms, 2s), Status::STALE);
  EXPECT_EQ(cache.status(origin + 1999ms, 500ms, 2s), Status::STALE);
  EXPECT_EQ(cache.status(origin + 2s, 500ms, 2s), Status::EXPIRED);
  EXPECT_TRUE(cache.get_latest().has_value());
  EXPECT_FALSE(cache.get_latest_valid(origin + 2s, 500ms, 2s).has_value());
  cache.update("B", origin + 3s);
  EXPECT_EQ(cache.status(origin + 3s, 500ms, 2s), Status::VALID);
}

TEST(CircularCache, InvalidTimeoutRelationshipsAreRejected) {
  // Section: Status evaluation never accepts negative or reversed timing
  // policy.
  Cache cache(1U);
  cache.update("A");
  EXPECT_THROW(cache.status(Cache::Clock::now(), -1ms, 2s),
               std::invalid_argument);
  EXPECT_THROW(cache.status(Cache::Clock::now(), 2s, 1s),
               std::invalid_argument);
}

TEST(CircularCache, ConcurrentReadsAndWritesStayBounded) {
  // Section: Multiple writers and a concurrent snapshot reader exercise the
  // internal mutex contract.
  constexpr std::size_t capacity = 32U;
  constexpr int updates_per_writer = 2000;
  Cache cache(capacity);
  std::atomic<bool> writers_finished{false};
  std::thread reader([&cache, &writers_finished]() {
    while (!writers_finished.load()) {
      const auto snapshot = cache.snapshot(Cache::Clock::now(), 1s, 2s);
      EXPECT_LE(snapshot.history_count, snapshot.capacity);
      const auto history = cache.get_history();
      EXPECT_LE(history.size(), cache.capacity());
    }
  });
  std::thread writer_a([&cache]() {
    for (int index = 0; index < updates_per_writer; ++index) {
      cache.update("A" + std::to_string(index));
    }
  });
  std::thread writer_b([&cache]() {
    for (int index = 0; index < updates_per_writer; ++index) {
      cache.update("B" + std::to_string(index));
    }
  });
  writer_a.join();
  writer_b.join();
  writers_finished.store(true);
  reader.join();
  EXPECT_EQ(cache.size(), capacity);
  EXPECT_EQ(cache.get_history().size(), capacity);
  ASSERT_TRUE(cache.get_latest().has_value());
  EXPECT_EQ(cache.get_latest()->sequence,
            static_cast<std::uint64_t>(updates_per_writer * 2));
}

} // namespace
