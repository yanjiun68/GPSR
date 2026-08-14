#pragma once

#include <atomic>
#include <cstdint>

namespace carry_my_luggage_phase1::utils
{
inline std::atomic<std::int64_t> g_bringup_pid{-1};

inline void setBringupPid(std::int64_t pid)
{
  g_bringup_pid.store(pid, std::memory_order_relaxed);
}

inline std::int64_t getBringupPid()
{
  return g_bringup_pid.load(std::memory_order_relaxed);
}

inline void clearBringupPid()
{
  setBringupPid(-1);
}
}  // namespace carry_my_luggage_phase1::utils
