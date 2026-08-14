#pragma once

#include <boost/statechart/event.hpp>

namespace carry_my_luggage_phase1::events
{
struct EvRetryLimitExceeded : boost::statechart::event<EvRetryLimitExceeded>
{
};
}  // namespace carry_my_luggage_phase1::events
