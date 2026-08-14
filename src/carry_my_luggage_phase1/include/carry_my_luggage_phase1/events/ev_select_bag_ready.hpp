#pragma once

#include <boost/statechart/event.hpp>

namespace carry_my_luggage_phase1::events
{
struct EvSelectBagReady : boost::statechart::event<EvSelectBagReady>
{
};
}  // namespace carry_my_luggage_phase1::events
