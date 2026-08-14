#pragma once

#include <boost/statechart/event.hpp>

namespace carry_my_luggage_phase1::events
{
struct EvFailsafe : boost::statechart::event<EvFailsafe>
{
};
}  // namespace carry_my_luggage_phase1::events
