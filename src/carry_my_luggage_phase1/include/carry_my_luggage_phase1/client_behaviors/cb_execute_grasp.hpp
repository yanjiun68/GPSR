#pragma once

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace carry_my_luggage_phase1::client_behaviors
{
class CbExecuteGrasp : public smacc2::SmaccAsyncClientBehavior
{
public:
  void onEntry() override;
};
}  // namespace carry_my_luggage_phase1::client_behaviors
