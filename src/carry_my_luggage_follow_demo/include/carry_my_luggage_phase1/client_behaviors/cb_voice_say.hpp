#pragma once

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <string>

namespace carry_my_luggage_phase1::client_behaviors
{
class CbVoiceSay : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbVoiceSay() = default;
  explicit CbVoiceSay(
    std::string text,
    std::string service_name = "/voice/tts/say",
    double timeout_sec = 10.0);

  void onEntry() override;

private:
  std::string text_;
  std::string service_name_{"/voice/tts/say"};
  double timeout_sec_{10.0};
};
}  // namespace carry_my_luggage_phase1::client_behaviors
