#pragma once

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <string>

namespace carry_my_luggage_phase1::client_behaviors
{
class CbVoiceListen : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbVoiceListen() = default;
  explicit CbVoiceListen(
    std::string transcript_key,
    float record_sec = 4.0F,
    std::string mic_device = "default",
    std::string whisper_model = "base",
    std::string whisper_language = "en",
    std::string action_name = "/voice/stt/listen",
    std::string wav_path = "");

  void onEntry() override;

private:
  std::string transcript_key_{"voice_last_transcript"};
  float record_sec_{4.0F};
  std::string mic_device_{"default"};
  std::string whisper_model_{"base"};
  std::string whisper_language_{"en"};
  std::string action_name_{"/voice/stt/listen"};
  std::string wav_path_;
  double server_timeout_sec_{10.0};
  double result_timeout_sec_{45.0};
};
}  // namespace carry_my_luggage_phase1::client_behaviors
