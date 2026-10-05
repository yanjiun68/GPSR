#!/usr/bin/env python3

from difflib import get_close_matches
import os
import subprocess
import tempfile
import threading
import time
from typing import Optional
import re
import wave

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from voice_interfaces.action import Receptionist, TranscribeSpeech

try:
    # Section: Preserve Receptionist's per-goal voice-path field with cached piper-tts synthesis.
    from piper import PiperVoice
except ImportError:
    PiperVoice = None


def contains_any(text, phrases):
    t = text.lower()
    return any(p in t for p in phrases)


class ReceptionistActionServer(Node):

    def __init__(self):
        super().__init__("receptionist_action_server")

        # Section: Receptionist delegates all microphone and Whisper work to voice_stt.
        self.declare_parameter("stt_action_name", "/voice/stt/transcribe_action")
        self.declare_parameter("post_tts_guard_ms", 400)
        self._stt_group = ReentrantCallbackGroup()
        self._stt_client = ActionClient(
            self,
            TranscribeSpeech,
            self.get_parameter("stt_action_name").value,
            callback_group=self._stt_group,
        )
        self.post_tts_guard_sec = max(0.0, float(self.get_parameter("post_tts_guard_ms").value) / 1000.0)
        self._piper_voice = None
        self._piper_model_path = None

        # -------------------------------------------------------
        # Temporary guest memory
        # Stores the latest collected profile while this node process
        # is alive so a later summary-only action can repeat it.
        # Precaution: this memory is intentionally volatile and will
        # be cleared when receptionist_node.py is restarted.
        # -------------------------------------------------------
        self._last_guest_profile = {
            "guest_name": "",
            "guest_age": 0,
            "guest_gender": "",
            "favorite_drink": "",
            "raw_name": "",
            "raw_age": "",
            "raw_gender": "",
            "raw_drink": "",
        }

        self._action_server = ActionServer(
            self,
            Receptionist,
            "receptionist",
            execute_callback=self.execute_cb,
        )

        self.get_logger().info("[receptionist] Action server ready; STT is delegated to voice_stt.")

        # -------------------------------------------------------
        # Dataset choices
        # Used to validate and normalize detected guest answers.
        # -------------------------------------------------------
        self.name_choices = [
            "adam", "axel", "chris", "hunter", "jack",
            "paris", "robin", "olivia", "william", "max"
        ]

        self.gender_choices = [
            "male", "female"
        ]

        self.drink_choices = [
            "orange juice", "coke", "soda", "coffee", "cocoa",
            "lemonade", "coconut milk", "green tea", "black tea"
        ]

        # -------------------------------------------------------
        # Alias mapping
        # If Whisper hears similar words, convert to dataset answer.
        # -------------------------------------------------------
        self.name_aliases = {
            "atom": "adam",
            "adan": "adam",
            "edam": "adam",

            "cris": "chris",
            "christ": "chris",
            "christopher": "chris",

            "jake": "jack",
            "jac": "jack",

            "will": "william",
            "williams": "william",

            "maxwell": "max",
            "maximum": "max",

            "olive": "olivia",
            "oliver": "olivia"
        }

        self.drink_aliases = {
            "cola": "coke",
            "coca cola": "coke",
            "coca-cola": "coke",
            "pepsi": "coke",

            "orange": "orange juice",
            "orange drink": "orange juice",
            "orange juices": "orange juice",
            "oj": "orange juice",

            "tea": "green tea",
            "green tea": "green tea",
            "black tea": "black tea",

            "milk": "coconut milk",
            "coconut": "coconut milk",

            "chocolate": "cocoa",
            "hot chocolate": "cocoa",

            "lemon": "lemonade",
            "lemon juice": "lemonade"
        }

        self.male_words = [
            "male", "man", "boy", "guy", "gentleman"
        ]

        self.female_words = [
            "female", "woman", "girl", "lady"
        ]

    # -------------------------------------------------------
    # TTS
    # -------------------------------------------------------
    def speak(self, text: str, model_path: str) -> bool:
        """
        Generate speech using Piper and play it using aplay.
        This version saves to /tmp/receptionist_tts.wav first.
        """

        if not model_path:
            self.get_logger().error("No Piper model path received")
            return False

        self.get_logger().info(f"Robot says: {text}")
        self.get_logger().info(f"Piper model path: {model_path}")

        try:
            if PiperVoice is None:
                raise RuntimeError("piper-tts is not installed")
            if self._piper_voice is None or self._piper_model_path != model_path:
                self._piper_voice = PiperVoice.load(model_path)
                self._piper_model_path = model_path
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temporary:
                output_wav = temporary.name
            try:
                with wave.open(output_wav, "wb") as wav_file:
                    self._piper_voice.synthesize_wav(text, wav_file)
                playback = subprocess.run(["aplay", "-q", output_wav], capture_output=True, text=True, timeout=60)
                if playback.returncode != 0:
                    raise RuntimeError(f"aplay failed: {playback.stderr[:200]}")
            finally:
                if os.path.exists(output_wav):
                    os.unlink(output_wav)

            self.get_logger().info("Audio played successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"TTS failed: {e}")
            return False

    # -------------------------------------------------------
    # Repeat answer
    # -------------------------------------------------------
    def repeat_answer(self, answer_type: str, answer, piper_model: str):
        """
        Repeat the matched/selected answer to the guest.
        This function never says unknown/don't know.
        """

        answer = str(answer).strip()

        self.speak(
            f"I heard your {answer_type} is {answer}. Thank you.",
            piper_model
        )

    # -------------------------------------------------------
    # Shared STT action
    # -------------------------------------------------------
    def transcribe_via_voice_stt(self, mic: str, sec: float, model: str, lang: str) -> str:
        """Special function: Reuse voice_stt so Receptionist never owns a Whisper model."""
        # Section: Action callbacks run on a reentrant group while this receptionist workflow waits.
        if not self._stt_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[receptionist] Shared STT action is unavailable.")
            return ""
        goal = TranscribeSpeech.Goal()
        goal.record_sec = float(sec)
        goal.mic_device = mic
        goal.whisper_model = model
        goal.whisper_language = lang
        goal.wav_path = ""
        completed = threading.Event()
        response = {}
        self._stt_client.send_goal_async(goal).add_done_callback(
            lambda future: (response.update(handle=future.result()), completed.set())
        )
        if not completed.wait(timeout=10.0) or not response.get("handle") or not response["handle"].accepted:
            return ""
        completed.clear()
        response["handle"].get_result_async().add_done_callback(
            lambda future: (response.update(result=future.result()), completed.set())
        )
        if not completed.wait(timeout=max(20.0, float(sec) + 30.0)):
            response["handle"].cancel_goal_async()
            return ""
        wrapped = response.get("result")
        return wrapped.result.transcript.strip() if wrapped and wrapped.result.success else ""

    # -------------------------------------------------------
    # Ask and transcribe
    # -------------------------------------------------------
    def ask_and_transcribe(
        self,
        question: str,
        mic: str,
        sec: float,
        piper_model: str,
        whisper_model: str,
        lang: str,
        tag: str
    ) -> str:
        """
        Ask question first.
        After TTS finishes, immediately start recording.
        No extra sleep delay.
        """

        if not self.speak(question, piper_model):
            return ""
        # Section: Receptionist's direct Piper path has no shared completion topic, so guard locally.
        time.sleep(self.post_tts_guard_sec)
        return self.transcribe_via_voice_stt(mic, sec, whisper_model, lang)

    # -------------------------------------------------------
    # Ask with retry and skip
    # -------------------------------------------------------
    def ask_until_valid(
        self,
        question: str,
        answer_type: str,
        parser_func,
        is_valid_func,
        mic: str,
        sec: float,
        piper_model: str,
        whisper_model: str,
        lang: str,
        tag: str,
        max_attempts: int = 5
    ):
        """
        Ask the same question up to max_attempts times.

        If valid answer is found:
            return parsed_answer, raw_transcript, True

        If still invalid after max_attempts:
            skip this question
            return None, last_raw_transcript, False
        """

        last_raw = ""

        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(
                f"Asking {answer_type}, attempt {attempt}/{max_attempts}"
            )

            if attempt == 1:
                ask_text = question
            else:
                ask_text = (
                    f"Sorry, I could not hear your {answer_type} clearly. "
                    f"Please say your {answer_type} again."
                )

            raw_answer = self.ask_and_transcribe(
                ask_text,
                mic,
                sec,
                piper_model,
                whisper_model,
                lang,
                f"{tag}_try_{attempt}"
            )

            parsed_answer = parser_func(raw_answer)

            self.get_logger().info(f"Raw {answer_type}: {raw_answer}")
            self.get_logger().info(f"Parsed {answer_type}: {parsed_answer}")

            last_raw = raw_answer

            if is_valid_func(parsed_answer):
                self.repeat_answer(answer_type, parsed_answer, piper_model)
                return parsed_answer, raw_answer, True

        self.get_logger().warn(
            f"Could not get valid {answer_type} after {max_attempts} attempts."
        )

        self.speak(
            f"I could not hear your {answer_type} clearly. "
            f"I will skip this question.",
            piper_model
        )

        return None, last_raw, False

    # -------------------------------------------------------
    # Text cleaning and matching
    # -------------------------------------------------------
    def clean_text(self, text: str) -> str:
        t = text.lower().strip()

        t = t.replace(".", "")
        t = t.replace(",", "")
        t = t.replace("?", "")
        t = t.replace("!", "")

        t = re.sub(r"\s+", " ", t)

        return t

    def remove_common_words(self, text: str) -> str:
        remove_words = [
            "my", "name", "is", "i", "am", "i'm",
            "favorite", "favourite", "drink", "want",
            "like", "please", "a", "an", "the",
            "would", "to", "have", "old", "years",
            "year", "age", "gender"
        ]

        words = text.split()
        filtered_words = [
            w for w in words
            if w not in remove_words
        ]

        return " ".join(filtered_words).strip()

    def match_from_list(self, text: str, choices: list) -> str:
        """
        Match guest answer to known dataset choices.
        """

        t = self.clean_text(text)

        if not t:
            return ""

        string_choices = [str(c).lower() for c in choices]

        # Direct phrase match
        for original, c_lower in zip(choices, string_choices):
            if c_lower in t:
                return str(original)

        # Clean common sentence words and try again
        filtered_text = self.remove_common_words(t)

        for original, c_lower in zip(choices, string_choices):
            if c_lower in filtered_text:
                return str(original)

        # Fuzzy match
        matches = get_close_matches(
            filtered_text,
            string_choices,
            n=1,
            cutoff=0.55
        )

        if matches:
            matched_index = string_choices.index(matches[0])
            return str(choices[matched_index])

        matches = get_close_matches(
            t,
            string_choices,
            n=1,
            cutoff=0.55
        )

        if matches:
            matched_index = string_choices.index(matches[0])
            return str(choices[matched_index])

        return ""

    # -------------------------------------------------------
    # Validation functions
    # -------------------------------------------------------
    def is_valid_name(self, answer) -> bool:
        if answer is None:
            return False

        answer = str(answer).strip().lower()

        if answer == "":
            return False

        return answer in self.name_choices

    def is_valid_age(self, answer) -> bool:
        if answer is None:
            return False

        try:
            age = int(answer)
        except Exception:
            return False

        return age > 0

    def is_valid_gender(self, answer) -> bool:
        if answer is None:
            return False

        answer = str(answer).strip().lower()

        return answer in self.gender_choices

    def is_valid_drink(self, answer) -> bool:
        if answer is None:
            return False

        answer = str(answer).strip().lower()

        return answer in self.drink_choices

    # -------------------------------------------------------
    # Yes / no decision
    # -------------------------------------------------------
    def decide(self, text: str):
        yes = [
            "yes", "yeah", "yep", "sure", "ok", "okay",
            "correct", "right"
        ]

        no = [
            "no", "not", "later", "don't", "do not",
            "wrong", "incorrect"
        ]

        if contains_any(text, no):
            return False

        if contains_any(text, yes):
            return True

        return None

    # -------------------------------------------------------
    # Parsing answers
    # -------------------------------------------------------
    def parse_name(self, text: str) -> str:
        t = self.clean_text(text)

        prefixes = [
            "my name is",
            "i am",
            "i'm",
            "call me",
            "you can call me"
        ]

        for p in prefixes:
            if t.startswith(p):
                t = t[len(p):].strip()
                break

        # Alias match first
        for alias, real_name in self.name_aliases.items():
            if alias in t:
                return real_name.title()

        # Dataset match
        matched = self.match_from_list(t, self.name_choices)

        if matched:
            return matched.title()

        # Do not return unknown/don't know.
        # Return empty so retry or skip will handle it.
        return ""

    def parse_age(self, text: str) -> int:
        t = self.clean_text(text).replace("-", " ")

        # Direct number, example: "20" or "I am 30 years old"
        m = re.search(r"\d+", t)

        if m:
            age = int(m.group())
            if age > 0:
                return age

        # Word number support
        ones = {
            "one": 1,
            "two": 2,
            "three": 3,
            "four": 4,
            "five": 5,
            "six": 6,
            "seven": 7,
            "eight": 8,
            "nine": 9,
            "ten": 10,
            "eleven": 11,
            "twelve": 12,
            "thirteen": 13,
            "fourteen": 14,
            "fifteen": 15,
            "sixteen": 16,
            "seventeen": 17,
            "eighteen": 18,
            "nineteen": 19
        }

        tens = {
            "twenty": 20,
            "thirty": 30,
            "forty": 40,
            "fifty": 50,
            "sixty": 60,
            "seventy": 70,
            "eighty": 80,
            "ninety": 90
        }

        words = [
            word for word in t.split()
            if word not in {
                "i", "am", "i'm", "my", "age", "is",
                "old", "years", "year"
            }
        ]

        total = 0
        current = 0
        found_age_word = False

        for word in words:
            if word in ones:
                current += ones[word]
                found_age_word = True
            elif word in tens:
                current += tens[word]
                found_age_word = True
            elif word == "hundred" and current > 0:
                current *= 100
                found_age_word = True
            elif word == "and":
                continue
            elif found_age_word:
                break

        total += current

        if found_age_word and total > 0:
            return total

        # Do not return unknown/don't know.
        return -1

    def parse_gender(self, text: str) -> str:
        t = self.clean_text(text)

        if any(w in t for w in self.female_words):
            return "female"

        if any(w in t for w in self.male_words):
            return "male"

        matched = self.match_from_list(t, self.gender_choices)

        if matched:
            return matched

        # Do not return unknown/don't know.
        return ""

    def parse_drink(self, text: str) -> str:
        t = self.clean_text(text)

        prefixes = [
            "i want",
            "my favourite drink is",
            "my favorite drink is",
            "i like",
            "my drink is",
            "i would like",
            "i would like to have",
            "can i have"
        ]

        for p in prefixes:
            if t.startswith(p):
                t = t[len(p):].strip()
                break

        # Alias match first. Check longer aliases before generic words like "tea".
        for alias, real_drink in sorted(
            self.drink_aliases.items(),
            key=lambda item: len(item[0]),
            reverse=True
        ):
            if alias in t:
                return real_drink

        # Dataset match
        matched = self.match_from_list(t, self.drink_choices)

        if matched:
            return matched

        # Do not return unknown/don't know.
        return ""

    # -------------------------------------------------------
    # Guest summary helpers
    # These helpers keep the final delivery format identical for
    # interview mode and direct summary-only mode.
    # Precaution: pass empty strings or 0 for missing guest fields
    # instead of inventing unknown information.
    # -------------------------------------------------------
    def build_guest_summary(
        self,
        guest_name: str,
        guest_age: int,
        guest_gender: str,
        favorite_drink: str
    ) -> str:
        # Normalize the name so the spoken output does not include extra spaces.
        name_text = str(guest_name).strip()

        # Normalize the gender so clients can compare result text reliably.
        gender_text = str(guest_gender).strip().lower()

        # Normalize the drink so the final summary uses one consistent spelling.
        drink_text = str(favorite_drink).strip().lower()

        # Protect the action server from bad age values passed by future clients.
        try:
            age_value = int(guest_age)
        except Exception:
            age_value = 0

        # Use the guest name as the spoken subject when it is available.
        subject = name_text if name_text else "The guest"

        # Build the spoken summary as short sentences for clearer TTS delivery.
        summary_sentences = []

        # Include the guest name only when the interview or goal provided it.
        if name_text:
            summary_sentences.append(f"The guest's name is {name_text}.")

        # Include the guest age only when the value is positive and believable.
        if age_value > 0:
            summary_sentences.append(f"{subject} is {age_value} years old.")

        # Include gender only when the value is present after normalization.
        if gender_text:
            summary_sentences.append(f"{subject}'s gender is {gender_text}.")

        # Include favorite drink only when the value is present after normalization.
        if drink_text:
            summary_sentences.append(
                f"{subject}'s favorite drink is {drink_text}."
            )

        # Return an explicit unavailable summary instead of speaking blank text.
        if not summary_sentences:
            return (
                "Guest information summary is unavailable because no guest "
                "information was provided."
            )

        # Prefix the summary so the host knows this is the final delivery stage.
        return "Guest information summary. " + " ".join(summary_sentences)

    # -------------------------------------------------------
    # Temporary guest memory helpers
    # These helpers store one guest profile after interview mode
    # and restore it when summary-only mode is triggered later.
    # Precaution: store only normalized strings and positive ages
    # so later summary generation cannot inherit invalid values.
    # -------------------------------------------------------
    def store_last_guest_profile(
        self,
        guest_name: str,
        guest_age: int,
        guest_gender: str,
        favorite_drink: str,
        raw_name: str,
        raw_age: str,
        raw_gender: str,
        raw_drink: str
    ):
        # Normalize the latest guest name before saving it in node memory.
        stored_name = str(guest_name).strip()

        # Normalize the latest guest age and reject non-positive ages.
        try:
            stored_age = max(0, int(guest_age))
        except Exception:
            stored_age = 0

        # Normalize the latest guest gender for stable summary output.
        stored_gender = str(guest_gender).strip().lower()

        # Normalize the latest favorite drink for stable summary output.
        stored_drink = str(favorite_drink).strip().lower()

        # Save the normalized guest profile for future summary-only goals.
        self._last_guest_profile = {
            "guest_name": stored_name,
            "guest_age": stored_age,
            "guest_gender": stored_gender,
            "favorite_drink": stored_drink,
            "raw_name": str(raw_name),
            "raw_age": str(raw_age),
            "raw_gender": str(raw_gender),
            "raw_drink": str(raw_drink),
        }

        # Log the stored profile without raw transcripts to keep runtime logs concise.
        self.get_logger().info(
            "Stored latest guest profile for summary-only replay: "
            f"name={stored_name}, age={stored_age}, "
            f"gender={stored_gender}, favorite_drink={stored_drink}"
        )

    def has_last_guest_profile(self) -> bool:
        # Check whether any final-summary field exists in temporary memory.
        return any([
            bool(self._last_guest_profile["guest_name"]),
            int(self._last_guest_profile["guest_age"]) > 0,
            bool(self._last_guest_profile["guest_gender"]),
            bool(self._last_guest_profile["favorite_drink"]),
        ])

    def profile_from_goal_or_memory(self, goal):
        # Normalize the action goal name so explicit goal data overrides memory.
        goal_name = str(goal.guest_name).strip().title()

        # Normalize the action goal age so explicit positive age overrides memory.
        goal_age = int(goal.guest_age) if int(goal.guest_age) > 0 else 0

        # Normalize the action goal gender using the same parser as interview mode.
        goal_gender = (
            self.parse_gender(goal.guest_gender)
            or str(goal.guest_gender).strip().lower()
        )

        # Normalize the action goal favorite drink using the same parser as interview mode.
        goal_drink = (
            self.parse_drink(goal.favorite_drink)
            or str(goal.favorite_drink).strip().lower()
        )

        # Fill missing name from memory when the summary-only goal leaves it empty.
        guest_name = goal_name or self._last_guest_profile["guest_name"]

        # Fill missing age from memory when the summary-only goal leaves it as zero.
        guest_age = goal_age or int(self._last_guest_profile["guest_age"])

        # Fill missing gender from memory when the summary-only goal leaves it empty.
        guest_gender = goal_gender or self._last_guest_profile["guest_gender"]

        # Fill missing drink from memory when the summary-only goal leaves it empty.
        favorite_drink = goal_drink or self._last_guest_profile["favorite_drink"]

        # Return the merged profile and the raw transcript fields held in memory.
        return {
            "guest_name": guest_name,
            "guest_age": guest_age,
            "guest_gender": guest_gender,
            "favorite_drink": favorite_drink,
            "raw_name": self._last_guest_profile["raw_name"],
            "raw_age": self._last_guest_profile["raw_age"],
            "raw_gender": self._last_guest_profile["raw_gender"],
            "raw_drink": self._last_guest_profile["raw_drink"],
        }

    def profile_from_memory_only(self):
        # Return only the latest stored guest name without reading goal fields.
        guest_name = self._last_guest_profile["guest_name"]

        # Return only the latest stored guest age without reading goal fields.
        guest_age = int(self._last_guest_profile["guest_age"])

        # Return only the latest stored guest gender without reading goal fields.
        guest_gender = self._last_guest_profile["guest_gender"]

        # Return only the latest stored favorite drink without reading goal fields.
        favorite_drink = self._last_guest_profile["favorite_drink"]

        # Return stored profile data and raw transcripts for result/debug output.
        return {
            "guest_name": guest_name,
            "guest_age": guest_age,
            "guest_gender": guest_gender,
            "favorite_drink": favorite_drink,
            "raw_name": self._last_guest_profile["raw_name"],
            "raw_age": self._last_guest_profile["raw_age"],
            "raw_gender": self._last_guest_profile["raw_gender"],
            "raw_drink": self._last_guest_profile["raw_drink"],
        }

    def fill_receptionist_result(
        self,
        result,
        guest_name: str,
        guest_age: int,
        guest_gender: str,
        favorite_drink: str,
        raw_name: str,
        raw_age: str,
        raw_gender: str,
        raw_drink: str,
        final_summary: str,
        success: bool,
        error: str
    ):
        # Store the normalized guest name for downstream state machines.
        result.guest_name = str(guest_name).strip()

        # Store age as 0 when it is unavailable or invalid.
        try:
            result.guest_age = max(0, int(guest_age))
        except Exception:
            result.guest_age = 0

        # Store the normalized gender string.
        result.guest_gender = str(guest_gender).strip().lower()

        # Store the normalized favorite drink string.
        result.favorite_drink = str(favorite_drink).strip().lower()

        # Preserve raw name transcript for debugging and replay.
        result.raw_name = str(raw_name)

        # Preserve raw age transcript for debugging and replay.
        result.raw_age = str(raw_age)

        # Preserve raw gender transcript for debugging and replay.
        result.raw_gender = str(raw_gender)

        # Preserve raw drink transcript for debugging and replay.
        result.raw_drink = str(raw_drink)

        # Mark arrival compatibility true after the action accepts the workflow.
        result.arrived = True

        # Mark handover compatibility true only when final delivery succeeded.
        result.handover_ok = bool(success)

        # Mirror the name transcript for old clients during migration.
        result.transcript_arrived = str(raw_name)

        # Mirror the drink transcript for old clients during migration.
        result.transcript_handover = str(raw_drink)

        # Return the exact final summary that was sent to TTS.
        result.final_summary = str(final_summary)

        # Return the action-level completion status.
        result.success = bool(success)

        # Return the action-level error message when something failed.
        result.error = str(error)

    # -------------------------------------------------------
    # Action callback
    # The Receptionist action replaces the discontinued guest dialog
    # goal with a standard name for RoboCup integration.
    # Precaution: keep feedback stages stable because other nodes may
    # branch on these strings during the receptionist task.
    # -------------------------------------------------------
    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = Receptionist.Feedback()
        result = Receptionist.Result()

        mic = goal.mic_device
        sec = goal.record_sec
        piper_model = goal.piper_model_path
        whisper_model = goal.whisper_model
        lang = goal.whisper_language
        summary_only = bool(goal.summary_only)
        replay_last_profile = bool(goal.replay_last_profile)

        self.get_logger().info("Received Receptionist goal")
        self.get_logger().info(f"Summary only: {summary_only}")
        self.get_logger().info(f"Replay last profile: {replay_last_profile}")
        self.get_logger().info(f"Mic: {mic}")
        self.get_logger().info(f"Record seconds: {sec}")
        self.get_logger().info(f"Piper model: {piper_model}")
        self.get_logger().info(f"Whisper model: {whisper_model}")
        self.get_logger().info(f"Whisper language: {lang}")

        # -------------------------------------------------------
        # Summary-only goal path
        # This path lets an external state machine trigger the robot
        # to introduce known guest information without asking again.
        # Precaution: this mode first uses structured goal fields,
        # then falls back to the latest stored interview profile unless
        # replay_last_profile is true, which ignores all goal guest fields.
        # -------------------------------------------------------
        if summary_only:
            # Select memory-only replay when the goal explicitly forbids new guest parsing.
            if replay_last_profile:
                # Use only the latest temporarily stored guest profile.
                replay_profile = self.profile_from_memory_only()

                # Log the memory-only mode so operators know goal guest fields were ignored.
                self.get_logger().info(
                    "Replay-last-profile mode enabled; ignoring guest fields in action goal."
                )

            else:
                # Merge explicit goal fields with the latest temporarily stored guest profile.
                replay_profile = self.profile_from_goal_or_memory(goal)

            # Use the merged guest name for final-summary generation.
            guest_name = replay_profile["guest_name"]

            # Use the merged guest age for final-summary generation.
            guest_age = replay_profile["guest_age"]

            # Use the merged guest gender for final-summary generation.
            guest_gender = replay_profile["guest_gender"]

            # Use the merged favorite drink for final-summary generation.
            favorite_drink = replay_profile["favorite_drink"]

            # Warn when no explicit or stored profile data exists to replay.
            if replay_last_profile and not self.has_last_guest_profile():
                self.get_logger().warn(
                    "Replay-last-profile goal received, but no stored guest "
                    "profile is available in node memory."
                )

            elif not self.has_last_guest_profile() and not any([
                bool(guest_name),
                int(guest_age) > 0,
                bool(guest_gender),
                bool(favorite_drink),
            ]):
                self.get_logger().warn(
                    "Summary-only goal received, but no stored guest profile "
                    "or explicit guest fields were available."
                )

            # Build the exact final sentence before publishing feedback.
            final_summary = self.build_guest_summary(
                guest_name,
                guest_age,
                guest_gender,
                favorite_drink
            )

            # Publish final-stage feedback for clients waiting on replay delivery.
            feedback.stage = "FINAL_SUMMARY"
            feedback.info = (
                "Introducing stored guest information"
                if replay_last_profile
                else "Introducing stored or provided guest information"
            )
            feedback.final_summary = final_summary
            goal_handle.publish_feedback(feedback)

            # Speak the summary and use the TTS result as the action status.
            summary_spoken = self.speak(final_summary, piper_model)

            # Fill a complete result even when TTS fails.
            self.fill_receptionist_result(
                result=result,
                guest_name=guest_name,
                guest_age=guest_age,
                guest_gender=guest_gender,
                favorite_drink=favorite_drink,
                raw_name=replay_profile["raw_name"],
                raw_age=replay_profile["raw_age"],
                raw_gender=replay_profile["raw_gender"],
                raw_drink=replay_profile["raw_drink"],
                final_summary=final_summary,
                success=summary_spoken,
                error="" if summary_spoken else "Final summary speech failed"
            )

            # Complete the ROS action with succeed only when speech was delivered.
            if summary_spoken:
                goal_handle.succeed()
                self.get_logger().info("Receptionist summary-only delivery succeeded")
            else:
                goal_handle.abort()
                self.get_logger().error("Receptionist summary-only delivery failed")

            return result

        # -------------------------------------------------------
        # 1. Ask name with retry + skip
        # -------------------------------------------------------
        feedback.stage = "ASK_NAME"
        feedback.info = "Asking guest name"
        feedback.final_summary = ""
        goal_handle.publish_feedback(feedback)

        guest_name, raw_name, name_ok = self.ask_until_valid(
            question="Hello, what is your name?",
            answer_type="name",
            parser_func=self.parse_name,
            is_valid_func=self.is_valid_name,
            mic=mic,
            sec=sec,
            piper_model=piper_model,
            whisper_model=whisper_model,
            lang=lang,
            tag="name",
            max_attempts=5
        )

        # -------------------------------------------------------
        # 2. Ask age with retry + skip
        # -------------------------------------------------------
        feedback.stage = "ASK_AGE"
        feedback.info = "Asking guest age"
        feedback.final_summary = ""
        goal_handle.publish_feedback(feedback)

        guest_age, raw_age, age_ok = self.ask_until_valid(
            question="How old are you?",
            answer_type="age",
            parser_func=self.parse_age,
            is_valid_func=self.is_valid_age,
            mic=mic,
            sec=sec,
            piper_model=piper_model,
            whisper_model=whisper_model,
            lang=lang,
            tag="age",
            max_attempts=5
        )

        # -------------------------------------------------------
        # 3. Ask gender with retry + skip
        # -------------------------------------------------------
        feedback.stage = "ASK_GENDER"
        feedback.info = "Asking guest gender"
        feedback.final_summary = ""
        goal_handle.publish_feedback(feedback)

        guest_gender, raw_gender, gender_ok = self.ask_until_valid(
            question="What is your gender?",
            answer_type="gender",
            parser_func=self.parse_gender,
            is_valid_func=self.is_valid_gender,
            mic=mic,
            sec=sec,
            piper_model=piper_model,
            whisper_model=whisper_model,
            lang=lang,
            tag="gender",
            max_attempts=5
        )

        # -------------------------------------------------------
        # 4. Ask favorite drink with retry + skip
        # -------------------------------------------------------
        feedback.stage = "ASK_DRINK"
        feedback.info = "Asking guest favorite drink"
        feedback.final_summary = ""
        goal_handle.publish_feedback(feedback)

        favorite_drink, raw_drink, drink_ok = self.ask_until_valid(
            question="What is your favorite drink?",
            answer_type="favorite drink",
            parser_func=self.parse_drink,
            is_valid_func=self.is_valid_drink,
            mic=mic,
            sec=sec,
            piper_model=piper_model,
            whisper_model=whisper_model,
            lang=lang,
            tag="drink",
            max_attempts=5
        )

        # -------------------------------------------------------
        # Final summary delivery
        # Only deliver fields that were actually understood.
        # Precaution: the final summary is published as feedback and
        # copied into the result for post-run verification.
        # -------------------------------------------------------
        # Normalize the final guest name before saving or speaking it.
        final_guest_name = str(guest_name) if name_ok else ""

        # Normalize the final guest age before saving or speaking it.
        final_guest_age = int(guest_age) if age_ok else 0

        # Normalize the final guest gender before saving or speaking it.
        final_guest_gender = str(guest_gender) if gender_ok else ""

        # Normalize the final favorite drink before saving or speaking it.
        final_favorite_drink = str(favorite_drink) if drink_ok else ""

        # Store the latest collected profile so a later summary-only goal can replay it.
        self.store_last_guest_profile(
            guest_name=final_guest_name,
            guest_age=final_guest_age,
            guest_gender=final_guest_gender,
            favorite_drink=final_favorite_drink,
            raw_name=raw_name,
            raw_age=raw_age,
            raw_gender=raw_gender,
            raw_drink=raw_drink
        )

        # Build the final summary from the normalized stored profile values.
        final_summary = self.build_guest_summary(
            final_guest_name,
            final_guest_age,
            final_guest_gender,
            final_favorite_drink
        )

        feedback.stage = "FINAL_SUMMARY"
        feedback.info = "Introducing collected guest information"
        feedback.final_summary = final_summary
        goal_handle.publish_feedback(feedback)

        summary_spoken = self.speak(final_summary, piper_model)

        # -------------------------------------------------------
        # Fill result
        # -------------------------------------------------------
        self.fill_receptionist_result(
            result=result,
            guest_name=final_guest_name,
            guest_age=final_guest_age,
            guest_gender=final_guest_gender,
            favorite_drink=final_favorite_drink,
            raw_name=raw_name,
            raw_age=raw_age,
            raw_gender=raw_gender,
            raw_drink=raw_drink,
            final_summary=final_summary,
            success=summary_spoken,
            error="" if summary_spoken else "Final summary speech failed"
        )

        if summary_spoken:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info("Receptionist workflow finished")
        self.get_logger().info(f"Final guest name: {guest_name}")
        self.get_logger().info(f"Final guest age: {guest_age}")
        self.get_logger().info(f"Final guest gender: {guest_gender}")
        self.get_logger().info(f"Final favorite drink: {favorite_drink}")
        self.get_logger().info(f"Final summary: {final_summary}")

        if not name_ok:
            self.get_logger().warn("Name question skipped")
        if not age_ok:
            self.get_logger().warn("Age question skipped")
        if not gender_ok:
            self.get_logger().warn("Gender question skipped")
        if not drink_ok:
            self.get_logger().warn("Drink question skipped")

        return result


def main():
    rclpy.init()

    node = ReceptionistActionServer()

    try:
        # Section: Allow shared-STT action client callbacks while Receptionist blocks in its action workflow.
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
        executor.remove_node(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# DEFAULT SETTINGS FOR TRIGGER RECORDING. 
# ros2 action send_goal --feedback /receptionist voice_interfaces/action/Receptionist "{
#   summary_only: false,
#   replay_last_profile: false,
#   record_sec: 4.0,
#   mic_device: 'default',
#   piper_model_path: '~/piper_models/en_US-lessac-high.onnx',
#   whisper_model: 'turbo',
#   whisper_language: 'en',
#   guest_name: '',
#   guest_age: 0,
#   guest_gender: '',
#   favorite_drink: ''
# }"


# set the summary_only to true to skip the recording and just repeat what is stored in the summary.
# replay_last_profile = true means the robot will only replay the last profile (the last person it talk to)
# replay_last_profile = false means the robot will ask for the guest information (the robot will record, listen and repeat)
