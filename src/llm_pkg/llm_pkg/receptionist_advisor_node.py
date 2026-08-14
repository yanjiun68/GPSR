#!/usr/bin/env python3

# Import json so the node can parse and validate model output.
import json

# Import os so the node can read the optional Gemini API key.
import os

# Import ROS 2 Python APIs.
import rclpy

# Import the ROS 2 node base class.
from rclpy.node import Node

# Import the typed receptionist advisor service.
from llm_interfaces.srv import GetReceptionistAdvice

# Import the OpenAI-compatible client when available.
try:
    from openai import OpenAI
except Exception:
    OpenAI = None


# Provide a bounded LLM advisor for receptionist social text only.
class ReceptionistAdvisorNode(Node):
    # Construct the service and optional model client.
    def __init__(self):
        # Give the node a stable ROS name.
        super().__init__("receptionist_advisor_node")

        # Declare the model name so launch files can override it.
        self.declare_parameter("model", "gemini-3.1-flash-lite-preview")

        # Declare a conservative timeout placeholder for future HTTP client tuning.
        self.declare_parameter("enable_online_llm", True)

        # Cache model configuration.
        self.model = str(self.get_parameter("model").value)
        self.enable_online_llm = bool(self.get_parameter("enable_online_llm").value)

        # Build the OpenAI-compatible client only when credentials and dependency exist.
        api_key = os.environ.get("GEMINI_API_KEY", "")
        self.client = None
        if self.enable_online_llm and OpenAI is not None and api_key:
            self.client = OpenAI(
                api_key=api_key,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
            )

        # Expose the safe receptionist advice service.
        self.create_service(
            GetReceptionistAdvice,
            "/llm/receptionist_advice",
            self._advice_cb,
        )

        # Log whether the node will call the online model or deterministic fallback.
        self.get_logger().info(
            "Receptionist advisor ready. "
            f"online_llm={'yes' if self.client is not None else 'no'}."
        )

    # Handle one receptionist advice request.
    def _advice_cb(self, request, response):
        # Use deterministic fallback when no online model is configured.
        if self.client is None:
            return self._fallback_response(request, response, "llm_unavailable")

        # Build a compact prompt that only asks for safe advisor fields.
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a safe RoboCup@Home receptionist advisor. "
                    "Return only JSON with speech_text, normalized_name, and normalized_drink. "
                    "Do not output robot commands, motor commands, joint commands, velocities, "
                    "skills, ROS topics, or safety bypass instructions."
                ),
            },
            {
                "role": "user",
                "content": json.dumps(
                    {
                        "request_type": request.request_type,
                        "context": self._context_to_dict(request.context),
                    },
                    ensure_ascii=True,
                ),
            },
        ]

        # Call the configured model and validate the returned JSON object.
        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                stream=False,
                response_format={"type": "json_object"},
                temperature=0.2,
            )
            content = completion.choices[0].message.content or "{}"
            payload = json.loads(content)
        except Exception as exc:
            self.get_logger().warning(f"Receptionist advisor LLM call failed: {exc}")
            return self._fallback_response(request, response, "llm_call_failed")

        # Reject unsafe payloads before copying anything into the response.
        if not self._payload_is_safe(payload):
            return self._fallback_response(request, response, "unsafe_llm_output")

        # Copy validated text fields into the typed response.
        response.success = True
        response.speech_text = str(payload.get("speech_text", "")).strip()
        response.normalized_name = str(payload.get("normalized_name", "")).strip()
        response.normalized_drink = str(payload.get("normalized_drink", "")).strip()
        response.error_code = ""

        # Fall back when the model returned no useful speech text.
        if not response.speech_text:
            return self._fallback_response(request, response, "empty_llm_output")

        # Return the validated response.
        return response

    # Convert the generated ROS context message to plain JSON data.
    def _context_to_dict(self, context):
        # Return a compact dictionary with only semantic task fields.
        return {
            "task_id": context.task_id,
            "current_state": context.current_state,
            "guest_index": int(context.guest_index),
            "target_guest_count": int(context.target_guest_count),
            "profile_id": context.profile_id,
            "guest_name": context.guest_name,
            "favorite_drink": context.favorite_drink,
            "target_profile_ids": list(context.target_profile_ids),
            "last_error": context.last_error,
            "dry_run": bool(context.dry_run),
        }

    # Return false when a payload contains forbidden direct-control content.
    def _payload_is_safe(self, payload):
        # Reject non-object JSON because the service contract requires named fields.
        if not isinstance(payload, dict):
            return False

        # Merge all string fields into one lowercase string for simple safety checks.
        combined_text = " ".join(str(value).lower() for value in payload.values())

        # Define forbidden terms that imply direct control or safety bypass.
        forbidden_terms = (
            "cmd_vel",
            "joint",
            "motor",
            "velocity",
            "pwm",
            "disable_safety",
            "bypass",
            "ros topic",
            "publish",
            "navigate_to_pose",
        )

        # Reject the payload when any forbidden term appears.
        return not any(term in combined_text for term in forbidden_terms)

    # Fill a deterministic response when the LLM is unavailable or unsafe.
    def _fallback_response(self, request, response, error_code):
        # Choose a stable sentence based on the request type.
        speech_text = self._fallback_speech(request)

        # Return success=false so callers know deterministic fallback was used.
        response.success = False
        response.speech_text = speech_text
        response.normalized_name = request.context.guest_name.strip()
        response.normalized_drink = request.context.favorite_drink.strip()
        response.error_code = error_code

        # Return the populated fallback response.
        return response

    # Build deterministic speech for common receptionist states.
    def _fallback_speech(self, request):
        # Normalize the request type for stable matching.
        request_type = request.request_type.strip().lower()

        # Return a greeting fallback.
        if request_type == "greeting":
            return "Hi, welcome, nice to meet you."

        # Return an introduction fallback with available guest details.
        if request_type == "introduction":
            guest_name = request.context.guest_name.strip() or "our guest"
            drink = request.context.favorite_drink.strip() or "their favorite drink"
            return f"Please welcome {guest_name}. {guest_name} likes {drink}."

        # Return a failure fallback.
        if request_type == "failure":
            return "I am sorry, I need to recover before continuing the receptionist task."

        # Return a clarification fallback.
        if request_type == "clarification":
            return "Could you please repeat that?"

        # Return a generic safe fallback.
        return "I will continue the receptionist task."


# Provide the standard ROS 2 executable entry point.
def main(args=None):
    # Initialize ROS 2.
    rclpy.init(args=args)

    # Construct the advisor node.
    node = ReceptionistAdvisorNode()

    # Serve advice requests until shutdown.
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Run the entry point when executed directly.
if __name__ == "__main__":
    main()
