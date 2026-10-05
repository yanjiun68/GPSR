"""Unit tests for the model-free Qwen planner input and output contracts."""

# Section: Exercise public contract helpers without importing ROS or model dependencies.
import pytest

# Section: Import the stable public contract functions and their validation exception.
from qwen_vlm_bridge.contract import ContractError, parse_input, parse_model_json, validate_plan


def test_parse_input_accepts_plain_text() -> None:
    """Special function: Verify plain text becomes a safe dry-run planner request."""
    # Section: Parse a normal operator instruction through the non-envelope path.
    request = parse_input(
        "go to the kitchen",
        "cli",
        allow_envelope=False,
        max_instruction_chars=128,
    )

    # Section: Check stable request fields while allowing the generated identifier to vary.
    assert request.source == "cli"
    assert request.instruction == "go to the kitchen"
    assert request.dry_run is True
    assert request.submit_requested is False
    assert request.request_id.startswith("cli_")


def test_validate_plan_accepts_an_allowed_action() -> None:
    """Special function: Verify an executable plan accepts an allow-listed action."""
    # Section: Build the smallest valid executable ActionQueue response.
    plan = {
        "requires_clarification": False,
        "actions": [{"skill": "MOVE_TO", "params": {"location": "kitchen"}}],
    }

    # Section: Validate the plan against the exact skill used in the response.
    assert validate_plan(plan, allowed_skills=("MOVE_TO",)) == plan


def test_parse_model_json_rejects_markdown_fences() -> None:
    """Special function: Verify model output must follow the strict JSON-only contract."""
    # Section: Reject Markdown-wrapped JSON before any action can be published.
    with pytest.raises(ContractError, match="Markdown"):
        parse_model_json("```json\n{}\n```")
