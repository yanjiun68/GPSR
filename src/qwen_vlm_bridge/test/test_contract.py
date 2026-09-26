"""Contract tests for Qwen debug input and semantic plan validation."""

import json

import pytest

from qwen_vlm_bridge.contract import ContractError, parse_input, parse_model_json, validate_plan


def test_plain_debug_text_creates_safe_request() -> None:
    """Special function: Verify convenient topic text defaults to dry-run and no submission."""
    request = parse_input(
        "Go to the kitchen",
        "debug",
        allow_envelope=True,
        max_instruction_chars=100,
    )
    assert request.instruction == "Go to the kitchen"
    assert request.source == "debug"
    assert request.dry_run is True
    assert request.submit_requested is False
    assert request.request_id.startswith("debug_")


def test_debug_json_envelope_preserves_correlation() -> None:
    """Special function: Verify structured shell tests retain request metadata."""
    request = parse_input(
        json.dumps(
            {
                "request_id": "debug_001",
                "instruction": "Find a coke",
                "dry_run": True,
                "submit": True,
            }
        ),
        "debug",
        allow_envelope=True,
        max_instruction_chars=100,
    )
    assert request.request_id == "debug_001"
    assert request.instruction == "Find a coke"
    assert request.submit_requested is True


def test_stt_json_like_text_remains_literal_transcript() -> None:
    """Special function: Ensure only the debug interface interprets JSON envelopes."""
    request = parse_input(
        '{"instruction":"spoken literal"}',
        "stt",
        allow_envelope=False,
        max_instruction_chars=100,
    )
    assert request.instruction == '{"instruction":"spoken literal"}'


def test_invalid_debug_envelope_is_rejected() -> None:
    """Special function: Reject malformed structured input before model inference."""
    with pytest.raises(ContractError, match="invalid JSON debug envelope"):
        parse_input("{bad", "debug", allow_envelope=True, max_instruction_chars=100)


def test_valid_executable_plan_is_accepted() -> None:
    """Special function: Accept a prompt-compliant skill and object parameter map."""
    plan = {
        "requires_clarification": False,
        "actions": [{"skill": "MOVE_TO", "params": {"location": "kitchen"}}],
    }
    assert validate_plan(plan) is plan


def test_fixed_information_skill_is_prompt_allowed() -> None:
    """Special function: Admit the prompt's direct fixed-information skill."""
    plan = {
        "requires_clarification": False,
        "actions": [
            {
                "skill": "FIXED_INFORMATION",
                "params": {"question": "what is today's date"},
            }
        ],
    }
    # Section: Prompt and bridge allow-lists must evolve together.
    assert validate_plan(plan) is plan


def test_unknown_skill_is_rejected() -> None:
    """Special function: Prevent model-invented skills from leaving the Qwen bridge."""
    plan = {
        "requires_clarification": False,
        "actions": [{"skill": "TELEPORT", "params": {}}],
    }
    with pytest.raises(ContractError, match="not allowed"):
        validate_plan(plan)


def test_clarification_cannot_contain_actions() -> None:
    """Special function: Ensure clarification output can never carry executable work."""
    plan = {
        "requires_clarification": True,
        "missing": ["location"],
        "actions": [{"skill": "MOVE_TO", "params": {"location": "kitchen"}}],
    }
    with pytest.raises(ContractError, match="must not contain actions"):
        validate_plan(plan)


def test_markdown_model_output_is_rejected() -> None:
    """Special function: Enforce strict JSON-only responses for downstream machines."""
    with pytest.raises(ContractError, match="Markdown"):
        parse_model_json('```json\n{"requires_clarification": true}\n```')
