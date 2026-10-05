"""Regression checks for ER-Flow prompt construction."""

# Section: Import the backend without loading model weights.
from astra_er_flow_perception.er_flow_backend import ERFlowBackend
import pytest


def test_prompt_preserves_literal_json_braces():
    """Special function: Ensure JSON examples are not parsed as format placeholders."""
    # Section: Reproduce the package prompt's command fields and literal output schema.
    backend = ERFlowBackend(
        model_path="unused",
        prompt_template=(
            "Target: {target}\n"
            "Context: {user_prompt}\n"
            '{"found":true,"bbox":[1,2,3,4]}'
        ),
    )
    prompt = backend.build_prompt("mineral water bottle", "clear bottle")

    # Section: Only the two supported command placeholders may change.
    assert "Target: mineral water bottle" in prompt
    assert "Context: clear bottle" in prompt
    assert '{"found":true,"bbox":[1,2,3,4]}' in prompt


def test_backend_rejects_non_nf4_quantization():
    """Special function: Prevent accidental full-precision model deployment."""
    # Section: Any mode other than NF4 must fail before model loading starts.
    with pytest.raises(ValueError, match="requires quantization_mode='nf4'"):
        ERFlowBackend(
            model_path="unused",
            quantization_mode="none",
        )
