"""Persistent Qwen model engine extracted from the original one-shot test_text.py flow."""

from __future__ import annotations

import json
from pathlib import Path
import threading
from typing import Any, Iterable

from qwen_vlm_bridge.contract import parse_model_json, validate_plan


class QwenPlannerEngine:
    """Special function: Load Qwen once and generate validated text-only semantic plans."""

    def __init__(
        self,
        *,
        model_path: str,
        system_prompt_path: str,
        max_new_tokens: int = 256,
        load_in_4bit: bool = True,
        local_files_only: bool = True,
        allowed_skills: Iterable[str] = (),
    ) -> None:
        """Special function: Store model settings without importing the heavy ML runtime."""
        # Section: Expand operator paths while retaining explicit offline model behavior.
        self.model_path = Path(model_path).expanduser().resolve()
        self.system_prompt_path = Path(system_prompt_path).expanduser().resolve()
        self.max_new_tokens = max(1, int(max_new_tokens))
        self.load_in_4bit = bool(load_in_4bit)
        self.local_files_only = bool(local_files_only)
        self.allowed_skills = tuple(allowed_skills)

        # Section: Model objects are created once by load and serialized by the inference lock.
        self.processor: Any = None
        self.model: Any = None
        self.system_prompt = ""
        self._torch: Any = None
        self._inference_lock = threading.Lock()

    def load(self) -> None:
        """Special function: Load the system prompt, processor, and local Qwen weights exactly once."""
        # Section: Validate all local assets before importing or allocating the model runtime.
        if not self.system_prompt_path.is_file():
            raise FileNotFoundError(f"system prompt does not exist: {self.system_prompt_path}")
        if not self.model_path.exists():
            raise FileNotFoundError(f"model path does not exist: {self.model_path}")
        self.system_prompt = self.system_prompt_path.read_text(encoding="utf-8").strip()
        if not self.system_prompt:
            raise ValueError(f"system prompt is empty: {self.system_prompt_path}")

        # Section: Delay optional CUDA dependencies so contract tests run without Transformers installed.
        import torch
        from transformers import (
            AutoProcessor,
            BitsAndBytesConfig,
            Qwen3VLForConditionalGeneration,
        )

        # Section: Match the proven test_text.py NF4 configuration when 4-bit loading is enabled.
        model_kwargs: dict[str, Any] = {
            "device_map": "auto",
            "low_cpu_mem_usage": True,
            "local_files_only": self.local_files_only,
        }
        if self.load_in_4bit:
            model_kwargs["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=torch.float16,
                bnb_4bit_use_double_quant=True,
            )

        # Section: Cache all expensive model objects for every later ROS request.
        self.processor = AutoProcessor.from_pretrained(
            str(self.model_path),
            local_files_only=self.local_files_only,
        )
        self.model = Qwen3VLForConditionalGeneration.from_pretrained(
            str(self.model_path),
            **model_kwargs,
        )
        # Section: Disable training behavior because the persistent engine only generates plans.
        self.model.eval()
        self._torch = torch

    def generate_plan(self, instruction: str) -> tuple[dict[str, Any], str]:
        """Special function: Generate one deterministic JSON plan from an in-memory transcript."""
        # Section: Refuse inference until the persistent model and prompt are fully ready.
        if self.model is None or self.processor is None or self._torch is None:
            raise RuntimeError("QwenPlannerEngine.load() must complete before inference")
        user_text = str(instruction or "").strip()
        if not user_text:
            raise ValueError("instruction is empty")

        # Section: Recreate the original test_text.py conversation using the transcript as user text.
        messages = [
            {
                "role": "system",
                "content": [{"type": "text", "text": self.system_prompt}],
            },
            {
                "role": "user",
                "content": [{"type": "text", "text": user_text}],
            },
        ]

        # Section: Serialize GPU access because concurrent generate calls are unsupported and waste memory.
        with self._inference_lock:
            inputs = self.processor.apply_chat_template(
                messages,
                tokenize=True,
                add_generation_prompt=True,
                return_dict=True,
                return_tensors="pt",
            )
            inputs = inputs.to(self.model.device)
            with self._torch.inference_mode():
                output_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=self.max_new_tokens,
                    do_sample=False,
                )

        # Section: Decode only newly generated tokens and preserve the raw response for diagnostics.
        generated_ids = output_ids[:, inputs["input_ids"].shape[1] :]
        raw_response = self.processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0].strip()

        # Section: Publishable plans must pass strict JSON and semantic-skill validation.
        plan = parse_model_json(raw_response)
        return validate_plan(plan, self.allowed_skills), raw_response

    @staticmethod
    def format_plan(plan: dict[str, Any]) -> str:
        """Special function: Produce stable UTF-8 JSON for ROS topics and CLI output."""
        return json.dumps(plan, ensure_ascii=False, sort_keys=True)
