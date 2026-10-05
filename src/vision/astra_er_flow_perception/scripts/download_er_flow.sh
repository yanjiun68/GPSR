#!/usr/bin/env bash
set -euo pipefail

MODEL_DIR="${1:-$HOME/models/UnifoLM-ER-Flow}"

mkdir -p "$(dirname "$MODEL_DIR")"

echo "Downloading Unitree UnifoLM-ER-Flow to:"
echo "  $MODEL_DIR"

if command -v hf >/dev/null 2>&1; then
  hf download unitreerobotics/UnifoLM-ER-Flow --local-dir "$MODEL_DIR"
elif command -v huggingface-cli >/dev/null 2>&1; then
  huggingface-cli download unitreerobotics/UnifoLM-ER-Flow --local-dir "$MODEL_DIR"
else
  echo "Neither 'hf' nor 'huggingface-cli' is available."
  echo "Install the Hugging Face CLI first:"
  echo "  pip install -U huggingface_hub"
  exit 1
fi

echo "Download complete."
