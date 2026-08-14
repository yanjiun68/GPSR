#!/usr/bin/env python3
"""Check the system OpenCV YuNet/SFace capability without starting ROS or a camera."""

import argparse
import json
from pathlib import Path
import sys

import cv2


# Special function: Render stable structured scalar values for shell diagnostics.
def display(value):
    """Return JSON-compatible scalar text."""
    if value is None:
        return 'null'
    if isinstance(value, bool):
        return str(value).lower()
    return json.dumps(str(value))


# Special function: Parse optional model files without loading camera or ROS resources.
def parse_args():
    """Return command-line model paths used for optional load checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--yunet-model', default='', help='Optional YuNet ONNX file to load.')
    parser.add_argument('--sface-model', default='', help='Optional SFace ONNX file to load.')
    return parser.parse_args()


# Special function: Verify model construction only when the operator supplies both files.
def model_load_status(yunet_model, sface_model):
    """Return whether optional model paths exist and construct through OpenCV DNN."""
    if not yunet_model and not sface_model:
        return None, 'model load check skipped; pass both --yunet-model and --sface-model'
    if not yunet_model or not sface_model:
        return False, 'both --yunet-model and --sface-model are required together'
    if not Path(yunet_model).is_file() or not Path(sface_model).is_file():
        return False, 'one or both supplied model files do not exist'
    try:
        cv2.FaceDetectorYN.create(yunet_model, '', (320, 320), 0.8, 0.3, 5000)
        cv2.FaceRecognizerSF.create(sface_model, '')
    except cv2.error as error:
        return False, f'{type(error).__name__}: {error}'
    return True, 'YuNet and SFace models loaded successfully'


# Special function: Print actionable OpenCV capability and optional model status.
def main():
    """Run CPU face-backend environment checks and return a shell-friendly status."""
    args = parse_args()
    # Section: These API checks do not import InsightFace, ONNX Runtime, Torch, or Matplotlib.
    has_yunet = hasattr(cv2, 'FaceDetectorYN')
    has_sface = hasattr(cv2, 'FaceRecognizerSF')
    models_ok, model_message = model_load_status(args.yunet_model, args.sface_model)
    api_ok = has_yunet and has_sface
    status_ok = api_ok and models_ok is not False
    print('python:')
    print(f'  executable: {display(sys.executable)}')
    print(f'  version: {display(sys.version.split()[0])}')
    print('opencv:')
    print(f'  version: {display(cv2.__version__)}')
    print(f'  path: {display(cv2.__file__)}')
    print(f'  FaceDetectorYN: {display(has_yunet)}')
    print(f'  FaceRecognizerSF: {display(has_sface)}')
    print('models:')
    print(f'  load_ok: {display(models_ok)}')
    print(f'  message: {display(model_message)}')
    print('status:')
    print(f'  ok: {display(status_ok)}')
    return 0 if status_ok else 1


# Section: Execute the diagnostic only when invoked as a script.
if __name__ == '__main__':
    raise SystemExit(main())
