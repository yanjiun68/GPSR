# Face models

Place these OpenCV Zoo-compatible ONNX files in this directory before building:

- `face_detection_yunet_2023mar.onnx`
- `face_recognition_sface_2021dec.onnx`

The runtime never downloads models. An explicit ROS parameter path takes priority,
followed by this installed package directory, then `face_model_dir`.
