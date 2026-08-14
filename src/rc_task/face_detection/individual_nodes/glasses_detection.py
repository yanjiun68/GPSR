import argparse
import os
from pathlib import Path

import cv2

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect glasses from a webcam using a YOLO .pt model."
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Path to the trained YOLO model. Defaults to the local InsightFace model assets.",
    )
    parser.add_argument(
        "--source",
        default="0",
        help="Webcam index like 0, or a video file path.",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.35,
        help="Confidence threshold for detections.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Inference image size for YOLO.",
    )
    return parser.parse_args()


def resolve_yolo_model_path(cli_model_path=None):
    base_dir = Path(__file__).resolve().parent
    package_dir = base_dir.parent
    candidate_paths = []
    seen = set()

    def add_candidate(path_like):
        if not path_like:
            return
        candidate = Path(path_like).expanduser()
        candidate_key = str(candidate)
        if candidate_key in seen:
            return
        seen.add(candidate_key)
        candidate_paths.append(candidate)

    add_candidate(cli_model_path)
    add_candidate(os.environ.get("YOLO_MODEL_PATH"))

    search_roots = [
        base_dir,
        package_dir,
        Path.cwd(),
    ]
    for root in search_roots:
        add_candidate(root / "best.pt")
        add_candidate(root / "glasses.pt")
        add_candidate(root / "detection_pt" / "glasses.pt")

    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("face_detection"))
        add_candidate(share_dir / "models" / "glasses.pt")
        add_candidate(share_dir / "detection_pt" / "glasses.pt")
    except Exception:
        pass

    for candidate in candidate_paths:
        if candidate.is_file():
            return str(candidate)

    return [str(candidate) for candidate in candidate_paths]


def open_video_source(source_value):
    source = int(source_value) if str(source_value).isdigit() else source_value
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video source: {source_value}")
    return cap


def get_class_name(model, class_id):
    names = getattr(model, "names", {})
    if isinstance(names, dict):
        return names.get(class_id, f"class_{class_id}")
    if isinstance(names, list) and 0 <= class_id < len(names):
        return names[class_id]
    return f"class_{class_id}"


def draw_detections(frame, results, model):
    detections_found = 0

    if not results or results[0].boxes is None:
        return frame, detections_found

    for box in results[0].boxes:
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        x1, y1, x2, y2 = box.xyxy[0].int().tolist()
        label = f"{get_class_name(model, cls_id)} {conf:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
        cv2.putText(
            frame,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 200, 0),
            2,
        )
        detections_found += 1

    return frame, detections_found


def main():
    args = parse_args()

    if YOLO is None:
        print("Error: ultralytics is not installed.")
        print("Install it with: pip install ultralytics")
        return

    resolved_model = resolve_yolo_model_path(args.model)
    if isinstance(resolved_model, list):
        print("Error: model file not found. Checked these paths:")
        for path in resolved_model:
            print(f"  - {path}")
        print("Place your trained best.pt in InsightFace/ or pass --model /path/to/best.pt")
        return

    print(f"[INFO] Loading model: {resolved_model}")
    model = YOLO(resolved_model)

    print(f"[INFO] Opening source: {args.source}")
    try:
        cap = open_video_source(args.source)
    except RuntimeError as exc:
        print(f"Error: {exc}")
        return

    window_name = "Glasses Detection"

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: failed to read frame")
            break

        results = model.predict(
            source=frame,
            conf=args.conf,
            imgsz=args.imgsz,
            verbose=False,
        )
        annotated_frame, detections_found = draw_detections(frame, results, model)

        status = f"Detections: {detections_found}"
        cv2.putText(
            annotated_frame,
            status,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        cv2.imshow(window_name, annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
