from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'face_detection'
# Section: Human appearance must never auto-load unrelated environment detectors.
HUMAN_ATTRIBUTE_PT_FILES = (
    'glasses.pt',
    'tshirt.pt',
    'facce.pt',
)


def package_share_dir() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME))


def default_glasses_model_path() -> str:
    detection_glasses = detection_assets_dir() / 'glasses.pt'
    if detection_glasses.exists():
        return str(detection_glasses)
    return str(package_share_dir() / 'models' / 'glasses.pt')


def detection_assets_dir() -> Path:
    return package_share_dir() / 'detection_pt'


def default_detection_model_paths() -> list[str]:
    """Special function: Return only models relevant to the selected person's appearance."""
    detection_dir = detection_assets_dir()
    model_paths = [
        str(detection_dir / filename)
        for filename in HUMAN_ATTRIBUTE_PT_FILES
        if (detection_dir / filename).exists()
    ]
    if model_paths:
        return model_paths
    return [default_glasses_model_path()]


def default_guest_db_path() -> str:
    db_dir = Path.home() / '.ros' / PACKAGE_NAME
    db_dir.mkdir(parents=True, exist_ok=True)
    return str(db_dir / 'guest_db.pkl')
