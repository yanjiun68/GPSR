#!/usr/bin/env python3
"""Render GPSR waypoint poses over the occupancy-grid map."""

# Section: Standard-library imports used for command-line parsing and pose math.
import argparse
import math
from pathlib import Path

# Section: Third-party imports used to load YAML and draw the annotated bitmap.
import yaml
from PIL import Image, ImageDraw, ImageFont


# Section: Shared colors keep every waypoint marker visually consistent.
MARKER_COLORS = (
    "#e53935",
    "#1e88e5",
    "#43a047",
    "#8e24aa",
    "#fb8c00",
    "#00acc1",
    "#6d4c41",
)


def load_yaml(path: Path) -> dict:
    """Special function: Load one YAML document and require a mapping root."""
    # Section: Read the YAML using UTF-8 so waypoint names remain portable.
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    # Section: Reject malformed inputs before attempting coordinate conversion.
    if not isinstance(document, dict):
        raise ValueError(f"Expected a YAML mapping in {path}")
    return document


def load_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    """Special function: Load a readable system font with a portable fallback."""
    # Section: Prefer DejaVu Sans because it is commonly installed with Pillow.
    font_name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(font_name, size)
    except OSError:
        return ImageFont.load_default()


def world_to_pixel(
    world_x: float,
    world_y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    image_height: int,
) -> tuple[float, float]:
    """Special function: Convert a ROS map-frame position into image pixels."""
    # Section: ROS maps begin at the lower-left while bitmap rows begin at the top.
    pixel_x = (world_x - origin_x) / resolution
    pixel_y = image_height - ((world_y - origin_y) / resolution)
    return pixel_x, pixel_y


def quaternion_to_yaw(z_value: float, w_value: float) -> float:
    """Special function: Convert a planar ROS quaternion into yaw radians."""
    # Section: GPSR waypoint poses contain only the quaternion z and w rotation terms.
    return 2.0 * math.atan2(z_value, w_value)


def render_map(map_yaml_path: Path, waypoints_path: Path, output_path: Path) -> None:
    """Special function: Draw all named poses over a scaled occupancy-grid map."""
    # Section: Load map metadata, its referenced PGM, and waypoint pose records.
    map_config = load_yaml(map_yaml_path)
    waypoint_config = load_yaml(waypoints_path)
    map_image_path = (map_yaml_path.parent / str(map_config["image"])).resolve()
    map_image = Image.open(map_image_path).convert("RGB")
    waypoints = waypoint_config.get("waypoints", [])

    # Section: Validate the map geometry fields required for exact placement.
    resolution = float(map_config["resolution"])
    origin = map_config["origin"]
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    if not isinstance(waypoints, list):
        raise ValueError("The waypoint YAML field 'waypoints' must be a list")

    # Section: Scale the small occupancy grid for readable labels and pose arrows.
    scale = 4
    title_height = 72
    footer_height = 62
    scaled_map = map_image.resize(
        (map_image.width * scale, map_image.height * scale),
        Image.Resampling.NEAREST,
    )
    canvas = Image.new(
        "RGB",
        (scaled_map.width, title_height + scaled_map.height + footer_height),
        "#f7f8fa",
    )
    canvas.paste(scaled_map, (0, title_height))
    draw = ImageDraw.Draw(canvas)

    # Section: Add a title and compact explanation of the marker conventions.
    title_font = load_font(30, bold=True)
    label_font = load_font(17, bold=True)
    detail_font = load_font(14)
    draw.text((22, 16), "GPSR Map and Named Waypoints", fill="#17202a", font=title_font)
    draw.text(
        (scaled_map.width - 365, 26),
        "Circle = position   Arrow = heading",
        fill="#455a64",
        font=detail_font,
    )

    # Section: Use tailored label offsets to avoid overlap in the central rooms.
    label_offsets = {
        "bedroom": (-225, 28),
        "living_room": (-260, 24),
        "kitchen": (-225, -145),
        "exit": (-20, -68),
        "entrance": (-35, 30),
        "instruction_point": (-300, -84),
        "dining_room": (34, 24),
    }

    # Section: Draw each waypoint using the map origin, resolution, and image height.
    for index, waypoint in enumerate(waypoints):
        pose = waypoint["pose"]
        position = pose["position"]
        orientation = pose["orientation"]
        world_x = float(position["x"])
        world_y = float(position["y"])
        map_x, map_y = world_to_pixel(
            world_x,
            world_y,
            origin_x,
            origin_y,
            resolution,
            map_image.height,
        )
        pixel_x = map_x * scale
        pixel_y = title_height + (map_y * scale)
        color = MARKER_COLORS[index % len(MARKER_COLORS)]
        marker_radius = 13

        # Section: Mark the waypoint center with a high-contrast numbered circle.
        draw.ellipse(
            (
                pixel_x - marker_radius,
                pixel_y - marker_radius,
                pixel_x + marker_radius,
                pixel_y + marker_radius,
            ),
            fill=color,
            outline="white",
            width=3,
        )
        waypoint_id = str(waypoint.get("id", index))
        id_bounds = draw.textbbox((0, 0), waypoint_id, font=detail_font)
        id_width = id_bounds[2] - id_bounds[0]
        id_height = id_bounds[3] - id_bounds[1]
        draw.text(
            (pixel_x - (id_width / 2), pixel_y - (id_height / 2) - 2),
            waypoint_id,
            fill="white",
            font=detail_font,
        )

        # Section: Draw the stored yaw as an arrow extending from the position marker.
        yaw = quaternion_to_yaw(float(orientation["z"]), float(orientation["w"]))
        arrow_length = 42
        arrow_x = pixel_x + (arrow_length * math.cos(yaw))
        arrow_y = pixel_y - (arrow_length * math.sin(yaw))
        draw.line((pixel_x, pixel_y, arrow_x, arrow_y), fill=color, width=6)
        arrow_angle = math.atan2(pixel_y - arrow_y, arrow_x - pixel_x)
        for side in (-1, 1):
            head_angle = arrow_angle + (side * math.radians(150))
            head_x = arrow_x + (14 * math.cos(head_angle))
            head_y = arrow_y - (14 * math.sin(head_angle))
            draw.line((arrow_x, arrow_y, head_x, head_y), fill=color, width=6)

        # Section: Connect each marker to a boxed name and exact world coordinates.
        waypoint_name = str(waypoint.get("name", f"waypoint_{index}"))
        offset_x, offset_y = label_offsets.get(waypoint_name, (30, 24))
        label_x = pixel_x + offset_x
        label_y = pixel_y + offset_y
        name_text = f"{waypoint_id}: {waypoint_name}"
        coordinate_text = f"({world_x:.2f}, {world_y:.2f}) m"
        name_bounds = draw.textbbox((0, 0), name_text, font=label_font)
        coordinate_bounds = draw.textbbox((0, 0), coordinate_text, font=detail_font)
        box_width = max(name_bounds[2], coordinate_bounds[2]) + 18
        box_height = 49
        box = (label_x, label_y, label_x + box_width, label_y + box_height)
        connector_x = label_x if offset_x >= 0 else label_x + box_width
        connector_y = label_y + (box_height / 2)
        draw.line((pixel_x, pixel_y, connector_x, connector_y), fill=color, width=3)
        draw.rounded_rectangle(box, radius=7, fill="#ffffffe8", outline=color, width=3)
        draw.text((label_x + 9, label_y + 4), name_text, fill="#17202a", font=label_font)
        draw.text(
            (label_x + 9, label_y + 27),
            coordinate_text,
            fill="#455a64",
            font=detail_font,
        )

    # Section: Add map metadata so the rendered geometry remains auditable.
    footer_y = title_height + scaled_map.height
    draw.rectangle((0, footer_y, scaled_map.width, canvas.height), fill="#263238")
    footer_text = (
        f"Map: {map_yaml_path.name}   |   Resolution: {resolution:.2f} m/pixel   |   "
        f"Origin: ({origin_x:.2f}, {origin_y:.2f})   |   Waypoints: {len(waypoints)}"
    )
    draw.text((22, footer_y + 21), footer_text, fill="white", font=detail_font)

    # Section: Save a lossless PNG and create its parent directory when necessary.
    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, format="PNG")


def parse_arguments() -> argparse.Namespace:
    """Special function: Parse optional project-relative input and output paths."""
    # Section: Provide defaults matching this repository's mutable runtime data layout.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=Path("maps/gpsr_map.yaml"))
    parser.add_argument(
        "--waypoints",
        type=Path,
        default=Path("waypoints/gpsr_waypoint.yaml"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("maps/gpsr_map_with_waypoints.png"),
    )
    return parser.parse_args()


def main() -> None:
    """Special function: Run the waypoint-map renderer from the command line."""
    # Section: Resolve command-line inputs and create the requested annotated map.
    arguments = parse_arguments()
    render_map(arguments.map, arguments.waypoints, arguments.output)
    print(f"Rendered waypoint map: {arguments.output.resolve()}")


# Section: Execute only when the file is run as a command-line program.
if __name__ == "__main__":
    main()
