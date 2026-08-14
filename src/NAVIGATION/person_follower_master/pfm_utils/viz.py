"""RViz MarkerArray builders for tracks and master."""

from __future__ import annotations

from typing import Iterable, Optional

from builtin_interfaces.msg import Time as RosTime
from visualization_msgs.msg import Marker, MarkerArray


def _base_marker(frame_id: str, stamp: RosTime, ns: str, marker_id: int,
                 marker_type: int) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = int(marker_id)
    m.type = marker_type
    m.action = Marker.ADD
    return m


def build_track_markers(
    tracks: Iterable,
    master_id: int,
    frame_id: str,
    stamp: RosTime,
) -> MarkerArray:
    arr = MarkerArray()
    # Clear previous markers deterministically.
    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    for trk in tracks:
        if getattr(trk, "status", "") != "CONFIRMED":
            continue
        is_master = (int(trk.id) == int(master_id))
        cyl = _base_marker(frame_id, stamp, "tracks", int(trk.id) * 2,
                           Marker.CYLINDER)
        cyl.scale.x = 0.40
        cyl.scale.y = 0.40
        cyl.scale.z = 1.60
        cyl.pose.position.x = float(trk.x[0])
        cyl.pose.position.y = float(trk.x[1])
        cyl.pose.position.z = 0.80
        cyl.pose.orientation.w = 1.0
        if is_master:
            cyl.color.r = 1.0
            cyl.color.g = 0.1
            cyl.color.b = 0.1
            cyl.color.a = 0.85
        else:
            cyl.color.r = 0.1
            cyl.color.g = 0.8
            cyl.color.b = 0.2
            cyl.color.a = 0.55
        arr.markers.append(cyl)

        label = _base_marker(frame_id, stamp, "track_labels", int(trk.id) * 2 + 1,
                             Marker.TEXT_VIEW_FACING)
        label.scale.z = 0.30
        label.pose.position.x = float(trk.x[0])
        label.pose.position.y = float(trk.x[1])
        label.pose.position.z = 1.85
        label.pose.orientation.w = 1.0
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        master_tag = " [MASTER]" if is_master else ""
        label.text = f"#{trk.id}{master_tag}"
        arr.markers.append(label)

    return arr
