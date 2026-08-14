#!/usr/bin/env python3
"""Master tracker node.

Core responsibility: subscribe to /cam_detections_reid, maintain a
Kalman-filtered multi-target tracker in odom frame with Hungarian
association on motion + appearance, implement the master enrollment
and re-identification state machine, and publish:

 - /tracks          (person_follower_master/msg/Tracks)
 - /master_target   (person_follower_master/msg/MasterTarget, 20 Hz)
 - /master_markers  (visualization_msgs/MarkerArray)

Services:
 - /enroll_master  (std_srvs/Trigger)  force enrollment now
 - /reset_master   (std_srvs/Trigger)  drop master, return to IDLE
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque
from typing import Deque, List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
import tf2_ros

from vision.msg import CamDetections

from person_follower_master.msg import Track as TrackMsg, Tracks, MasterTarget
from pfm_utils.kalman_tracker import KalmanTrack
from pfm_utils.association import (
    build_cost_matrix,
    solve_assignment,
)
from pfm_utils.tf_utils import lookup_transform, apply_transform_2d
from pfm_utils.viz import build_track_markers


STATE_IDLE = "IDLE"
STATE_ENROLLING = "ENROLLING"
STATE_FOLLOWING = "FOLLOWING"
STATE_LOST = "LOST"


class MasterTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("master_tracker_node")

        # ----- Parameters -----
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("detections_topic", "/cam_detections_reid")
        self.declare_parameter("assoc_alpha_motion", 1.0)
        self.declare_parameter("assoc_beta_reid", 2.0)
        self.declare_parameter("gate_mahalanobis", 9.21)
        # gate_reid: association rejects pairs with cos-sim < (1 - gate_reid).
        # 0.50 → require sim > 0.50, which keeps different-identity crossings
        # from hijacking a track via motion-only matching.
        self.declare_parameter("gate_reid", 0.50)
        self.declare_parameter("sigma_p", 0.30)
        self.declare_parameter("sigma_v", 0.50)
        self.declare_parameter("meas_std", 0.08)
        self.declare_parameter("tentative_hits", 3)
        self.declare_parameter("confirmed_lost_limit", 8)
        self.declare_parameter("delete_lost_limit", 80)
        self.declare_parameter("min_gallery_bbox_area", 2500)
        self.declare_parameter("gallery_size", 10)
        self.declare_parameter("master_gallery_size", 30)
        self.declare_parameter("enroll_min_dist", 0.8)
        self.declare_parameter("enroll_max_dist", 2.0)
        self.declare_parameter("enroll_bearing_deg", 15.0)
        self.declare_parameter("enroll_stable_sec", 3.0)
        self.declare_parameter("relock_min_sim", 0.75)
        self.declare_parameter("relock_margin", 0.10)
        self.declare_parameter("keep_min_sim", 0.65)
        self.declare_parameter("lost_hold_sec", 0.5)
        self.declare_parameter("recovery_max_sim_gate", 0.72)
        self.declare_parameter("recovery_sim_floor", 0.62)
        self.declare_parameter("recovery_sim_decay", 0.005)  # sim gate drop per second lost
        self.declare_parameter("recovery_max_dist", 3.0)
        self.declare_parameter("recovery_dist_growth", 0.50)  # dist gate growth per second lost
        self.declare_parameter("lost_timeout_sec", 10.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("high_quality_bbox_area", 4000)
        self.declare_parameter("high_quality_sim", 0.80)
        # Bbox-overlap protection: when two YOLO detections have IoU above
        # this, their OSNet crops mix pixels from both identities and their
        # embeddings are unreliable. We use it to (a) block gallery pushes
        # from polluted detections and (b) require stricter Re-ID thresholds
        # when identity-first assigning the master track.
        self.declare_parameter("ambig_iou_threshold", 0.15)
        # Identity-first thresholds for assigning the master track. The
        # "clear" pair applies when the best-matching detection has no
        # significant bbox overlap with another detection; "ambig" applies
        # when it does.
        self.declare_parameter("master_sim_clear", 0.55)
        self.declare_parameter("master_margin_clear", 0.08)
        self.declare_parameter("master_sim_ambig", 0.70)
        self.declare_parameter("master_margin_ambig", 0.15)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        det_topic = str(self.get_parameter("detections_topic").value)
        self.alpha_motion = float(self.get_parameter("assoc_alpha_motion").value)
        self.beta_reid = float(self.get_parameter("assoc_beta_reid").value)
        self.gate_mahalanobis = float(self.get_parameter("gate_mahalanobis").value)
        self.gate_reid = float(self.get_parameter("gate_reid").value)
        self.sigma_p = float(self.get_parameter("sigma_p").value)
        self.sigma_v = float(self.get_parameter("sigma_v").value)
        self.meas_std = float(self.get_parameter("meas_std").value)
        self.tentative_hits = int(self.get_parameter("tentative_hits").value)
        self.confirmed_lost_limit = int(self.get_parameter("confirmed_lost_limit").value)
        self.delete_lost_limit = int(self.get_parameter("delete_lost_limit").value)
        self.min_gallery_bbox_area = int(self.get_parameter("min_gallery_bbox_area").value)
        self.gallery_size = int(self.get_parameter("gallery_size").value)
        self.master_gallery_size = int(self.get_parameter("master_gallery_size").value)
        self.enroll_min_dist = float(self.get_parameter("enroll_min_dist").value)
        self.enroll_max_dist = float(self.get_parameter("enroll_max_dist").value)
        self.enroll_bearing_rad = math.radians(
            float(self.get_parameter("enroll_bearing_deg").value))
        self.enroll_stable_sec = float(self.get_parameter("enroll_stable_sec").value)
        self.relock_min_sim = float(self.get_parameter("relock_min_sim").value)
        self.relock_margin = float(self.get_parameter("relock_margin").value)
        self.keep_min_sim = float(self.get_parameter("keep_min_sim").value)
        self.lost_hold_sec = float(self.get_parameter("lost_hold_sec").value)
        self.recovery_max_sim_gate = float(self.get_parameter("recovery_max_sim_gate").value)
        self.recovery_sim_floor = float(self.get_parameter("recovery_sim_floor").value)
        self.recovery_sim_decay = float(self.get_parameter("recovery_sim_decay").value)
        self.recovery_max_dist = float(self.get_parameter("recovery_max_dist").value)
        self.recovery_dist_growth = float(self.get_parameter("recovery_dist_growth").value)
        self.lost_timeout_sec = float(self.get_parameter("lost_timeout_sec").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.hq_area = int(self.get_parameter("high_quality_bbox_area").value)
        self.hq_sim = float(self.get_parameter("high_quality_sim").value)
        self.ambig_iou_threshold = float(self.get_parameter("ambig_iou_threshold").value)
        self.master_sim_clear = float(self.get_parameter("master_sim_clear").value)
        self.master_margin_clear = float(self.get_parameter("master_margin_clear").value)
        self.master_sim_ambig = float(self.get_parameter("master_sim_ambig").value)
        self.master_margin_ambig = float(self.get_parameter("master_margin_ambig").value)

        # ----- State -----
        self.tracks: List[KalmanTrack] = []
        self._next_track_id = 1
        self.master_state = STATE_IDLE
        self.master_track_id: int = -1
        self.master_gallery: Deque[np.ndarray] = deque(maxlen=self.master_gallery_size)
        self._master_gallery_anchor: Optional[np.ndarray] = None  # permanent index-0
        self.last_master_pose: Optional[tuple] = None  # (x, y, stamp_sec)
        self._enroll_candidate_id: int = -1
        self._enroll_start_sec: float = 0.0
        self._last_pub_master: Optional[MasterTarget] = None
        self._last_alarm_log_sec = 0.0
        self._last_idfirst_log_sec = 0.0

        # ----- TF -----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----- I/O -----
        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE
        qos_reliable.history = HistoryPolicy.KEEP_LAST

        qos_master = QoSProfile(depth=1)
        qos_master.reliability = ReliabilityPolicy.RELIABLE
        qos_master.history = HistoryPolicy.KEEP_LAST
        qos_master.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub_det = self.create_subscription(
            CamDetections, det_topic, self._on_detections, qos_reliable)
        self.pub_tracks = self.create_publisher(Tracks, "/tracks", qos_reliable)
        self.pub_master = self.create_publisher(MasterTarget, "/master_target", qos_master)
        self.pub_markers = self.create_publisher(MarkerArray, "/master_markers", qos_reliable)

        self.srv_enroll = self.create_service(Trigger, "/enroll_master",
                                              self._on_enroll_srv)
        self.srv_reset = self.create_service(Trigger, "/reset_master",
                                             self._on_reset_srv)

        timer_period = 1.0 / max(1.0, publish_rate)
        self.create_timer(timer_period, self._on_publish_timer)

        self.get_logger().info(
            f"master_tracker ready. State={self.master_state}. "
            f"enrollment window: dist=[{self.enroll_min_dist}, "
            f"{self.enroll_max_dist}] m, bearing=±{math.degrees(self.enroll_bearing_rad):.0f}°, "
            f"hold={self.enroll_stable_sec}s."
        )

    # ========================================================================
    # Helpers
    # ========================================================================
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _spawn_track(self, xy: np.ndarray, stamp_sec: float,
                     embed: Optional[np.ndarray], bbox_area: int,
                     ambig: bool = False) -> KalmanTrack:
        trk = KalmanTrack(
            track_id=self._next_track_id,
            init_xy=xy,
            stamp=stamp_sec,
            sigma_p=self.sigma_p,
            sigma_v=self.sigma_v,
            meas_std=self.meas_std,
            gallery_size=self.gallery_size,
        )
        self._next_track_id += 1
        trk.last_embed_ambig = bool(ambig)
        if embed is not None and bbox_area >= self.min_gallery_bbox_area \
                and not ambig:
            trk.push_embedding(embed, bbox_area)
        else:
            trk.last_bbox_area = int(bbox_area)
            if embed is not None:
                trk.last_embed = embed.astype(np.float32, copy=True)
        return trk

    @staticmethod
    def _bbox_iou(b1: tuple, b2: tuple) -> float:
        x1 = max(b1[0], b2[0])
        y1 = max(b1[1], b2[1])
        x2 = min(b1[2], b2[2])
        y2 = min(b1[3], b2[3])
        if x2 <= x1 or y2 <= y1:
            return 0.0
        inter = float((x2 - x1) * (y2 - y1))
        a1 = float((b1[2] - b1[0]) * (b1[3] - b1[1]))
        a2 = float((b2[2] - b2[0]) * (b2[3] - b2[1]))
        union = a1 + a2 - inter
        if union <= 0.0:
            return 0.0
        return inter / union

    def _compute_ambiguity_flags(self, bboxes: List[tuple]) -> List[bool]:
        n = len(bboxes)
        flags = [False] * n
        if n < 2:
            return flags
        thr = self.ambig_iou_threshold
        for i in range(n):
            for j in range(i + 1, n):
                iou = self._bbox_iou(bboxes[i], bboxes[j])
                if iou > thr:
                    flags[i] = True
                    flags[j] = True
        return flags

    def _apply_identity_first_master(
        self,
        cost: np.ndarray,
        det_embed: List[Optional[np.ndarray]],
        det_xy: List[np.ndarray],
        det_ambig: List[bool],
    ) -> None:
        """Override the cost matrix to force (or withhold) the master track's
        assignment based on appearance alone, before Hungarian runs.

        If we have no locked master (master_track_id <= 0) or no gallery,
        this is a no-op and standard Hungarian handles everything.
        """
        if self.master_track_id <= 0 or not self.master_gallery:
            return
        n_t, n_d = cost.shape
        if n_t == 0 or n_d == 0:
            return
        # Find the master track's row index.
        master_ti = -1
        for i, trk in enumerate(self.tracks):
            if trk.id == self.master_track_id:
                master_ti = i
                break
        if master_ti < 0:
            return

        # Rank detections by similarity to master_gallery.
        scored = []
        for j, embed in enumerate(det_embed):
            if embed is None:
                continue
            sim = self._similarity_against_master_gallery(embed)
            scored.append((j, sim))
        if not scored:
            # No embeddings available — leave Hungarian alone (fallback to
            # motion-only assoc for the master).
            return
        scored.sort(key=lambda p: p[1], reverse=True)
        best_j, s1 = scored[0]
        s2 = scored[1][1] if len(scored) >= 2 else 0.0

        # Check motion gate for the identity winner.
        master_trk = self.tracks[master_ti]
        m2 = master_trk.mahalanobis2(det_xy[best_j])
        motion_ok = m2 <= self.gate_mahalanobis

        # Choose thresholds based on whether the winner is in an overlap.
        ambig = bool(det_ambig[best_j])
        sim_gate = self.master_sim_ambig if ambig else self.master_sim_clear
        margin_gate = self.master_margin_ambig if ambig else self.master_margin_clear

        accept = (motion_ok
                  and s1 >= sim_gate
                  and (s1 - s2) >= margin_gate)

        from pfm_utils.association import LARGE_COST

        if accept:
            # Force the master track to take detection best_j.
            cost[master_ti, :] = LARGE_COST
            cost[:, best_j] = LARGE_COST
            cost[master_ti, best_j] = 0.0
        else:
            # Withhold the master from this frame's assignment — we'd
            # rather brief LOST than wrong lock.
            cost[master_ti, :] = LARGE_COST
            # Throttle the log so crowded scenes don't spam.
            now = self._now_sec()
            if now - self._last_idfirst_log_sec > 1.0:
                self.get_logger().info(
                    f"master id-first: withheld (s1={s1:.2f}, s2={s2:.2f}, "
                    f"margin={s1 - s2:.2f}, ambig={ambig}, "
                    f"gate={sim_gate:.2f}/{margin_gate:.2f}, "
                    f"motion_ok={motion_ok})")
                self._last_idfirst_log_sec = now

    def _find_track(self, track_id: int) -> Optional[KalmanTrack]:
        for trk in self.tracks:
            if trk.id == int(track_id):
                return trk
        return None

    def _transform_detection_to_odom(
        self,
        x_bf: float, y_bf: float,
        stamp: Optional[Time] = None,
    ) -> Optional[tuple]:
        tf_msg = lookup_transform(self.tf_buffer, self.odom_frame,
                                  self.base_frame, stamp, timeout_sec=0.1)
        if tf_msg is None:
            return None
        xo, yo, _ = apply_transform_2d(tf_msg, x_bf, y_bf, 0.0)
        return xo, yo

    def _bearing_in_base(self, xo: float, yo: float) -> Optional[tuple]:
        """Return (distance, bearing_rad) of an odom-frame point relative
        to base_footprint."""
        tf_msg = lookup_transform(self.tf_buffer, self.base_frame,
                                  self.odom_frame, None, timeout_sec=0.1)
        if tf_msg is None:
            return None
        xb, yb, _ = apply_transform_2d(tf_msg, xo, yo, 0.0)
        return math.hypot(xb, yb), math.atan2(yb, xb)

    # ========================================================================
    # Detection callback — the per-frame association + track lifecycle step
    # ========================================================================
    def _on_detections(self, msg: CamDetections) -> None:
        stamp_sec = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        stamp_time = Time(seconds=msg.stamp.sec, nanoseconds=msg.stamp.nanosec)

        # 1) Transform each detection from base_footprint (vision_node's
        #    global_frame) to odom so motion model sees stable coordinates.
        det_xy: List[np.ndarray] = []
        det_embed: List[Optional[np.ndarray]] = []
        det_area: List[int] = []
        det_bbox: List[tuple] = []  # (xmin, ymin, xmax, ymax), image pixels
        for d in msg.detections:
            pt = self._transform_detection_to_odom(float(d.x), float(d.y), stamp_time)
            if pt is None:
                continue
            det_xy.append(np.array([pt[0], pt[1]], dtype=np.float64))
            if len(d.embedding) > 0:
                det_embed.append(np.asarray(d.embedding, dtype=np.float32))
            else:
                det_embed.append(None)
            det_area.append(
                (int(d.xmax) - int(d.xmin)) * (int(d.ymax) - int(d.ymin)))
            det_bbox.append(
                (int(d.xmin), int(d.ymin), int(d.xmax), int(d.ymax)))

        # 1b) Flag detections whose bbox overlaps another detection.
        # When two person bboxes overlap in image space, OSNet crops from
        # EACH bbox capture pixels from BOTH identities, producing polluted
        # embeddings that are roughly equally similar to either gallery.
        # We use this flag to (1) require stricter Re-ID evidence before
        # assigning the master track to an ambiguous detection, and
        # (2) forbid gallery updates from polluted embeddings.
        det_ambig: List[bool] = self._compute_ambiguity_flags(det_bbox)

        # 2) Predict all tracks forward.
        for trk in self.tracks:
            trk.predict(stamp_sec)

        # 3) Build cost matrix and run Hungarian.
        detections_for_assoc = list(zip(det_xy, det_embed))
        cost = build_cost_matrix(
            self.tracks,
            detections_for_assoc,
            alpha_motion=self.alpha_motion,
            beta_reid=self.beta_reid,
            gate_mahalanobis=self.gate_mahalanobis,
            gate_reid=self.gate_reid,
        )

        # 3b) Identity-first master assignment. When we have a master identity
        # locked, the cost matrix alone can hijack the master track onto a
        # confounder during overlap (polluted embeddings make motion-gate the
        # decider). Instead:
        #   - Find the detection most similar to master_gallery.
        #   - Require a clear sim threshold AND convincing margin over second
        #     place. Thresholds tighten when the best detection is ambiguous.
        #   - Force this pair into the cost matrix; exclude other matches in
        #     that row/column.
        #   - If no detection is convincing enough, FORBID the master track
        #     from being assigned to anyone this frame (Kalman predicts;
        #     lost_count increments). Prefer brief LOST over wrong lock.
        self._apply_identity_first_master(cost, det_embed, det_xy, det_ambig)

        matches, unmatched_tracks, unmatched_dets = solve_assignment(cost)

        # 4) Apply updates and bookkeeping.
        for ti, di in matches:
            trk = self.tracks[ti]
            z = det_xy[di]
            trk.update(z, stamp_sec)
            if det_embed[di] is not None:
                ambig = det_ambig[di]
                trk.last_embed_ambig = ambig
                # Protect the track's OWN gallery from polluted embeddings
                # too — it gets used in the next frame's association cost.
                if det_area[di] >= self.min_gallery_bbox_area and not ambig:
                    trk.push_embedding(det_embed[di], det_area[di])
                else:
                    trk.last_embed = det_embed[di].astype(np.float32, copy=True)
                    trk.last_bbox_area = int(det_area[di])

        for ti in unmatched_tracks:
            self.tracks[ti].mark_missed()

        for di in unmatched_dets:
            self.tracks.append(
                self._spawn_track(det_xy[di], stamp_sec,
                                  det_embed[di], det_area[di],
                                  ambig=det_ambig[di]))

        # 5) Promote / demote / delete.
        kept: List[KalmanTrack] = []
        for trk in self.tracks:
            # Revive LOST → CONFIRMED when we get a fresh association hit.
            # Without this, the old master track stays LOST forever even
            # after being matched to a new detection, so _best_track_similarity
            # (which filters to CONFIRMED/TENTATIVE) never considers it and
            # the state machine can't re-lock. Demote-back from LOST is the
            # whole point of letting a lost track be re-observed.
            # Revival requires a CLEAN (non-ambiguous) association —
            # don't bring a LOST track back to life on the strength of a
            # polluted overlap embedding.
            if trk.status == "LOST" and trk.lost_count == 0 \
                    and trk.hits >= self.tentative_hits \
                    and not trk.last_embed_ambig:
                trk.status = "CONFIRMED"
                if trk.id == self.master_track_id:
                    self.get_logger().info(
                        f"master track #{trk.id} revived from LOST "
                        f"(hits={trk.hits})")
            if trk.status == "TENTATIVE" and trk.hits >= self.tentative_hits:
                trk.status = "CONFIRMED"
            if trk.status == "CONFIRMED" and trk.lost_count > self.confirmed_lost_limit:
                trk.status = "LOST"
            # Delete old LOST tracks, except the master — the state machine
            # needs to keep the master track alive across the lost_timeout.
            if trk.status == "LOST" and trk.lost_count > self.delete_lost_limit \
                    and trk.id != self.master_track_id:
                continue
            # Aggressive cleanup of stale TENTATIVE tracks that never
            # matured (e.g. spurious detections).
            if trk.status == "TENTATIVE" and trk.lost_count > self.confirmed_lost_limit:
                continue
            kept.append(trk)
        self.tracks = kept

        # 6) Master identity state machine.
        self._step_master_state(stamp_sec)

        # 7) Publish /tracks immediately (master republishes on timer).
        self._publish_tracks(stamp_sec)

    # ========================================================================
    # Master state machine
    # ========================================================================
    def _step_master_state(self, stamp_sec: float) -> None:
        if self.master_state in (STATE_IDLE, STATE_ENROLLING):
            self._step_enrollment(stamp_sec)
        elif self.master_state == STATE_FOLLOWING:
            self._step_following(stamp_sec)
        elif self.master_state == STATE_LOST:
            self._step_lost(stamp_sec)

    def _enrollment_candidates(self) -> List[tuple]:
        """Return list of (track, distance, bearing) for tracks in the
        enrollment window."""
        out = []
        for trk in self.tracks:
            if trk.status != "CONFIRMED":
                continue
            db = self._bearing_in_base(float(trk.x[0]), float(trk.x[1]))
            if db is None:
                continue
            dist, bearing = db
            if (self.enroll_min_dist <= dist <= self.enroll_max_dist
                    and abs(bearing) <= self.enroll_bearing_rad):
                out.append((trk, dist, bearing))
        return out

    def _step_enrollment(self, stamp_sec: float) -> None:
        candidates = self._enrollment_candidates()
        if len(candidates) != 1:
            # Need exactly one person in the window to avoid ambiguity.
            self._enroll_candidate_id = -1
            self._enroll_start_sec = 0.0
            self.master_state = STATE_IDLE
            return

        trk = candidates[0][0]
        if trk.id != self._enroll_candidate_id:
            self._enroll_candidate_id = trk.id
            self._enroll_start_sec = stamp_sec
            self.master_state = STATE_ENROLLING
            self.get_logger().info(
                f"enrollment: candidate track #{trk.id} stable window started")
            return

        held = stamp_sec - self._enroll_start_sec
        if held >= self.enroll_stable_sec:
            self._commit_master(trk)

    def _commit_master(self, trk: KalmanTrack) -> None:
        self.master_track_id = int(trk.id)
        self.master_gallery.clear()
        # Seed master gallery with the candidate's current gallery.
        for emb in list(trk.gallery):
            self.master_gallery.append(emb.astype(np.float32, copy=True))
        if trk.last_embed is not None and not self.master_gallery:
            self.master_gallery.append(trk.last_embed.astype(np.float32, copy=True))
        if self.master_gallery:
            self._master_gallery_anchor = self.master_gallery[0].copy()
        self.master_state = STATE_FOLLOWING
        self.last_master_pose = (float(trk.x[0]), float(trk.x[1]), self._now_sec())
        self.get_logger().info(
            f"MASTER ENROLLED: track #{self.master_track_id} "
            f"(gallery size={len(self.master_gallery)})")

    def _best_track_similarity(self, include_tentative: bool = False) -> tuple:
        """Return (best_track, s1, s2). `include_tentative` is used by
        the LOST state so a freshly-spawned track (which would otherwise
        need ~3 frames to reach CONFIRMED) can re-lock the master
        immediately if its appearance strongly matches the gallery."""
        if not self.master_gallery:
            return (None, 0.0, 0.0)
        scored = []
        for trk in self.tracks:
            if trk.status == "CONFIRMED":
                pass
            elif trk.status == "TENTATIVE" and include_tentative:
                pass
            else:
                continue
            if trk.last_embed is None:
                continue
            # s = max over master_gallery of cosine(track.last_embed, gallery_k)
            best = 0.0
            qn = float(np.linalg.norm(trk.last_embed))
            if qn < 1e-8:
                continue
            q = trk.last_embed / qn
            for g in self.master_gallery:
                gn = float(np.linalg.norm(g))
                if gn < 1e-8:
                    continue
                sim = float(np.dot(q, g / gn))
                if sim > best:
                    best = sim
            scored.append((trk, best))
        if not scored:
            return (None, 0.0, 0.0)
        scored.sort(key=lambda pair: pair[1], reverse=True)
        best_trk, s1 = scored[0]
        s2 = scored[1][1] if len(scored) >= 2 else 0.0
        return (best_trk, s1, s2)

    def _step_following(self, stamp_sec: float) -> None:
        cur = self._find_track(self.master_track_id)
        best_trk, s1, s2 = self._best_track_similarity()

        # Case A: current master track still alive, similarity still acceptable.
        if cur is not None and cur.status == "CONFIRMED":
            cur_sim = self._similarity_against_master_gallery(cur.last_embed)
            if cur_sim >= self.keep_min_sim:
                self._maybe_push_master_gallery(cur, cur_sim)
                self.last_master_pose = (float(cur.x[0]), float(cur.x[1]), self._now_sec())
                return
            # Current master's similarity dropped — consider relock.

        # Case B: relock to the best match if score is clearly above threshold
        # and the margin over second-best is convincing.
        if best_trk is not None and s1 >= self.relock_min_sim and (s1 - s2) >= self.relock_margin:
            if best_trk.id != self.master_track_id:
                self.get_logger().info(
                    f"MASTER RELOCKED: #{self.master_track_id} -> #{best_trk.id} "
                    f"(s1={s1:.2f}, s2={s2:.2f})")
                self.master_track_id = int(best_trk.id)
            self._maybe_push_master_gallery(best_trk, s1)
            self.last_master_pose = (float(best_trk.x[0]), float(best_trk.x[1]), self._now_sec())
            return

        # Case C: current master is LOST or no acceptable match.
        # Move to LOST state; don't clear the anchor pose yet.
        self.get_logger().warn(
            f"MASTER LOST: s1={s1:.2f} margin={s1 - s2:.2f} — entering LOST")
        self.master_state = STATE_LOST
        self._last_alarm_log_sec = stamp_sec

    def _step_lost(self, stamp_sec: float) -> None:
        if self.last_master_pose is None:
            # Shouldn't happen, but guard against it.
            self.master_state = STATE_IDLE
            self.master_track_id = -1
            return

        # Include TENTATIVE tracks: the master may re-enter the FOV
        # from the side and produce a fresh track that hasn't yet
        # reached CONFIRMED. We don't want to wait ~0.3 s for the
        # 3-hit promotion before even considering re-lock.
        best_trk, s1, _ = self._best_track_similarity(include_tentative=True)
        last_x, last_y, _ = self.last_master_pose
        time_since = self._now_sec() - self.last_master_pose[2]

        # Adaptive gates: the longer we've been searching, the more
        # lenient we must be. Lighting, pose and scale will all have
        # drifted, and the last-known pose becomes meaningless after
        # the master has walked away and come back.
        sim_gate = max(self.recovery_sim_floor,
                       self.recovery_max_sim_gate - self.recovery_sim_decay * time_since)
        dist_gate = self.recovery_max_dist + self.recovery_dist_growth * time_since

        if best_trk is not None and s1 >= sim_gate:
            dx = float(best_trk.x[0]) - last_x
            dy = float(best_trk.x[1]) - last_y
            gap = math.hypot(dx, dy)
            if gap <= dist_gate:
                self.master_track_id = int(best_trk.id)
                self.master_state = STATE_FOLLOWING
                self.last_master_pose = (float(best_trk.x[0]), float(best_trk.x[1]),
                                         self._now_sec())
                self._maybe_push_master_gallery(best_trk, s1)
                self.get_logger().info(
                    f"MASTER RECOVERED: track #{best_trk.id} "
                    f"sim={s1:.2f} (gate={sim_gate:.2f}), "
                    f"gap={gap:.2f}m (gate={dist_gate:.2f}m), "
                    f"after {time_since:.1f}s")
                return

        # Never give up — keep searching forever. Log progress
        # periodically so the user sees we are still trying.
        if time_since > self.lost_timeout_sec:
            if stamp_sec - self._last_alarm_log_sec > 2.0:
                self.get_logger().warn(
                    f"still searching for master ({time_since:.0f}s, "
                    f"best s={s1:.2f} gate={sim_gate:.2f})")
                self._last_alarm_log_sec = stamp_sec

    def _similarity_against_master_gallery(self, embed: Optional[np.ndarray]) -> float:
        if embed is None or not self.master_gallery:
            return 0.0
        qn = float(np.linalg.norm(embed))
        if qn < 1e-8:
            return 0.0
        q = embed / qn
        best = 0.0
        for g in self.master_gallery:
            gn = float(np.linalg.norm(g))
            if gn < 1e-8:
                continue
            sim = float(np.dot(q, g / gn))
            if sim > best:
                best = sim
        return best

    def _maybe_push_master_gallery(self, trk: KalmanTrack, sim: float) -> None:
        if trk.last_embed is None:
            return
        if trk.last_bbox_area < self.hq_area:
            return
        if sim < self.hq_sim:
            return
        # Never push a polluted (overlap-crop) embedding into master_gallery
        # — it would bias future matching toward whoever was standing next
        # to the master at the time.
        if trk.last_embed_ambig:
            return
        # Preserve anchor at index 0: evict oldest non-anchor when full.
        if len(self.master_gallery) >= self.master_gallery.maxlen:
            # deque maxlen handles FIFO. If anchor is still at index 0, fine.
            pass
        self.master_gallery.append(trk.last_embed.astype(np.float32, copy=True))

    # ========================================================================
    # Services
    # ========================================================================
    def _on_enroll_srv(self, request: Trigger.Request,
                       response: Trigger.Response) -> Trigger.Response:
        candidates = self._enrollment_candidates()
        if not candidates:
            response.success = False
            response.message = "No candidate in enrollment window (dist, bearing)."
            return response
        candidates.sort(key=lambda t: (abs(t[2]), t[1]))  # smallest bearing, then distance
        trk = candidates[0][0]
        self._commit_master(trk)
        response.success = True
        response.message = f"Master enrolled immediately as track #{trk.id}."
        return response

    def _on_reset_srv(self, request: Trigger.Request,
                      response: Trigger.Response) -> Trigger.Response:
        old = self.master_track_id
        self.master_track_id = -1
        self.master_state = STATE_IDLE
        self.master_gallery.clear()
        self._master_gallery_anchor = None
        self.last_master_pose = None
        self._enroll_candidate_id = -1
        self._enroll_start_sec = 0.0
        response.success = True
        response.message = f"Master reset (was #{old})."
        self.get_logger().info(response.message)
        return response

    # ========================================================================
    # Publishers
    # ========================================================================
    def _publish_tracks(self, stamp_sec: float) -> None:
        msg = Tracks()
        msg.header = Header()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for trk in self.tracks:
            if trk.status == "TENTATIVE":
                continue
            tm = TrackMsg()
            tm.id = int(trk.id)
            tm.x = float(trk.x[0])
            tm.y = float(trk.x[1])
            tm.vx = float(trk.x[2])
            tm.vy = float(trk.x[3])
            tm.reid_similarity = float(
                self._similarity_against_master_gallery(trk.last_embed))
            tm.hits = int(min(65535, trk.hits))
            tm.lost_count = int(min(65535, trk.lost_count))
            tm.is_master = bool(trk.id == self.master_track_id)
            tm.status = str(trk.status)
            msg.tracks.append(tm)
        self.pub_tracks.publish(msg)

    def _on_publish_timer(self) -> None:
        now_ros = self.get_clock().now().to_msg()
        now_sec = self._now_sec()

        mt = MasterTarget()
        mt.header.frame_id = self.odom_frame
        mt.header.stamp = now_ros
        mt.track_id = int(self.master_track_id)
        mt.state = str(self.master_state)
        mt.is_locked = bool(self.master_state == STATE_FOLLOWING)

        cur = self._find_track(self.master_track_id) if self.master_track_id > 0 else None
        if cur is not None and self.master_state == STATE_FOLLOWING:
            mt.x = float(cur.x[0])
            mt.y = float(cur.x[1])
            mt.vx = float(cur.x[2])
            mt.vy = float(cur.x[3])
            db = self._bearing_in_base(mt.x, mt.y)
            if db is not None:
                mt.distance = float(db[0])
                mt.bearing = float(db[1])
            mt.reid_similarity = float(
                self._similarity_against_master_gallery(cur.last_embed))
            mt.time_since_last_seen = 0.0
        elif self.last_master_pose is not None and self.master_state == STATE_LOST:
            mt.x = float(self.last_master_pose[0])
            mt.y = float(self.last_master_pose[1])
            mt.vx = 0.0
            mt.vy = 0.0
            db = self._bearing_in_base(mt.x, mt.y)
            if db is not None:
                mt.distance = float(db[0])
                mt.bearing = float(db[1])
            mt.reid_similarity = 0.0
            mt.time_since_last_seen = float(now_sec - self.last_master_pose[2])
        else:
            mt.x = 0.0
            mt.y = 0.0
            mt.vx = 0.0
            mt.vy = 0.0
            mt.distance = 0.0
            mt.bearing = 0.0
            mt.reid_similarity = 0.0
            mt.time_since_last_seen = 0.0

        self.pub_master.publish(mt)

        markers = build_track_markers(
            self.tracks, self.master_track_id, self.odom_frame, now_ros)
        self.pub_markers.publish(markers)


def main() -> None:
    rclpy.init()
    node = MasterTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
