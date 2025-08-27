#!/usr/bin/env python3
# semantic_query_node.py
#
# Minimal ROS 2 Humble node for querying a scene graph JSON by text.
# - JSON schema: { world_id, frame_id, keyframes:[ { pose, place{label,aliases,conf}, objects:[{name,aliases,category,conf}] } ] }
# - Subscribes: /semantic_query (std_msgs/String)  e.g., "where is the toilet?" or "go to the coffee machine"
# - Publishes:  /semantic_goal  (geometry_msgs/PoseStamped)
#
# IMPORTANT: No hardcoded synonyms. Only terms present in the JSON are used.
#
# Run:
#   ros2 run <your_pkg> semantic_query_node.py \
#     --ros-args -p scene_json_path:=/home/$USER/.ros/semmap/scene_graph.json

import json
import math
import os
import re
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


def canon(s: Optional[str]) -> str:
    """Lowercase & collapse spaces; strip most punctuation except alphanumerics and spaces."""
    if not s:
        return ""
    s = s.lower()
    s = re.sub(r"[^\w\s]", " ", s)      # remove punctuation
    s = re.sub(r"\s+", " ", s).strip()  # collapse spaces
    return s

def term_in_text(term: str, text: str) -> bool:
    """Simple containment check after canonicalization."""
    t = canon(term)
    q = canon(text)
    return t and (t in q or q in t)  # allow either direction for partial phrasing

def euclid2d(pose_dict: Dict[str, Any], ref_xy: Tuple[float, float]) -> float:
    x = float(pose_dict["position"]["x"])
    y = float(pose_dict["position"]["y"])
    return math.hypot(x - ref_xy[0], y - ref_xy[1])

def to_posestamped(p: Dict[str, Any], fallback_frame: str) -> PoseStamped:
    frame_id = p.get("frame_id", fallback_frame)
    pos = p["position"]
    ori = p.get("orientation", {}) or {}
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose = Pose(
        position=Point(x=float(pos["x"]), y=float(pos["y"]), z=float(pos.get("z", 0.0))),
        orientation=Quaternion(
            x=float(ori.get("x", 0.0)),
            y=float(ori.get("y", 0.0)),
            z=float(ori.get("z", 0.0)),
            w=float(ori.get("w", 1.0)),
        ),
    )
    return msg


class SemanticQueryNode(Node):
    def __init__(self):
        super().__init__("semantic_query_node")

        # --- Parameters ---
        self.declare_parameter("scene_json_path", os.path.expanduser("~/.ros/semmap/scene_graph.json"))
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("prefer_nearest_to", "")   # optional "x,y" tiebreaker
        self.declare_parameter("enable_place_search", True)
        self.declare_parameter("enable_object_search", True)

        self.scene_json_path = self.get_parameter("scene_json_path").get_parameter_value().string_value
        self.default_frame   = self.get_parameter("default_frame_id").get_parameter_value().string_value
        self.prefer_nearest  = self.get_parameter("prefer_nearest_to").get_parameter_value().string_value
        self.use_place       = self.get_parameter("enable_place_search").get_parameter_value().bool_value
        self.use_object      = self.get_parameter("enable_object_search").get_parameter_value().bool_value

        # --- IO ---
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.sub_q    = self.create_subscription(String, "/semantic_query", self.on_query, 10)

        # --- Load & auto-reload scene file ---
        self.scene = self._load_scene(self.scene_json_path)
        self.timer = self.create_timer(2.0, self._maybe_reload)  # cheap reload

        # Build an index of terms available in the scene (for debug/logging only)
        self.place_terms, self.object_terms = self._collect_terms(self.scene)

        self.get_logger().info(
            f"[semantic_query_node] Loaded scene '{self.scene.get('world_id','?')}', "
            f"frame='{self.scene.get('frame_id', self.default_frame)}'. "
            f"Place terms: {sorted(self.place_terms) if self.place_terms else []} | "
            f"Object terms: {sorted(list(self.object_terms)[:10]) + (['...'] if len(self.object_terms)>10 else [])}"
        )

    # ---------- Scene I/O ----------
    def _load_scene(self, path: str) -> Dict[str, Any]:
        try:
            with open(path, "r") as f:
                data = json.load(f)
            self._last_mtime = os.path.getmtime(path)
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to read scene JSON at {path}: {e}")
            return {}

    def _maybe_reload(self):
        try:
            mtime = os.path.getmtime(self.scene_json_path)
            if getattr(self, "_last_mtime", None) is None or mtime > self._last_mtime:
                self.scene = self._load_scene(self.scene_json_path)
                self.place_terms, self.object_terms = self._collect_terms(self.scene)
                self.get_logger().info("[semantic_query_node] Scene reloaded.")
        except FileNotFoundError:
            pass

    def _collect_terms(self, scene: Dict[str, Any]) -> Tuple[set, set]:
        """Collect only the terms that exist in the scene (no extra synonyms)."""
        place_terms = set()
        object_terms = set()
        for kf in scene.get("keyframes", []) or []:
            plc = kf.get("place", {}) or {}
            lbl = canon(plc.get("label", ""))
            if lbl:
                place_terms.add(lbl)
            for a in (plc.get("aliases", []) or []):
                ca = canon(a)
                if ca:
                    place_terms.add(ca)
            for obj in (kf.get("objects", []) or []):
                n = canon(obj.get("name", ""))
                if n:
                    object_terms.add(n)
                for a in (obj.get("aliases", []) or []):
                    ca = canon(a)
                    if ca:
                        object_terms.add(ca)
                c = canon(obj.get("category", ""))
                if c:
                    object_terms.add(c)
        return place_terms, object_terms

    # ---------- Query handling ----------
    def on_query(self, msg: String):
        if not self.scene:
            self.get_logger().warn("No scene loaded; ignoring query.")
            return

        scene_frame = self.scene.get("frame_id", self.default_frame)
        qtext = msg.data or ""
        qcanon = canon(qtext)

        candidates: List[Dict[str, Any]] = []

        if self.use_place:
            candidates += self._gather_place_candidates(qcanon, scene_frame)
        if self.use_object:
            candidates += self._gather_object_candidates(qcanon, scene_frame)

        if not candidates:
            self.get_logger().warn(
                f"No match for query='{qtext}'. "
                f"Known place terms: {sorted(self.place_terms)}; "
                f"known object terms (sample): {sorted(list(self.object_terms)[:10]) + (['...'] if len(self.object_terms)>10 else [])}"
            )
            return

        # Rank: higher score first; tie-breaker: nearest to optional point
        candidates.sort(key=lambda c: -c.get("score", 0.0))
        if self.prefer_nearest:
            try:
                px, py = [float(v) for v in self.prefer_nearest.split(",")]
                candidates.sort(key=lambda c: euclid2d(c["pose"], (px, py)))
            except Exception:
                pass

        best = candidates[0]
        goal_msg = to_posestamped(best["pose"], fallback_frame=scene_frame)
        self.pub_goal.publish(goal_msg)

        self.get_logger().info(
            f"[semantic_query_node] Query='{qtext}' → "
            f"type={best.get('type')} src={best.get('src')} "
            f"→ goal=({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}) "
            f"frame='{goal_msg.header.frame_id}' score={best.get('score','n/a')}"
        )

    # ---------- Candidate builders ----------
    def _gather_place_candidates(self, qcanon: str, scene_frame: str) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for kf in self.scene.get("keyframes", []) or []:
            plc = kf.get("place", {}) or {}
            pose = kf.get("pose")
            if not pose:
                continue  # cannot navigate without pose

            # Terms we are allowed to use (only what's in the scene)
            terms = []
            lbl = plc.get("label", None)
            if lbl:
                terms.append(lbl)
            terms += plc.get("aliases", []) or []

            # Did any term appear in the query?
            matched = False
            match_bonus = 0.0
            for t in terms:
                if term_in_text(t, qcanon):
                    matched = True
                    # small boost for longer (more specific) terms
                    match_bonus = max(match_bonus, min(len(canon(t)) / 20.0, 0.3))
            if not matched:
                continue

            # Score: use provided conf if any, else default 0.7; add small bonus for specificity
            base = plc.get("conf", None)
            score = (float(base) if base is not None else 0.7) + match_bonus

            pose_out = {
                "position": {
                    "x": pose["position"]["x"],
                    "y": pose["position"]["y"],
                    "z": pose["position"].get("z", 0.0),
                },
                "orientation": pose.get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
                "frame_id": pose.get("frame_id", scene_frame),
            }
            out.append({"type": "place", "src": f'keyframe:{kf.get("kf_id","")}', "pose": pose_out, "score": score})
        return out

    def _gather_object_candidates(self, qcanon: str, scene_frame: str) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for kf in self.scene.get("keyframes", []) or []:
            pose = kf.get("pose")
            if not pose:
                continue

            best_obj_score = None
            any_match = False

            for obj in (kf.get("objects", []) or []):
                # Allowed terms: object name + aliases + category (all from JSON)
                terms = []
                if obj.get("name"):
                    terms.append(obj["name"])
                terms += obj.get("aliases", []) or []
                if obj.get("category"):
                    terms.append(obj["category"])

                matched_this_obj = False
                match_bonus = 0.0
                for t in terms:
                    if term_in_text(t, qcanon):
                        matched_this_obj = True
                        # boost by term length (specificity) up to 0.3
                        match_bonus = max(match_bonus, min(len(canon(t)) / 20.0, 0.3))

                if matched_this_obj:
                    any_match = True
                    base = obj.get("conf", None)
                    score = (float(base) if base is not None else 0.6) + match_bonus
                    if best_obj_score is None or score > best_obj_score:
                        best_obj_score = score

            if any_match:
                pose_out = {
                    "position": {
                        "x": pose["position"]["x"],
                        "y": pose["position"]["y"],
                        "z": pose["position"].get("z", 0.0),
                    },
                    "orientation": pose.get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
                    "frame_id": pose.get("frame_id", scene_frame),
                }
                out.append({
                    "type": "object",
                    "src": f'keyframe:{kf.get("kf_id","")}',
                    "pose": pose_out,
                    "score": float(best_obj_score),
                })
        return out


def main():
    rclpy.init()
    node = SemanticQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
