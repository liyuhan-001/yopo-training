#!/usr/bin/env python3
import argparse
import csv
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
import sensor_msgs.point_cloud2 as pc2


def stamp_seconds(msg):
    stamp = msg.header.stamp.to_sec()
    return stamp if stamp > 0 else time.monotonic()


class Stage5Logger:
    def __init__(self, args):
        rospy.init_node("stage5_logger")
        self.args = args
        self.lock = threading.RLock()
        self.latest_odom = None
        self.latest_lidar_min = None
        self.active = False
        self.collision_topic_seen = False
        self.collision_state = False

        rospy.Subscriber("/sim/odom", Odometry, self.odom_cb, queue_size=100)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                         self.goal_cb, queue_size=10)
        rospy.Subscriber("/lidar_points", PointCloud2,
                         self.lidar_cb, queue_size=10)
        rospy.Subscriber("/yopo/inference_ms", Float32,
                         self.inference_cb, queue_size=100)
        # 当前源码默认没有发布这个话题；以后添加模拟器碰撞检测后即可记录。
        rospy.Subscriber("/sim/collision", Bool,
                         self.collision_cb, queue_size=20)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Stage5 logger ready; click a 2D Nav Goal to start one trial.")
        rospy.spin()

    def goal_cb(self, msg):
        with self.lock:
            if self.latest_odom is None:
                rospy.logwarn("No odometry yet; goal ignored. Click the goal again.")
                return
            if self.active:
                self.finish(False, "goal_replaced")

            p = self.latest_odom["p"].copy()
            self.start = p
            # 与当前 test_yopo_ros.py 一致：目标高度固定为 2 m。
            self.goal = np.array([msg.pose.position.x, msg.pose.position.y, 2.0])
            delta = self.goal - self.start
            norm = np.linalg.norm(delta)
            self.goal_axis = delta / norm if norm > 1e-6 else np.zeros(3)

            self.start_t = self.latest_odom["t"]
            self.prev_t = self.start_t
            self.prev_p = p
            self.path_m = 0.0
            self.speed_integral = 0.0
            self.speed_time = 0.0
            self.min_lidar_m = float("inf")
            self.last_lidar_min = None
            self.near_count = 0
            self.near_active = False
            self.collision_count = 0
            self.collision_topic_seen = False
            self.collision_state = False
            self.infer_sum_ms = 0.0
            self.infer_count = 0
            self.stagnation_events = 0
            self.stagnation_window = deque([(self.start_t, p.copy())])
            self.last_stagnation_t = self.start_t
            self.last_progress_sign = None
            self.oscillation_reversals = 0
            self.anomalies = set()
            self.rows = []
            self.active = True

            rospy.loginfo(
                "Trial %s started: start=%s goal=%s",
                self.args.run_id, np.round(self.start, 3), np.round(self.goal, 3)
            )

    def odom_cb(self, msg):
        t = stamp_seconds(msg)
        p = np.array([msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z])
        v = np.array([msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])

        with self.lock:
            self.latest_odom = {"t": t, "p": p, "v": v}
            if not self.active:
                return

            dt = t - self.prev_t
            if dt <= 0:
                self.anomalies.add("non_monotonic_odom_time")
                return
            if dt > 0.5:
                self.anomalies.add("odom_gap_over_0.5s")

            self.path_m += float(np.linalg.norm(p - self.prev_p))
            speed = float(np.linalg.norm(v))
            self.speed_integral += speed * dt
            self.speed_time += dt

            goal_error = float(np.linalg.norm(self.goal - p))
            elapsed = t - self.start_t

            # 连续 5 秒位移很小、且仍未接近目标，记为一次疑似停滞。
            self.stagnation_window.append((t, p.copy()))
            while self.stagnation_window and t - self.stagnation_window[0][0] > 5.0:
                self.stagnation_window.popleft()
            if (len(self.stagnation_window) > 1
                    and t - self.stagnation_window[0][0] >= 4.8
                    and np.linalg.norm(p - self.stagnation_window[0][1]) < 0.15
                    and goal_error > self.args.success_radius
                    and t - self.last_stagnation_t >= 5.0):
                self.stagnation_events += 1
                self.last_stagnation_t = t

            # 沿起点到目标方向的速度反复换向，作为疑似振荡指标。
            progress_speed = float(np.dot(v, self.goal_axis))
            if abs(progress_speed) > 0.15:
                sign = 1 if progress_speed > 0 else -1
                if self.last_progress_sign is not None and sign != self.last_progress_sign:
                    self.oscillation_reversals += 1
                self.last_progress_sign = sign

            if speed > max(1.0, 1.5 * self.args.speed):
                self.anomalies.add("overspeed")

            self.rows.append({
                "stamp": t,
                "x": p[0], "y": p[1], "z": p[2],
                "vx": v[0], "vy": v[1], "vz": v[2],
                "speed_mps": speed,
                "goal_error_m": goal_error,
                "path_length_m": self.path_m,
                "min_lidar_range_m": self.last_lidar_min
            })

            self.prev_t = t
            self.prev_p = p

            if goal_error <= self.args.success_radius:
                self.finish(True, "reached")
            elif elapsed >= self.args.timeout:
                self.anomalies.add("timeout")
                self.finish(False, "timeout")

    def lidar_cb(self, msg):
        try:
            distances = (
                math.sqrt(x*x + y*y + z*z)
                for x, y, z in pc2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            )
            dmin = min(distances, default=None)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Could not read LiDAR points: %s", exc)
            return

        if dmin is None:
            return

        with self.lock:
            self.latest_lidar_min = dmin
            if not self.active:
                return
            self.last_lidar_min = dmin
            self.min_lidar_m = min(self.min_lidar_m, dmin)

            # 这是近碰撞代理指标，不等同于模拟器真实接触事件。
            if dmin < self.args.near_radius and not self.near_active:
                self.near_count += 1
                self.near_active = True
            elif dmin > self.args.near_radius + 0.10:
                self.near_active = False

    def inference_cb(self, msg):
        with self.lock:
            if self.active:
                self.infer_sum_ms += float(msg.data)
                self.infer_count += 1

    def collision_cb(self, msg):
        with self.lock:
            if not self.active:
                return
            self.collision_topic_seen = True
            if msg.data and not self.collision_state:
                self.collision_count += 1
            self.collision_state = bool(msg.data)

    def finish(self, success, reason):
        if not self.active:
            return

        end_t = self.prev_t
        end_p = self.prev_p
        flight_time = max(0.0, end_t - self.start_t)
        final_error = float(np.linalg.norm(self.goal - end_p))
        avg_speed = self.speed_integral / self.speed_time if self.speed_time > 0 else 0.0
        avg_infer = self.infer_sum_ms / self.infer_count if self.infer_count else ""

        run_dir = os.path.expanduser(self.args.out_dir)
        os.makedirs(run_dir, exist_ok=True)

        raw_path = os.path.join(run_dir, self.args.run_id + "_odom.csv")
        if self.rows:
            with open(raw_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self.rows[0].keys())
                writer.writeheader()
                writer.writerows(self.rows)

        summary_path = os.path.join(run_dir, "summary.csv")
        row = {
            "run_id": self.args.run_id,
            "scene_type": self.args.scene_type,
            "maze_type": self.args.maze_type,
            "tree_dist_m": self.args.tree_dist,
            "seed": self.args.seed,
            "target_speed_mps": self.args.speed,
            "start_x": self.start[0], "start_y": self.start[1], "start_z": self.start[2],
            "goal_x": self.goal[0], "goal_y": self.goal[1], "goal_z": self.goal[2],
            "success": int(success),
            "finish_reason": reason,
            "final_goal_error_m": final_error,
            "flight_time_s": flight_time,
            "path_length_m": self.path_m,
            "avg_speed_mps": avg_speed,
            "min_lidar_range_m": "" if math.isinf(self.min_lidar_m) else self.min_lidar_m,
            "avg_inference_ms": avg_infer,
            "collision_topic_seen": int(self.collision_topic_seen),
            "collision_count": self.collision_count if self.collision_topic_seen else "",
            "near_collision_proxy_count": self.near_count,
            "stagnation_events": self.stagnation_events,
            "oscillation_flag": int(self.oscillation_reversals >= 4),
            "oscillation_reversals": self.oscillation_reversals,
            "other_anomalies": ";".join(sorted(self.anomalies)) or self.args.notes
        }
        exists = os.path.exists(summary_path)
        with open(summary_path, "a", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            if not exists:
                writer.writeheader()
            writer.writerow(row)

        self.active = False
        rospy.loginfo("Saved %s and %s", raw_path, summary_path)

    def on_shutdown(self):
        with self.lock:
            if self.active:
                self.finish(False, "manual_stop")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-id", required=True)
    p.add_argument("--tree-dist", type=float, required=True)
    p.add_argument("--seed", type=int, default=3)
    p.add_argument("--speed", type=float, default=6.0)
    p.add_argument("--maze-type", type=int, default=5)
    p.add_argument("--scene-type", default="Random Forest")
    p.add_argument("--success-radius", type=float, default=5.0)
    p.add_argument("--near-radius", type=float, default=0.25)
    p.add_argument("--timeout", type=float, default=120.0)
    p.add_argument("--out-dir", default="~/YOPO/stage5_experiment")
    p.add_argument("--notes", default="")
    Stage5Logger(p.parse_args())


if __name__ == "__main__":
    main()
