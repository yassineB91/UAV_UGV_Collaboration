#!/usr/bin/env python3
import argparse
import csv
import json
import math
from itertools import combinations
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

matplotlib.use("Agg")
import matplotlib.pyplot as plt


CORE_POSE_TOPICS = {
    "/ugv/compare/odom_only_pose": "odom_only",
    "/ugv/compare/icp_pose": "icp",
    "/ugv/compare/scan_match_pose": "scan_match",
}

OPTIONAL_COMPARISON_POSE_TOPICS = {
    "/ugv/compare/uav_pose": "uav",
}

OPTIONAL_POSE_TOPICS = {
    "/ugv/odom": "odom_source",
    "/ugv/icp/pose": "icp_source",
    "/ugv/scan_match/pose": "scan_match_source",
    "/uav/pose": "uav_source",
}

OPTIONAL_CONTEXT_TOPICS = {
    "/ugv/scan": "scan",
    "/map": "map",
    "/uav/detection_boxes": "detections",
    "/uav/obstacle_mask": "obstacle_mask",
    "/clock": "clock",
}


def wrap_to_pi(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(math.atan2(siny_cosp, cosy_cosp))


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def init_csv(path: Path, header):
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    writer.writerow(header)
    return handle, writer


def pose_from_message(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        return msg.pose.pose
    if hasattr(msg, "pose"):
        return msg.pose
    raise TypeError(f"Unsupported pose message type: {type(msg)!r}")


def write_pose_row(writer, bag_time_ns: int, msg):
    pose = pose_from_message(msg)
    q = pose.orientation
    writer.writerow(
        [
            bag_time_ns,
            stamp_to_ns(msg.header.stamp),
            msg.header.frame_id,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            q.x,
            q.y,
            q.z,
            q.w,
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        ]
    )


def export_scan_stats(writer, bag_time_ns: int, msg):
    valid_ranges = np.asarray(
        [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max],
        dtype=float,
    )
    if valid_ranges.size == 0:
        min_valid = np.nan
        max_valid = np.nan
        mean_valid = np.nan
    else:
        min_valid = float(valid_ranges.min())
        max_valid = float(valid_ranges.max())
        mean_valid = float(valid_ranges.mean())
    writer.writerow(
        [
            bag_time_ns,
            stamp_to_ns(msg.header.stamp),
            msg.header.frame_id,
            len(msg.ranges),
            int(valid_ranges.size),
            min_valid,
            max_valid,
            mean_valid,
        ]
    )


def export_map_stats(writer, bag_time_ns: int, msg):
    data = np.asarray(msg.data, dtype=np.int16)
    writer.writerow(
        [
            bag_time_ns,
            stamp_to_ns(msg.header.stamp),
            msg.header.frame_id,
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            int(np.count_nonzero(data > 0)),
            int(np.count_nonzero(data == 0)),
            int(np.count_nonzero(data < 0)),
        ]
    )


def export_detection_stats(writer, bag_time_ns: int, msg):
    class_ids = []
    scores = []
    for detection in msg.detections:
        if detection.results:
            class_ids.append(str(detection.results[0].hypothesis.class_id))
            scores.append(str(float(detection.results[0].hypothesis.score)))
    writer.writerow(
        [
            bag_time_ns,
            stamp_to_ns(msg.header.stamp),
            msg.header.frame_id,
            len(msg.detections),
            ";".join(class_ids),
            ";".join(scores),
        ]
    )


def export_obstacle_mask_stats(writer, bag_time_ns: int, msg):
    pixels = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    nonzero_count = int(np.count_nonzero(pixels))
    mean_value = float(pixels.mean()) if pixels.size > 0 else float("nan")
    writer.writerow(
        [
            bag_time_ns,
            stamp_to_ns(msg.header.stamp),
            msg.header.frame_id,
            int(msg.height),
            int(msg.width),
            msg.encoding,
            int(msg.step),
            int(pixels.size),
            nonzero_count,
            mean_value,
        ]
    )


def build_reader(bag_uri: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def load_pose_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if df.empty:
        return df
    df["time_ns"] = df["stamp_ns"].where(df["stamp_ns"] > 0, df["bag_time_ns"])
    df["publish_lag_ms"] = 1e-6 * np.maximum(0, df["bag_time_ns"] - df["stamp_ns"])
    return df.sort_values("time_ns").reset_index(drop=True)


def path_length_m(df: pd.DataFrame) -> float:
    if len(df) < 2:
        return 0.0
    dx = np.diff(df["x"].to_numpy(dtype=float))
    dy = np.diff(df["y"].to_numpy(dtype=float))
    return float(np.sum(np.hypot(dx, dy)))


def sampled_indices(count: int, target_count: int) -> np.ndarray:
    if count <= 0 or target_count <= 0:
        return np.asarray([], dtype=int)
    if count <= target_count:
        return np.arange(count, dtype=int)
    return np.unique(np.linspace(0, count - 1, num=target_count, dtype=int))


def plot_trajectory_sequence(ax, df: pd.DataFrame, label: str, color: str, span_m: float):
    x = df["x"].to_numpy(dtype=float)
    y = df["y"].to_numpy(dtype=float)
    time_ns = df["time_ns"].to_numpy(dtype=np.int64)
    if x.size == 0:
        return

    ax.plot(x, y, label=label, linewidth=1.8, color=color, alpha=0.9)
    ax.scatter(x[0], y[0], s=44, marker="o", facecolors="white", edgecolors=color, linewidths=1.6, zorder=4)
    ax.scatter(x[-1], y[-1], s=56, marker="X", color=color, linewidths=1.0, zorder=4)

    checkpoint_idx = sampled_indices(len(x), min(12, len(x)))
    if checkpoint_idx.size > 0:
        ax.scatter(x[checkpoint_idx], y[checkpoint_idx], s=22, color=color, alpha=0.95, zorder=3)
        label_offset = 0.012 * span_m
        t0_ns = int(time_ns[0])
        for point_idx in checkpoint_idx:
            elapsed_s = 1e-9 * float(int(time_ns[point_idx]) - t0_ns)
            ax.text(
                x[point_idx] + label_offset,
                y[point_idx] + label_offset,
                f"{elapsed_s:.0f}s",
                color=color,
                fontsize=8,
                bbox={"boxstyle": "round,pad=0.15", "facecolor": "white", "alpha": 0.7, "edgecolor": "none"},
                zorder=5,
            )


def pairwise_metrics(left_key: str, right_key: str, left_df: pd.DataFrame, right_df: pd.DataFrame, tol_ns: int):
    if left_df.empty or right_df.empty:
        return None, None

    if len(left_df) <= len(right_df):
        ref_key, ref_df = left_key, left_df
        cmp_key, cmp_df = right_key, right_df
    else:
        ref_key, ref_df = right_key, right_df
        cmp_key, cmp_df = left_key, left_df

    ref = ref_df[["time_ns", "x", "y", "yaw_rad"]].copy()
    cmp = cmp_df[["time_ns", "x", "y", "yaw_rad"]].copy().rename(
        columns={
            "time_ns": "other_time_ns",
            "x": "other_x",
            "y": "other_y",
            "yaw_rad": "other_yaw_rad",
        }
    )
    merged = pd.merge_asof(
        ref.sort_values("time_ns"),
        cmp.sort_values("other_time_ns"),
        left_on="time_ns",
        right_on="other_time_ns",
        direction="nearest",
        tolerance=tol_ns,
    ).dropna()
    if merged.empty:
        return None, None

    merged["dt_ms"] = 1e-6 * np.abs(merged["time_ns"] - merged["other_time_ns"])
    merged["xy_error_m"] = np.hypot(merged["x"] - merged["other_x"], merged["y"] - merged["other_y"])
    merged["yaw_error_deg"] = np.degrees(
        np.abs(wrap_to_pi(merged["yaw_rad"].to_numpy() - merged["other_yaw_rad"].to_numpy()))
    )
    merged["reference_pose"] = ref_key
    merged["compared_pose"] = cmp_key

    metrics = {
        "pose_a": left_key,
        "pose_b": right_key,
        "reference_pose": ref_key,
        "compared_pose": cmp_key,
        "sync_count": int(len(merged)),
        "mean_dt_ms": float(merged["dt_ms"].mean()),
        "max_dt_ms": float(merged["dt_ms"].max()),
        "mean_xy_error_m": float(merged["xy_error_m"].mean()),
        "rmse_xy_error_m": float(np.sqrt(np.mean(np.square(merged["xy_error_m"])))),
        "max_xy_error_m": float(merged["xy_error_m"].max()),
        "mean_yaw_error_deg": float(merged["yaw_error_deg"].mean()),
        "rmse_yaw_error_deg": float(np.sqrt(np.mean(np.square(merged["yaw_error_deg"])))),
        "max_yaw_error_deg": float(merged["yaw_error_deg"].max()),
    }
    return metrics, merged


def comparison_vs_uav(all_pose_frames, tol_ns: int):
    if "uav" not in all_pose_frames or all_pose_frames["uav"].empty:
        return pd.DataFrame()

    uav_df = all_pose_frames["uav"]
    base = uav_df[["time_ns", "bag_time_ns", "stamp_ns", "x", "y", "yaw_rad"]].copy().rename(
        columns={
            "x": "uav_x",
            "y": "uav_y",
            "yaw_rad": "uav_yaw_rad",
        }
    )

    for key in ("odom_only", "icp", "scan_match"):
        if key not in all_pose_frames or all_pose_frames[key].empty:
            return pd.DataFrame()
        other = all_pose_frames[key][["time_ns", "x", "y", "yaw_rad"]].copy().rename(
            columns={
                "time_ns": f"{key}_time_ns",
                "x": f"{key}_x",
                "y": f"{key}_y",
                "yaw_rad": f"{key}_yaw_rad",
            }
        )
        base = pd.merge_asof(
            base.sort_values("time_ns"),
            other.sort_values(f"{key}_time_ns"),
            left_on="time_ns",
            right_on=f"{key}_time_ns",
            direction="nearest",
            tolerance=tol_ns,
        )
        if f"{key}_x" in base.columns:
            base[f"{key}_dt_ms"] = 1e-6 * np.abs(base["time_ns"] - base[f"{key}_time_ns"])
            base[f"{key}_xy_error_m"] = np.hypot(base[f"{key}_x"] - base["uav_x"], base[f"{key}_y"] - base["uav_y"])
            base[f"{key}_yaw_error_deg"] = np.degrees(
                np.abs(wrap_to_pi(base[f"{key}_yaw_rad"].to_numpy() - base["uav_yaw_rad"].to_numpy()))
            )

    return base.dropna().reset_index(drop=True)


def save_metrics_csv(metrics_rows, path: Path):
    if not metrics_rows:
        return
    pd.DataFrame(metrics_rows).to_csv(path, index=False)


def main():
    parser = argparse.ArgumentParser(description="Export and analyze a multi-pose comparison bag.")
    parser.add_argument("bag_uri", help="Path to the ROS 2 bag directory")
    parser.add_argument(
        "-o",
        "--output-dir",
        help="Directory where CSVs and analysis artifacts will be written",
    )
    parser.add_argument(
        "--sync-tolerance-ms",
        type=float,
        default=250.0,
        help="Nearest-neighbor synchronization tolerance for pairwise pose comparison",
    )
    args = parser.parse_args()

    bag_uri = Path(args.bag_uri).resolve()
    if args.output_dir:
        output_dir = Path(args.output_dir).resolve()
    else:
        output_dir = bag_uri.parent / "analysis" / f"{bag_uri.name}_analysis"
    export_dir = output_dir / "csv"
    plot_dir = output_dir / "plots"
    ensure_dir(export_dir)
    ensure_dir(plot_dir)

    reader = build_reader(str(bag_uri))
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    missing_required = [topic for topic in CORE_POSE_TOPICS if topic not in topic_types]
    if missing_required:
        raise RuntimeError(
            "Bag is missing required comparison topics: "
            + ", ".join(missing_required)
        )

    all_topics = {}
    all_topics.update(CORE_POSE_TOPICS)
    all_topics.update(OPTIONAL_COMPARISON_POSE_TOPICS)
    all_topics.update(OPTIONAL_POSE_TOPICS)
    all_topics.update(OPTIONAL_CONTEXT_TOPICS)

    message_types = {
        topic: get_message(topic_types[topic])
        for topic in all_topics
        if topic in topic_types
    }

    handles = []
    pose_writers = {}
    pose_files = {}
    pose_topics = {}
    pose_topics.update(CORE_POSE_TOPICS)
    pose_topics.update(OPTIONAL_COMPARISON_POSE_TOPICS)
    pose_topics.update(OPTIONAL_POSE_TOPICS)
    for topic, key in pose_topics.items():
        if topic not in message_types:
            continue
        path = export_dir / f"{key}.csv"
        pose_files[key] = path
        handle, writer = init_csv(
            path,
            ["bag_time_ns", "stamp_ns", "frame_id", "x", "y", "z", "qx", "qy", "qz", "qw", "yaw_rad"],
        )
        handles.append(handle)
        pose_writers[topic] = writer

    scan_handle = scan_writer = None
    if "/ugv/scan" in message_types:
        scan_handle, scan_writer = init_csv(
            export_dir / "scan_stats.csv",
            ["bag_time_ns", "stamp_ns", "frame_id", "num_ranges", "num_valid_ranges", "min_valid_range", "max_valid_range", "mean_valid_range"],
        )
        handles.append(scan_handle)

    map_handle = map_writer = None
    if "/map" in message_types:
        map_handle, map_writer = init_csv(
            export_dir / "map_stats.csv",
            ["bag_time_ns", "stamp_ns", "frame_id", "width", "height", "resolution", "origin_x", "origin_y", "occupied_count", "free_count", "unknown_count"],
        )
        handles.append(map_handle)

    det_handle = det_writer = None
    if "/uav/detection_boxes" in message_types:
        det_handle, det_writer = init_csv(
            export_dir / "detection_stats.csv",
            ["bag_time_ns", "stamp_ns", "frame_id", "num_detections", "class_ids", "scores"],
        )
        handles.append(det_handle)

    obstacle_mask_handle = obstacle_mask_writer = None
    if "/uav/obstacle_mask" in message_types:
        obstacle_mask_handle, obstacle_mask_writer = init_csv(
            export_dir / "obstacle_mask_stats.csv",
            [
                "bag_time_ns",
                "stamp_ns",
                "frame_id",
                "height",
                "width",
                "encoding",
                "step",
                "num_pixels",
                "nonzero_pixels",
                "mean_pixel_value",
            ],
        )
        handles.append(obstacle_mask_handle)

    clock_handle = clock_writer = None
    if "/clock" in message_types:
        clock_handle, clock_writer = init_csv(export_dir / "clock.csv", ["bag_time_ns", "clock_ns"])
        handles.append(clock_handle)

    topic_counts = {}
    try:
        while reader.has_next():
            topic, data, bag_time_ns = reader.read_next()
            if topic not in message_types:
                continue
            msg = deserialize_message(data, message_types[topic])
            topic_counts[topic] = topic_counts.get(topic, 0) + 1

            if topic in pose_writers:
                write_pose_row(pose_writers[topic], bag_time_ns, msg)
            elif topic == "/ugv/scan":
                export_scan_stats(scan_writer, bag_time_ns, msg)
            elif topic == "/map":
                export_map_stats(map_writer, bag_time_ns, msg)
            elif topic == "/uav/detection_boxes":
                export_detection_stats(det_writer, bag_time_ns, msg)
            elif topic == "/uav/obstacle_mask":
                export_obstacle_mask_stats(obstacle_mask_writer, bag_time_ns, msg)
            elif topic == "/clock":
                clock_writer.writerow([bag_time_ns, stamp_to_ns(msg.clock)])
    finally:
        for handle in handles:
            handle.close()

    pose_frames = {key: load_pose_csv(path) for key, path in pose_files.items()}
    core_frames = {key: pose_frames[key] for key in CORE_POSE_TOPICS.values()}
    empty_required = [key for key, df in core_frames.items() if df.empty]
    if empty_required:
        raise RuntimeError("Required pose topics were recorded but produced no messages: " + ", ".join(empty_required))

    comparison_frames = dict(core_frames)
    uav_compare_df = pose_frames.get("uav")
    if uav_compare_df is not None and not uav_compare_df.empty:
        comparison_frames["uav"] = uav_compare_df

    tol_ns = int(args.sync_tolerance_ms * 1_000_000.0)
    pairwise_rows = []
    pairwise_exports = []
    for left_key, right_key in combinations(comparison_frames.keys(), 2):
        metrics, merged = pairwise_metrics(left_key, right_key, comparison_frames[left_key], comparison_frames[right_key], tol_ns)
        if metrics is None:
            continue
        pairwise_rows.append(metrics)
        merged.to_csv(export_dir / f"pairwise_{left_key}_vs_{right_key}.csv", index=False)
        pairwise_exports.append((left_key, right_key))

    uav_comparison = comparison_vs_uav(comparison_frames, tol_ns)
    if not uav_comparison.empty:
        uav_comparison.to_csv(export_dir / "comparison_vs_uav.csv", index=False)

    pose_summary_rows = []
    for key, df in comparison_frames.items():
        duration_sec = 0.0
        if len(df) > 1:
            duration_sec = float((df["time_ns"].iloc[-1] - df["time_ns"].iloc[0]) * 1e-9)
        pose_summary_rows.append(
            {
                "pose": key,
                "sample_count": int(len(df)),
                "duration_sec": duration_sec,
                "path_length_m": path_length_m(df),
                "start_x": float(df["x"].iloc[0]),
                "start_y": float(df["y"].iloc[0]),
                "end_x": float(df["x"].iloc[-1]),
                "end_y": float(df["y"].iloc[-1]),
            }
        )
    pose_summary = pd.DataFrame(pose_summary_rows)
    pose_summary.to_csv(output_dir / "pose_summary.csv", index=False)
    save_metrics_csv(pairwise_rows, output_dir / "pairwise_metrics.csv")

    context_summary = {}
    scan_stats_path = export_dir / "scan_stats.csv"
    if scan_stats_path.exists():
        scan_df = pd.read_csv(scan_stats_path)
        if not scan_df.empty:
            context_summary["scan_valid_ranges_mean"] = float(scan_df["num_valid_ranges"].mean())
            context_summary["scan_valid_ranges_min"] = int(scan_df["num_valid_ranges"].min())
            context_summary["scan_valid_ranges_max"] = int(scan_df["num_valid_ranges"].max())

    map_stats_path = export_dir / "map_stats.csv"
    if map_stats_path.exists():
        map_df = pd.read_csv(map_stats_path)
        if not map_df.empty:
            context_summary["map_occupied_mean"] = float(map_df["occupied_count"].mean())
            context_summary["map_occupied_min"] = int(map_df["occupied_count"].min())
            context_summary["map_occupied_max"] = int(map_df["occupied_count"].max())

    det_stats_path = export_dir / "detection_stats.csv"
    if det_stats_path.exists():
        det_df = pd.read_csv(det_stats_path)
        if not det_df.empty:
            context_summary["detection_count_mean"] = float(det_df["num_detections"].mean())
            context_summary["detection_count_max"] = int(det_df["num_detections"].max())

    if uav_compare_df is not None and not uav_compare_df.empty:
        context_summary["uav_compare_publish_lag_mean_ms"] = float(uav_compare_df["publish_lag_ms"].mean())
        context_summary["uav_compare_publish_lag_max_ms"] = float(uav_compare_df["publish_lag_ms"].max())

    uav_source_df = pose_frames.get("uav_source")
    if uav_source_df is not None and not uav_source_df.empty:
        context_summary["uav_source_publish_lag_mean_ms"] = float(uav_source_df["publish_lag_ms"].mean())
        context_summary["uav_source_publish_lag_max_ms"] = float(uav_source_df["publish_lag_ms"].max())

    obstacle_mask_stats_path = export_dir / "obstacle_mask_stats.csv"
    if obstacle_mask_stats_path.exists():
        obstacle_mask_df = pd.read_csv(obstacle_mask_stats_path)
        if not obstacle_mask_df.empty:
            context_summary["obstacle_mask_nonzero_mean"] = float(obstacle_mask_df["nonzero_pixels"].mean())
            context_summary["obstacle_mask_nonzero_min"] = int(obstacle_mask_df["nonzero_pixels"].min())
            context_summary["obstacle_mask_nonzero_max"] = int(obstacle_mask_df["nonzero_pixels"].max())

    trajectory_colors = {
        "odom_only": "#1f77b4",
        "icp": "#d62728",
        "scan_match": "#2ca02c",
        "uav": "#ff7f0e",
    }
    all_x = np.concatenate([df["x"].to_numpy(dtype=float) for df in comparison_frames.values() if not df.empty])
    all_y = np.concatenate([df["y"].to_numpy(dtype=float) for df in comparison_frames.values() if not df.empty])
    span_m = float(max(all_x.max() - all_x.min(), all_y.max() - all_y.min(), 1.0))

    fig, ax = plt.subplots(figsize=(10, 8.5))
    for key, label in (
        ("odom_only", "Odom only"),
        ("icp", "ICP"),
        ("scan_match", "Scan matcher"),
    ):
        plot_trajectory_sequence(ax, comparison_frames[key], label, trajectory_colors[key], span_m)
    if "uav" in comparison_frames:
        plot_trajectory_sequence(
            ax,
            comparison_frames["uav"],
            "UAV pose",
            trajectory_colors["uav"],
            span_m,
        )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory Comparison With Time-Ordered Positions")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.text(
        0.02,
        0.02,
        "o = start, X = end, labeled dots = elapsed time along the trajectory",
        transform=ax.transAxes,
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "alpha": 0.85, "edgecolor": "0.8"},
    )
    fig.tight_layout()
    fig.savefig(plot_dir / "trajectories.png", dpi=180)
    plt.close(fig)

    if not uav_comparison.empty:
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        time_s = 1e-9 * (uav_comparison["time_ns"] - uav_comparison["time_ns"].iloc[0])
        for key, label in (
            ("odom_only", "Odom only"),
            ("icp", "ICP"),
            ("scan_match", "Scan matcher"),
        ):
            axes[0].plot(time_s.to_numpy(), uav_comparison[f"{key}_xy_error_m"].to_numpy(), label=label)
            axes[1].plot(time_s.to_numpy(), uav_comparison[f"{key}_yaw_error_deg"].to_numpy(), label=label)
        axes[0].set_ylabel("XY error vs UAV [m]")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[1].set_ylabel("Yaw error vs UAV [deg]")
        axes[1].set_xlabel("time [s]")
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(plot_dir / "errors_vs_uav.png", dpi=180)
        plt.close(fig)

    if pairwise_rows:
        pairwise_df = pd.DataFrame(pairwise_rows)
        plt.figure(figsize=(10, 5))
        labels = [f"{row['pose_a']} vs {row['pose_b']}" for row in pairwise_rows]
        plt.bar(labels, pairwise_df["rmse_xy_error_m"].to_numpy())
        plt.ylabel("RMSE XY error [m]")
        plt.title("Pairwise RMSE")
        plt.xticks(rotation=30, ha="right")
        plt.tight_layout()
        plt.savefig(plot_dir / "pairwise_rmse_xy.png", dpi=180)
        plt.close()

    context_plotted = False
    fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=False)
    axis_index = 0
    if scan_stats_path.exists():
        scan_df = pd.read_csv(scan_stats_path)
        if not scan_df.empty:
            t = 1e-9 * (scan_df["bag_time_ns"] - scan_df["bag_time_ns"].iloc[0])
            axes[axis_index].plot(t.to_numpy(), scan_df["num_valid_ranges"].to_numpy())
            axes[axis_index].set_title("Laser Scan Valid Beams")
            axes[axis_index].set_ylabel("count")
            axes[axis_index].grid(True, alpha=0.3)
            axis_index += 1
            context_plotted = True
    if map_stats_path.exists():
        map_df = pd.read_csv(map_stats_path)
        if not map_df.empty and axis_index < 3:
            t = 1e-9 * (map_df["bag_time_ns"] - map_df["bag_time_ns"].iloc[0])
            axes[axis_index].plot(t.to_numpy(), map_df["occupied_count"].to_numpy())
            axes[axis_index].set_title("Map Occupied Cells")
            axes[axis_index].set_ylabel("count")
            axes[axis_index].grid(True, alpha=0.3)
            axis_index += 1
            context_plotted = True
    if obstacle_mask_stats_path.exists():
        obstacle_mask_df = pd.read_csv(obstacle_mask_stats_path)
        if not obstacle_mask_df.empty and axis_index < 5:
            t = 1e-9 * (obstacle_mask_df["bag_time_ns"] - obstacle_mask_df["bag_time_ns"].iloc[0])
            axes[axis_index].plot(t.to_numpy(), obstacle_mask_df["nonzero_pixels"].to_numpy())
            axes[axis_index].set_title("Obstacle Mask Nonzero Pixels")
            axes[axis_index].set_ylabel("count")
            axes[axis_index].grid(True, alpha=0.3)
            axis_index += 1
            context_plotted = True
    if uav_compare_df is not None and not uav_compare_df.empty and axis_index < 5:
        t = 1e-9 * (uav_compare_df["bag_time_ns"] - uav_compare_df["bag_time_ns"].iloc[0])
        axes[axis_index].plot(t.to_numpy(), uav_compare_df["publish_lag_ms"].to_numpy())
        axes[axis_index].set_title("UAV Pose Publish Lag")
        axes[axis_index].set_ylabel("ms")
        axes[axis_index].grid(True, alpha=0.3)
        axis_index += 1
        context_plotted = True
    if det_stats_path.exists():
        det_df = pd.read_csv(det_stats_path)
        if not det_df.empty and axis_index < 5:
            t = 1e-9 * (det_df["bag_time_ns"] - det_df["bag_time_ns"].iloc[0])
            axes[axis_index].stem(t.to_numpy(), det_df["num_detections"].to_numpy(), basefmt=" ")
            axes[axis_index].set_title("UAV Detections")
            axes[axis_index].set_ylabel("count")
            axes[axis_index].grid(True, alpha=0.3)
            context_plotted = True
    if context_plotted:
        axes[min(axis_index, 4)].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(plot_dir / "context_overview.png", dpi=180)
    plt.close(fig)

    metrics = {
        "bag_uri": str(bag_uri),
        "output_dir": str(output_dir),
        "sync_tolerance_ms": float(args.sync_tolerance_ms),
        "topic_counts": topic_counts,
        "available_comparison_poses": list(comparison_frames.keys()),
        "pose_summary": pose_summary_rows,
        "pairwise_metrics": pairwise_rows,
        "context_summary": context_summary,
        "pairwise_exports": [f"{left} vs {right}" for left, right in pairwise_exports],
    }
    with (output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)

    report_lines = [
        "Pose Comparison Bag Analysis",
        f"Bag: {bag_uri}",
        f"Output: {output_dir}",
        f"Sync tolerance: {args.sync_tolerance_ms:.1f} ms",
        "",
        "Pose summary:",
    ]
    for row in pose_summary_rows:
        report_lines.append(
            f"- {row['pose']}: {row['sample_count']} samples, {row['duration_sec']:.2f}s, path {row['path_length_m']:.2f} m"
        )
    if "uav" not in comparison_frames:
        report_lines.append("- uav: no usable messages were recorded on /ugv/compare/uav_pose")
    if pairwise_rows:
        report_lines.append("")
        report_lines.append("Pairwise XY RMSE:")
        for row in pairwise_rows:
            report_lines.append(
                f"- {row['pose_a']} vs {row['pose_b']}: {row['rmse_xy_error_m']:.3f} m "
                f"(yaw RMSE {row['rmse_yaw_error_deg']:.2f} deg, {row['sync_count']} synced samples)"
            )
    if not uav_comparison.empty:
        report_lines.append("")
        report_lines.append("Errors vs UAV:")
        for key in ("odom_only", "icp", "scan_match"):
            report_lines.append(
                f"- {key}: XY mean {uav_comparison[f'{key}_xy_error_m'].mean():.3f} m, "
                f"XY RMSE {np.sqrt(np.mean(np.square(uav_comparison[f'{key}_xy_error_m']))):.3f} m"
            )
    if context_summary:
        report_lines.append("")
        report_lines.append("Context summary:")
        for key, value in context_summary.items():
            report_lines.append(f"- {key}: {value}")

    (output_dir / "report.txt").write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    print("\n".join(report_lines))


if __name__ == "__main__":
    main()
