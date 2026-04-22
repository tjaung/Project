#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import associate  # noqa: E402


def normalize_timestamps(entries):
    if not entries:
        return entries

    max_abs_time = max(abs(float(stamp)) for stamp in entries.keys())
    if max_abs_time < 1e11:
        return entries

    normalized = {}
    for stamp, values in entries.items():
        normalized[float(stamp) * 1e-9] = values
    return normalized


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.eye(3)
    qw, qx, qy, qz = q / norm
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def pose_to_matrix(entry):
    tx, ty, tz, qx, qy, qz, qw = [float(v) for v in entry[:7]]
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    transform[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
    return transform


def align_sim3(model_xyz, data_xyz):
    model_zerocentered = model_xyz - model_xyz.mean(axis=1, keepdims=True)
    data_zerocentered = data_xyz - data_xyz.mean(axis=1, keepdims=True)

    w = np.zeros((3, 3), dtype=np.float64)
    for col in range(model_xyz.shape[1]):
        w += np.outer(model_zerocentered[:, col], data_zerocentered[:, col])

    u, _, vh = np.linalg.svd(w.T)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vh) < 0:
        s[2, 2] = -1
    rot = u @ s @ vh

    rot_model = rot @ model_zerocentered
    dots = 0.0
    norms = 0.0
    for col in range(data_zerocentered.shape[1]):
        dots += float(data_zerocentered[:, col].T @ rot_model[:, col])
        norms += float(model_zerocentered[:, col].T @ model_zerocentered[:, col])

    scale = dots / norms if norms > 0.0 else 1.0
    trans = data_xyz.mean(axis=1, keepdims=True) - scale * rot @ model_xyz.mean(axis=1, keepdims=True)
    return rot, trans, scale


def matrix_rotation_angle(rotation):
    trace = np.trace(rotation)
    cos_angle = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    return math.acos(cos_angle)


def find_delta_index(times, start_idx, delta_sec, tolerance_sec):
    target = times[start_idx] + delta_sec
    best_idx = -1
    best_diff = float("inf")
    for idx in range(start_idx + 1, len(times)):
        diff = abs(times[idx] - target)
        if diff < best_diff:
            best_diff = diff
            best_idx = idx
        if times[idx] > target and diff > best_diff:
            break
    if best_idx < 0 or best_diff > tolerance_sec:
        return -1
    return best_idx


def main():
    parser = argparse.ArgumentParser(description="Compute TUM ATE and RPE metrics for a trajectory.")
    parser.add_argument("groundtruth", help="Ground truth trajectory file in TUM format.")
    parser.add_argument("estimated", help="Estimated trajectory file in TUM format.")
    parser.add_argument("--offset", type=float, default=0.0, help="Timestamp offset applied to estimated trajectory.")
    parser.add_argument("--max-difference", type=float, default=0.02, help="Maximum timestamp difference for association.")
    parser.add_argument("--rpe-delta-sec", type=float, default=1.0, help="Relative pose evaluation interval in seconds.")
    parser.add_argument("--rpe-tolerance-sec", type=float, default=0.05, help="Allowed timestamp tolerance for RPE pairing.")
    parser.add_argument("--tracking-time-sec", type=float, default=None, help="Mean tracking time per frame in seconds.")
    parser.add_argument("--median-tracking-time-sec", type=float, default=None, help="Median tracking time per frame in seconds.")
    parser.add_argument("--dataset-name", default="", help="Optional dataset name stored in CSV output.")
    parser.add_argument("--csv", default="", help="Optional CSV file to append the computed metrics to.")
    args = parser.parse_args()

    gt_list = normalize_timestamps(associate.read_file_list(args.groundtruth, remove_bounds=False))
    est_list = normalize_timestamps(associate.read_file_list(args.estimated, remove_bounds=False))
    matches = associate.associate(gt_list, est_list, args.offset, args.max_difference)
    if len(matches) < 2:
        raise SystemExit("Not enough associated trajectory pairs to evaluate.")

    gt_xyz = np.array([[float(v) for v in gt_list[a][0:3]] for a, _ in matches], dtype=np.float64).T
    est_xyz = np.array([[float(v) for v in est_list[b][0:3]] for _, b in matches], dtype=np.float64).T
    rot, trans, scale = align_sim3(est_xyz, gt_xyz)

    gt_times = []
    gt_poses = []
    est_poses_aligned = []
    ate_errors = []

    for gt_stamp, est_stamp in matches:
        gt_pose = pose_to_matrix(gt_list[gt_stamp])
        est_pose = pose_to_matrix(est_list[est_stamp])
        aligned_pose = np.eye(4, dtype=np.float64)
        aligned_pose[:3, :3] = rot @ est_pose[:3, :3]
        aligned_pose[:3, 3:4] = scale * rot @ est_pose[:3, 3:4] + trans

        gt_times.append(gt_stamp)
        gt_poses.append(gt_pose)
        est_poses_aligned.append(aligned_pose)
        ate_errors.append(np.linalg.norm(aligned_pose[:3, 3] - gt_pose[:3, 3]))

    ate_errors = np.array(ate_errors, dtype=np.float64)

    rpe_trans_errors = []
    rpe_rot_errors = []
    rpe_percent_errors = []
    for idx in range(len(gt_times) - 1):
        jdx = find_delta_index(gt_times, idx, args.rpe_delta_sec, args.rpe_tolerance_sec)
        if jdx < 0:
            continue

        gt_rel = np.linalg.inv(gt_poses[idx]) @ gt_poses[jdx]
        est_rel = np.linalg.inv(est_poses_aligned[idx]) @ est_poses_aligned[jdx]
        rel_error = np.linalg.inv(gt_rel) @ est_rel

        trans_err = np.linalg.norm(rel_error[:3, 3])
        gt_rel_dist = np.linalg.norm(gt_rel[:3, 3])
        rot_err = matrix_rotation_angle(rel_error[:3, :3])

        rpe_trans_errors.append(trans_err)
        rpe_rot_errors.append(rot_err)
        if gt_rel_dist > 1e-9:
            rpe_percent_errors.append(100.0 * trans_err / gt_rel_dist)

    if not rpe_trans_errors:
        raise SystemExit("Not enough associated pairs to compute RPE at the requested interval.")

    rpe_trans_errors = np.array(rpe_trans_errors, dtype=np.float64)
    rpe_rot_errors = np.array(rpe_rot_errors, dtype=np.float64)
    rpe_percent_errors = np.array(rpe_percent_errors, dtype=np.float64)

    metrics = {
        "dataset": args.dataset_name,
        "matched_pairs": len(matches),
        "ate_rmse_m": math.sqrt(np.mean(np.square(ate_errors))),
        "ate_mean_m": float(np.mean(ate_errors)),
        "ate_median_m": float(np.median(ate_errors)),
        "ate_scale": scale,
        "rpe_interval_sec": args.rpe_delta_sec,
        "rpe_pairs": len(rpe_trans_errors),
        "rpe_rmse_m": math.sqrt(np.mean(np.square(rpe_trans_errors))),
        "rpe_mean_m": float(np.mean(rpe_trans_errors)),
        "rpe_median_m": float(np.median(rpe_trans_errors)),
        "rpe_rmse_percent": math.sqrt(np.mean(np.square(rpe_percent_errors))),
        "rpe_mean_percent": float(np.mean(rpe_percent_errors)),
        "rpe_median_percent": float(np.median(rpe_percent_errors)),
        "rpe_rmse_deg": math.degrees(math.sqrt(np.mean(np.square(rpe_rot_errors)))),
    }

    print("Trajectory Metrics")
    print(f"matched_pairs: {metrics['matched_pairs']}")
    print(f"ate_rmse_m: {metrics['ate_rmse_m']:.6f}")
    print(f"ate_mean_m: {metrics['ate_mean_m']:.6f}")
    print(f"ate_median_m: {metrics['ate_median_m']:.6f}")
    print(f"ate_scale: {metrics['ate_scale']:.6f}")
    print(f"rpe_interval_sec: {metrics['rpe_interval_sec']:.3f}")
    print(f"rpe_pairs: {metrics['rpe_pairs']}")
    print(f"rpe_rmse_m: {metrics['rpe_rmse_m']:.6f}")
    print(f"rpe_mean_m: {metrics['rpe_mean_m']:.6f}")
    print(f"rpe_median_m: {metrics['rpe_median_m']:.6f}")
    print(f"rpe_rmse_percent: {metrics['rpe_rmse_percent']:.6f}")
    print(f"rpe_mean_percent: {metrics['rpe_mean_percent']:.6f}")
    print(f"rpe_median_percent: {metrics['rpe_median_percent']:.6f}")
    print(f"rpe_rmse_deg: {metrics['rpe_rmse_deg']:.6f}")
    if args.tracking_time_sec is not None:
        metrics["tracking_time_mean_ms"] = args.tracking_time_sec * 1000.0
        print(f"tracking_time_mean_ms: {metrics['tracking_time_mean_ms']:.6f}")
    if args.median_tracking_time_sec is not None:
        metrics["tracking_time_median_ms"] = args.median_tracking_time_sec * 1000.0
        print(f"tracking_time_median_ms: {metrics['tracking_time_median_ms']:.6f}")

    if args.csv:
        csv_path = os.path.abspath(args.csv)
        csv_dir = os.path.dirname(csv_path)
        if csv_dir and not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        fieldnames = [
            "dataset",
            "matched_pairs",
            "ate_rmse_m",
            "ate_mean_m",
            "ate_median_m",
            "ate_scale",
            "rpe_interval_sec",
            "rpe_pairs",
            "rpe_rmse_m",
            "rpe_mean_m",
            "rpe_median_m",
            "rpe_rmse_percent",
            "rpe_mean_percent",
            "rpe_median_percent",
            "rpe_rmse_deg",
            "tracking_time_mean_ms",
            "tracking_time_median_ms",
        ]
        write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
        with open(csv_path, "a", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow({key: metrics.get(key, "") for key in fieldnames})
        print(f"metrics_csv: {csv_path}")


if __name__ == "__main__":
    main()
