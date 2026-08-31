from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from .sample_storage import save_json, save_yaml, utc_now_iso
from .transforms import invert_transform, residual_between_transforms, serialize_transform


METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

RESULT_YAML_NAME = "result_handeye.yaml"
RESULT_METRICS_NAME = "result_metrics.json"


def _load_samples(samples_jsonl: Path) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    if not samples_jsonl.exists():
        raise FileNotFoundError(f"Sample log does not exist: {samples_jsonl}")
    with samples_jsonl.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            samples.append(json.loads(line))
    return samples


def _transform_from_serialized(payload: dict[str, Any]) -> np.ndarray:
    return np.asarray(payload["matrix"], dtype=np.float64).reshape(4, 4)


def _result_paths(run_dir: Path) -> tuple[Path, Path]:
    return run_dir / RESULT_YAML_NAME, run_dir / RESULT_METRICS_NAME


def _build_result_payload(
    *,
    run_dir: Path,
    solution: dict[str, Any],
    chosen_method: str,
) -> dict[str, Any]:
    chosen = solution["methods"][chosen_method]
    job_config = {}
    job_config_path = run_dir / "job_config.yaml"
    if job_config_path.exists():
        with job_config_path.open("r", encoding="utf-8") as stream:
            job_config = yaml.safe_load(stream) or {}
    base_frame = str(job_config.get("robot_base_frame", ""))
    ee_frame = str(job_config.get("fk_link_name", ""))
    camera_frame = str(job_config.get("camera_frame", ""))
    return {
        "saved_at": utc_now_iso(),
        "run_directory": str(run_dir),
        "chosen_method": chosen_method,
        "camera_to_gripper": chosen["camera_to_gripper"],
        "gripper_to_camera": chosen["gripper_to_camera"],
        "metrics": chosen["metrics"],
        "num_valid_samples": int(solution["num_valid_samples"]),
        "selection_logic": solution["selection_logic"],
        "calibration_frames": {
            "robot_base_frame": base_frame,
            "end_effector_frame": ee_frame,
            "camera_frame": camera_frame,
            "calibrated_transform": (
                f"{ee_frame} -> {camera_frame}" if job_config.get("eye_in_hand", True)
                else f"{base_frame} -> {camera_frame}"
            ),
        },
    }


def _average_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    accumulator = np.zeros((3, 3), dtype=np.float64)
    for rotation in rotations:
        accumulator += rotation
    u, _, vt = np.linalg.svd(accumulator)
    average = u @ vt
    if np.linalg.det(average) < 0.0:
        u[:, -1] *= -1.0
        average = u @ vt
    return average


def _metrics_for_transform(samples: list[dict[str, Any]], transform: np.ndarray, eye_in_hand: bool) -> dict[str, float]:
    base_to_target_transforms = []
    for sample in samples:
        base_to_ee = _transform_from_serialized(sample["base_to_ee"])
        target_to_camera = _transform_from_serialized(sample["target_to_camera"])
        base_to_target_transforms.append(base_to_ee @ transform @ target_to_camera)

    rotations = [transform[:3, :3] for transform in base_to_target_transforms]
    translations = np.asarray([transform[:3, 3] for transform in base_to_target_transforms], dtype=np.float64)

    mean_rotation = _average_rotation(rotations)
    mean_translation = np.mean(translations, axis=0)
    mean_transform = np.eye(4, dtype=np.float64)
    mean_transform[:3, :3] = mean_rotation
    mean_transform[:3, 3] = mean_translation

    translation_errors = []
    rotation_errors = []
    for transform in base_to_target_transforms:
        residual = residual_between_transforms(mean_transform, transform)
        translation_errors.append(residual.translation_error_m)
        rotation_errors.append(residual.rotation_error_deg)

    translation_rms = float(np.sqrt(np.mean(np.square(translation_errors)))) if translation_errors else 0.0
    rotation_rms = float(np.sqrt(np.mean(np.square(rotation_errors)))) if rotation_errors else 0.0
    score = float(translation_rms * 1000.0 + rotation_rms)

    return {
        "translation_rms_m": translation_rms,
        "rotation_rms_deg": rotation_rms,
        "score": score,
    }


def solve_handeye_from_samples(samples: list[dict[str, Any]], eye_in_hand: bool = True) -> dict[str, Any]:
    valid_samples = [sample for sample in samples if sample.get("valid_sample", False)]
    if len(valid_samples) < 3:
        raise ValueError("Need at least 3 valid samples to solve hand-eye calibration.")

    r_gripper2base = []
    t_gripper2base = []
    r_target2cam = []
    t_target2cam = []

    for sample in valid_samples:
        base_to_ee = _transform_from_serialized(sample["base_to_ee"])
        target_to_camera = _transform_from_serialized(sample["target_to_camera"])
        # ROS TF lookup(base, ee) returns base_T_ee.  OpenCV's
        # R_gripper2base is the same coordinate mapping (EE coordinates into
        # the robot base), so do not invert this matrix.
        robot_pose = base_to_ee
        r_gripper2base.append(robot_pose[:3, :3])
        t_gripper2base.append(robot_pose[:3, 3].reshape(3, 1))
        r_target2cam.append(target_to_camera[:3, :3])
        t_target2cam.append(target_to_camera[:3, 3].reshape(3, 1))

    method_results: dict[str, Any] = {}
    best_method = ""
    best_score = None

    for method_name, method_id in METHODS.items():
        rotation, translation = cv2.calibrateHandEye(
            r_gripper2base,
            t_gripper2base,
            r_target2cam,
            t_target2cam,
            method=method_id,
        )
        solved_transform = np.eye(4, dtype=np.float64)
        solved_transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        solved_transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
        if eye_in_hand:
            camera_to_robot = solved_transform
            metrics = _metrics_for_transform(valid_samples, camera_to_robot, eye_in_hand)
        else:
            # For eye-to-hand (fixed camera, target on EE), OpenCV already
            # returns camera_T_base. Do not reinterpret it as EE_T_target.
            camera_to_robot = solved_transform
            ee_to_target_samples = []
            for sample in valid_samples:
                base_to_ee = _transform_from_serialized(sample["base_to_ee"])
                target_to_camera = _transform_from_serialized(sample["target_to_camera"])
                # camera_to_robot is base_T_camera and target_to_camera is
                # camera_T_target. The target is on the moving EE, so the
                # invariant is EE_T_target, not base_T_target.
                ee_to_target_samples.append(
                    invert_transform(base_to_ee) @ camera_to_robot @ target_to_camera
                )
            mean_rotation = _average_rotation([x[:3, :3] for x in ee_to_target_samples])
            mean_translation = np.mean([x[:3, 3] for x in ee_to_target_samples], axis=0)
            te = [float(np.linalg.norm(x[:3, 3] - mean_translation)) for x in ee_to_target_samples]
            re = [residual_between_transforms(
                np.block([[mean_rotation, mean_translation.reshape(3, 1)],
                           [np.zeros((1, 3)), np.ones((1, 1))]]), x
            ).rotation_error_deg for x in ee_to_target_samples]
            metrics = {
                "translation_rms_m": float(np.sqrt(np.mean(np.square(te)))),
                "rotation_rms_deg": float(np.sqrt(np.mean(np.square(re)))),
            }
            metrics["score"] = metrics["translation_rms_m"] * 1000.0 + metrics["rotation_rms_deg"]
        method_results[method_name] = {
            "camera_to_gripper": serialize_transform(camera_to_robot),
            "gripper_to_camera": serialize_transform(invert_transform(camera_to_robot)),
            "camera_to_base": serialize_transform(camera_to_robot),
            "base_to_camera": serialize_transform(invert_transform(camera_to_robot)),
            "metrics": metrics,
        }
        if best_score is None or metrics["score"] < best_score:
            best_method = method_name
            best_score = metrics["score"]

    return {
        "num_valid_samples": len(valid_samples),
        "selection_logic": "Choose the method with the smallest score = translation_rms_m * 1000 + rotation_rms_deg.",
        "chosen_method": best_method,
        "methods": method_results,
    }


def solve_calibration_run(run_dir: str | Path, eye_in_hand: bool = True) -> dict[str, Any]:
    run_dir = Path(run_dir).expanduser().resolve()
    samples = _load_samples(run_dir / "samples.jsonl")
    solution = solve_handeye_from_samples(samples, eye_in_hand=eye_in_hand)

    result_yaml_path, metrics_json_path = _result_paths(run_dir)
    chosen_method = solution["chosen_method"]
    result_payload = _build_result_payload(
        run_dir=run_dir,
        solution=solution,
        chosen_method=chosen_method,
    )

    save_yaml(result_yaml_path, result_payload)
    save_json(metrics_json_path, solution)
    return {
        "run_directory": str(run_dir),
        "result_handeye_yaml": str(result_yaml_path),
        "result_metrics_json": str(metrics_json_path),
        **solution,
    }


def validate_calibration_run(run_dir: str | Path) -> dict[str, Any]:
    run_dir = Path(run_dir).expanduser().resolve()
    result_payload = load_calibration_result(run_dir)

    samples = _load_samples(run_dir / "samples.jsonl")
    chosen_method = str(result_payload.get("chosen_method", ""))
    camera_to_gripper = _transform_from_serialized(result_payload["camera_to_gripper"])
    metrics = _metrics_for_transform([sample for sample in samples if sample.get("valid_sample", False)], camera_to_gripper)
    return {
        "run_directory": str(run_dir),
        "chosen_method": chosen_method,
        "metrics": metrics,
        "num_samples": sum(1 for sample in samples if sample.get("valid_sample", False)),
    }


def load_calibration_result(run_dir: str | Path) -> dict[str, Any]:
    run_dir = Path(run_dir).expanduser().resolve()
    result_yaml_path, _metrics_json_path = _result_paths(run_dir)
    if not result_yaml_path.exists():
        raise FileNotFoundError(f"Calibration result file does not exist: {result_yaml_path}")

    with result_yaml_path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"Expected a mapping in calibration result file: {result_yaml_path}")

    for key in ("chosen_method", "camera_to_gripper", "gripper_to_camera"):
        if key not in payload:
            raise ValueError(f"Calibration result file is missing '{key}': {result_yaml_path}")

    if "metrics" not in payload:
        payload["metrics"] = {}
    payload["run_directory"] = str(run_dir)
    return payload
