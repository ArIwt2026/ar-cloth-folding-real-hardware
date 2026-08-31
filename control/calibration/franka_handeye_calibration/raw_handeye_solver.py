from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .charuco_pose_estimator import detect_aruco_pose, get_dictionary
from .handeye_solver import solve_calibration_run
from .job_config import CameraIntrinsics
from .transforms import quaternion_to_matrix


def main(args=None):
    ap = argparse.ArgumentParser(description="Solve hand-eye calibration from raw collector samples")
    ap.add_argument("input_dir")
    ap.add_argument("--output-dir", default="")
    ap.add_argument("--marker-size", type=float, default=0.07)
    ap.add_argument("--dictionary", default="DICT_APRILTAG_25h9")
    ap.add_argument("--eye-in-hand", action="store_true")
    ns = ap.parse_args(args)
    source = Path(ns.input_dir).expanduser().resolve()
    out = Path(ns.output_dir).expanduser().resolve() if ns.output_dir else source / "solved"
    out.mkdir(parents=True, exist_ok=True)
    dictionary = get_dictionary(ns.dictionary)
    samples = []
    for meta_path in sorted(source.glob("sample_*.yaml")):
        meta = yaml.safe_load(meta_path.read_text()) or {}
        image_path = Path(meta["image"])
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"SKIP {meta_path.name}: image cannot be read")
            continue
        k = np.asarray(meta["camera_matrix"], dtype=np.float64).reshape(3, 3)
        d = np.asarray(meta["distortion_coefficients"], dtype=np.float64)
        # The collector preserves CameraInfo dimensions; account for any
        # crop in the actual raw image before solving PnP.
        cw, ch = int(meta.get("camera_info_width", image.shape[1])), int(meta.get("camera_info_height", image.shape[0]))
        if (cw, ch) != (image.shape[1], image.shape[0]) and cw == image.shape[1]:
            k[1, 2] -= 0.5 * (ch - image.shape[0])
        intr = CameraIntrinsics(k, d, image.shape[1], image.shape[0], Path("raw_collector"))
        detection = detect_aruco_pose(image, dictionary, intr, ns.marker_size, robust=True)
        if not detection.success or detection.target_to_camera is None:
            print(f"SKIP {meta_path.name}: marker not detected")
            continue
        q = meta["base_to_ee_quaternion_xyzw"]
        b = np.eye(4)
        b[:3, :3] = quaternion_to_matrix(*map(float, q))
        b[:3, 3] = np.asarray(meta["base_to_ee_translation"], dtype=float)
        samples.append({
            "valid_sample": True,
            "base_to_ee": {"matrix": b.tolist()},
            "target_to_camera": {"matrix": detection.target_to_camera.tolist()},
            "marker_ids": [] if detection.marker_ids is None else [int(v) for v in detection.marker_ids.flatten()],
            "raw_image": str(image_path),
            "source_metadata": str(meta_path),
        })
        print(f"OK {meta_path.name}: IDs={samples[-1]['marker_ids']}")
    (out / "samples.jsonl").write_text("".join(json.dumps(s) + "\n" for s in samples))
    print(f"Detected {len(samples)} valid samples; solving in {out}")
    result = solve_calibration_run(out, eye_in_hand=ns.eye_in_hand)
    print(json.dumps({k: result[k] for k in ("result_handeye_yaml", "result_metrics_json", "chosen_method", "num_valid_samples")}, indent=2))


if __name__ == "__main__":
    main()
