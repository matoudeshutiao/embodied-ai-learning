#!/usr/bin/env python3
"""纯 AX=XB 手眼标定：读 charuco_pose_sampler 的 poses.json。

求解只用：
  T_base_link8  （TF 读到）
  T_target2cam  （ChArUco PnP 读到）
真值 T_link8_cam 只在最后做误差比较，不进入求解。

用法：
  python3 scripts/hand_eye_calibrate.py --poses <poses.json>
"""

import argparse
import glob
import itertools
import json
import math
import os
from datetime import datetime

import cv2
import numpy as np

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def inv_T(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def rot_angle_deg(R_a, R_b):
    cos_value = (np.trace(R_a.T @ R_b) - 1.0) / 2.0
    cos_value = float(np.clip(cos_value, -1.0, 1.0))
    return math.degrees(math.acos(cos_value))


def pose_error(T_est, T_truth):
    t_err = 1000.0 * float(
        np.linalg.norm(T_est[:3, 3] - T_truth[:3, 3])
    )
    r_err = rot_angle_deg(T_est[:3, :3], T_truth[:3, :3])
    return t_err, r_err


def target_from_solution(solution):
    rvec = np.asarray(solution["rvec"], dtype=float).reshape(3, 1)
    tvec = np.asarray(solution["tvec"], dtype=float).reshape(3)
    rotation, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = tvec
    return T


def solve(records, method):
    Rg = [np.asarray(r["T_base_link8"])[:3, :3] for r in records]
    tg = [np.asarray(r["T_base_link8"])[:3, 3].reshape(3, 1)
          for r in records]
    Rt = [np.asarray(r["T_target2cam"])[:3, :3] for r in records]
    tt = [np.asarray(r["T_target2cam"])[:3, 3].reshape(3, 1)
          for r in records]
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        Rg, tg, Rt, tt, method=METHODS[method]
    )
    T = np.eye(4)
    T[:3, :3] = R_cam2gripper
    T[:3, 3] = t_cam2gripper.reshape(3)
    return T


def target_consistency(records, T_est):
    """板子不动，T_base_target 应该对所有姿态一致。"""
    mats = []
    for r in records:
        T_bt = (
            np.asarray(r["T_base_link8"])
            @ T_est
            @ np.asarray(r["T_target2cam"])
        )
        mats.append(T_bt)
    ref = mats[0]
    trans = []
    rots = []
    for T in mats[1:]:
        rel = inv_T(ref) @ T
        trans.append(1000.0 * float(np.linalg.norm(rel[:3, 3])))
        rots.append(rot_angle_deg(np.eye(3), rel[:3, :3]))
    if not trans:
        return 0.0, 0.0, 0.0, 0.0
    return (
        float(np.mean(trans)),
        float(np.max(trans)),
        float(np.mean(rots)),
        float(np.max(rots)),
    )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--poses", required=True)
    parser.add_argument("--gap", type=float, default=0.1)
    parsed = parser.parse_args(args)

    with open(parsed.poses, encoding="utf-8") as handle:
        data = json.load(handle)

    truth = np.asarray(data["truth_T_link8_cam"], dtype=float)
    all_records = [
        r for r in data["records"]
        if r.get("accepted") and "T_base_link8" in r
        and "T_target2cam" in r
    ]
    nonamb = [
        r for r in all_records
        if r.get("reproj_gap", 0.0) >= parsed.gap
    ]

    subsets = {
        "all": all_records,
        "nonamb": nonamb,
    }

    print("truth T_link8_cam:")
    for row in truth:
        print("  " + np.array2string(row, precision=4))
    print(
        f"\naccepted={len(all_records)}, "
        f"nonamb(gap>={parsed.gap})={len(nonamb)}"
    )

    results = []
    for name, records in subsets.items():
        if len(records) < 6:
            print(f"\n[{name}] too few poses: {len(records)}")
            continue
        for method in METHODS:
            try:
                T_est = solve(records, method)
            except Exception as exc:
                print(f"\n[{name}/{method}] failed: {exc}")
                continue

            t_err, r_err = pose_error(T_est, truth)
            t_mean, t_max, r_mean, r_max = target_consistency(
                records, T_est
            )

            # 留 3 个姿态不参与求解，看板子一致性
            order = np.linspace(
                0, len(records) - 1, 3
            ).round().astype(int)
            hold = [records[i] for i in order]
            train = [
                r for i, r in enumerate(records) if i not in set(order)
            ]
            hold_t = hold_r = 0.0
            if len(train) >= 6:
                T_train = solve(train, method)
                T_bt_train = [
                    np.asarray(r["T_base_link8"])
                    @ T_train
                    @ np.asarray(r["T_target2cam"])
                    for r in train
                ]
                ref = T_bt_train[0]
                hold_errors = []
                for r in hold:
                    T_bt = (
                        np.asarray(r["T_base_link8"])
                        @ T_train
                        @ np.asarray(r["T_target2cam"])
                    )
                    rel = inv_T(ref) @ T_bt
                    hold_errors.append(
                        (
                            1000.0 * float(np.linalg.norm(rel[:3, 3])),
                            rot_angle_deg(np.eye(3), rel[:3, :3]),
                        )
                    )
                hold_t = float(
                    np.mean([e[0] for e in hold_errors])
                )
                hold_r = float(
                    np.mean([e[1] for e in hold_errors])
                )

            print(f"\n[{name}/{method}] n={len(records)}")
            print(
                f"  truth error: t={t_err:.2f} mm, r={r_err:.3f} deg"
            )
            print(
                f"  target consistency: "
                f"t mean/max={t_mean:.2f}/{t_max:.2f} mm, "
                f"r mean/max={r_mean:.3f}/{r_max:.3f} deg"
            )
            print(
                f"  holdout mean: t={hold_t:.2f} mm, "
                f"r={hold_r:.3f} deg"
            )
            print("  T_link8_cam:")
            for row in T_est:
                print("    " + np.array2string(row, precision=4))

            results.append(
                {
                    "subset": name,
                    "method": method,
                    "n": len(records),
                    "truth_t_mm": t_err,
                    "truth_r_deg": r_err,
                    "target_t_mean_mm": t_mean,
                    "target_t_max_mm": t_max,
                    "target_r_mean_deg": r_mean,
                    "target_r_max_deg": r_max,
                    "holdout_t_mean_mm": hold_t,
                    "holdout_r_mean_deg": hold_r,
                    "T_link8_cam": T_est.tolist(),
                }
            )

    # 对 ambiguous 姿态枚举两种 IPPE 解，用静态板一致性选全局分支。
    # 选择过程不使用真值；真值只用于最后的误差报告。
    ambiguous = [
        r for r in all_records
        if r.get("ambiguous") and len(r.get("solutions", [])) >= 2
    ]
    if ambiguous:
        best_branch = None
        for combo in itertools.product(
            [0, 1], repeat=len(ambiguous)
        ):
            selected = []
            amb_index = 0
            for record in all_records:
                new_record = dict(record)
                if (
                    record.get("ambiguous")
                    and len(record.get("solutions", [])) >= 2
                ):
                    solution = record["solutions"][combo[amb_index]]
                    new_record["T_target2cam"] = (
                        target_from_solution(solution).tolist()
                    )
                    amb_index += 1
                selected.append(new_record)
            try:
                T_candidate = solve(selected, "PARK")
            except Exception:
                continue
            t_mean, t_max, r_mean, r_max = target_consistency(
                selected, T_candidate
            )
            score = t_mean + 10.0 * r_mean
            if best_branch is None or score < best_branch["score"]:
                best_branch = {
                    "score": score,
                    "combo": combo,
                    "records": selected,
                    "t_mean": t_mean,
                    "t_max": t_max,
                    "r_mean": r_mean,
                    "r_max": r_max,
                }

        if best_branch is not None:
            print("\nbranch search (selection uses consistency only)")
            for index, record in enumerate(ambiguous):
                print(
                    f"  {record['label']}: "
                    f"solution {best_branch['combo'][index]}"
                )
            for method in METHODS:
                try:
                    T_est = solve(best_branch["records"], method)
                except Exception as exc:
                    print(f"\n[branch/{method}] failed: {exc}")
                    continue
                t_err, r_err = pose_error(T_est, truth)
                print(f"\n[branch/{method}] n={len(all_records)}")
                print(
                    f"  truth error: t={t_err:.2f} mm, "
                    f"r={r_err:.3f} deg"
                )
                print(
                    f"  target consistency: "
                    f"t mean/max={best_branch['t_mean']:.2f}/"
                    f"{best_branch['t_max']:.2f} mm, "
                    f"r mean/max={best_branch['r_mean']:.3f}/"
                    f"{best_branch['r_max']:.3f} deg"
                )
                results.append(
                    {
                        "subset": "branch_search",
                        "method": method,
                        "n": len(all_records),
                        "branch_combo": list(best_branch["combo"]),
                        "branch_labels": [
                            r["label"] for r in ambiguous
                        ],
                        "truth_t_mm": t_err,
                        "truth_r_deg": r_err,
                        "target_t_mean_mm": best_branch["t_mean"],
                        "target_t_max_mm": best_branch["t_max"],
                        "target_r_mean_deg": best_branch["r_mean"],
                        "target_r_max_deg": best_branch["r_max"],
                        "T_link8_cam": T_est.tolist(),
                    }
                )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "experiments",
            "hand_eye",
            f"run_{stamp}",
        )
    )
    os.makedirs(outdir, exist_ok=True)
    with open(
        os.path.join(outdir, "result.json"), "w", encoding="utf-8"
    ) as handle:
        json.dump(
            {
                "poses": os.path.abspath(parsed.poses),
                "gap": parsed.gap,
                "truth_T_link8_cam": truth.tolist(),
                "results": results,
            },
            handle,
            indent=2,
            ensure_ascii=False,
        )
    print(f"\noutput: {outdir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
