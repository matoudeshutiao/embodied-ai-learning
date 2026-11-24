#!/usr/bin/env python3
"""腕部相机 PBVS 闭环接近。

初始位姿用 /model_states 真值选（只选起点）；
闭环目标由腕部相机 + 手眼标定计算，不使用真值；
真值只用于最后的误差评估。

用法：
  python3 scripts/wrist_servo.py --offset 0.20 --starts 5
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy

from wrist_cube_detect import WristCubeDetect
from wrist_vision import CUBE_HALF, estimate_cube, draw_debug

# 五个起点相对方块真值的偏移；真值只用于摆起点
STARTS = [
    (0.0, 0.0),
    (0.02, 0.0),
    (-0.02, 0.0),
    (0.0, 0.02),
    (0.0, -0.02),
]


class WristServo(WristCubeDetect):
    def wait_tf_ready(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                self.lookup_link8()
                return True
            except Exception:
                self.spin_for(0.2)
        return False

    def run_servo(self, offset, alpha, tol, max_iter, n_starts):
        if not self.wait_ready():
            print("ERROR: 没有收到 wrist_camera / model_states")
            return 1
        if not self.wait_tf_ready():
            print("ERROR: TF 未就绪")
            return 1

        print(
            f"hand-eye: method={self.calib.get('method')} "
            f"validation={self.calib.get('validation')}"
        )
        outdir = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "..", "output"
            )
        )
        os.makedirs(outdir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        results = []
        for start_index in range(n_starts):
            dx, dy = STARTS[start_index % len(STARTS)]
            start_x = float(self.truth[0] + dx)
            start_y = float(self.truth[1] + dy)
            start_z = float(
                self.truth[2] + CUBE_HALF + offset
            )
            print(
                f"\n[start {start_index}] "
                f"({start_x:.3f},{start_y:.3f},{start_z:.3f}) "
                f"offset=({dx:+.2f},{dy:+.2f})"
            )
            if not self.move_to(start_x, start_y, start_z):
                print(
                    f"  MoveIt failed at start "
                    f"code={self.last_move_code}"
                )
                results.append(
                    {
                        "start": start_index,
                        "converged": False,
                        "iterations": 0,
                        "reason": "moveit_start",
                    }
                )
                continue
            self.spin_for(1.0)

            converged = False
            iterations = 0
            final_truth_error = None
            final_hand_error = None
            for iteration in range(max_iter):
                frame = self.capture()
                if frame is None:
                    print("  no fresh frame")
                    break
                image, depth = frame
                T_base_link8 = self.lookup_link8()
                T_base_cam = T_base_link8 @ self.T_link8_cam
                result, message = estimate_cube(
                    image, depth, self.K, T_base_cam
                )
                if result is None:
                    print(f"  iter {iteration}: detect failed: {message}")
                    break

                quality = (
                    result["plane_inliers"]
                    >= 0.85 * result["red_points"]
                    and result["red_points"] >= 3000
                    and result["normal"][2] >= 0.98
                    and 0.36
                    <= result["cube_center"][2]
                    <= 0.39
                )
                if not quality:
                    print(
                        f"  iter {iteration}: low-quality detection "
                        f"(red={result['red_points']}, "
                        f"inliers={result['plane_inliers']}, "
                        f"normal_z={result['normal'][2]:.3f}), "
                        f"stop this start"
                    )
                    break

                top = result["top_center"]
                cube = result["cube_center"]
                desired = np.array(
                    [top[0], top[1], top[2] + offset]
                )
                current = T_base_link8[:3, 3].copy()
                error = desired - current
                final_hand_error = float(np.linalg.norm(error))
                final_truth_error = 1000.0 * float(
                    np.linalg.norm(cube - self.truth)
                )
                iterations = iteration + 1
                print(
                    f"  iter {iteration}: cube="
                    f"({cube[0]:.4f},{cube[1]:.4f},{cube[2]:.4f}) "
                    f"hand_err={final_hand_error*1000:.2f} mm "
                    f"truth_err={final_truth_error:.2f} mm "
                    f"inliers={result['plane_inliers']}"
                )
                if final_hand_error < tol:
                    converged = True
                    break

                target = current + alpha * error
                if not self.move_to(
                    float(target[0]),
                    float(target[1]),
                    float(target[2]),
                ):
                    print(
                        f"  MoveIt failed code={self.last_move_code}, "
                        f"retry +5 mm z"
                    )
                    if not self.move_to(
                        float(target[0]),
                        float(target[1]),
                        float(target[2]) + 0.005,
                    ):
                        print(
                            f"  MoveIt failed again "
                            f"code={self.last_move_code}"
                        )
                        break
                self.spin_for(1.0)

            if converged and results is not None:
                lines = [
                    f"start {start_index} converged",
                    f"hand_err={final_hand_error*1000:.2f} mm",
                    f"truth_err={final_truth_error:.2f} mm",
                ]
                debug = draw_debug(image, result, lines)
                path = os.path.join(
                    outdir,
                    f"wrist_servo_{stamp}_{start_index}.png",
                )
                cv2.imwrite(path, debug)
                print(f"  final debug: {path}")

            results.append(
                {
                    "start": start_index,
                    "dx": dx,
                    "dy": dy,
                    "converged": bool(converged),
                    "iterations": iterations,
                    "final_hand_error_mm": (
                        None if final_hand_error is None
                        else 1000.0 * final_hand_error
                    ),
                    "final_truth_error_mm": final_truth_error,
                }
            )

        converged_count = sum(
            1 for r in results if r["converged"]
        )
        truth_errors = [
            r["final_truth_error_mm"] for r in results
            if r["final_truth_error_mm"] is not None
        ]
        print(
            f"\nsummary: converged {converged_count}/{len(results)}"
        )
        if truth_errors:
            print(
                f"truth error mean/max: "
                f"{np.mean(truth_errors):.2f}/"
                f"{np.max(truth_errors):.2f} mm"
            )
        print(
            f"iterations: "
            f"{[r['iterations'] for r in results]}"
        )
        return 0 if converged_count == len(results) else 1


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--offset", type=float, default=0.20)
    parser.add_argument("--alpha", type=float, default=0.7)
    parser.add_argument("--tol-mm", type=float, default=2.5)
    parser.add_argument("--max-iter", type=int, default=5)
    parser.add_argument("--starts", type=int, default=5)
    parsed = parser.parse_args(args)

    rclpy.init()
    node = WristServo(parsed)
    try:
        return node.run_servo(
            parsed.offset,
            parsed.alpha,
            parsed.tol_mm / 1000.0,
            parsed.max_iter,
            parsed.starts,
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
