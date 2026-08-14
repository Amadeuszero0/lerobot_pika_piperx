"""Pinocchio/CasADi IK adapted from AgileX PikaAnyArm's Piper implementation."""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin


@dataclass(frozen=True)
class PiperIKResult:
    joints_rad: tuple[float, ...]
    position_error_m: float
    rotation_error_rad: float


@dataclass(frozen=True)
class PiperIKUpdate:
    request_id: int
    target_pose: tuple[float, ...]
    result: PiperIKResult | None


def _official_ik_worker_main(
    request_connection: Any,
    result_connection: Any,
    urdf_path: str,
    package_dir: str,
) -> None:
    try:
        solver = OfficialPiperIK(urdf_path, package_dir)
        result_connection.send(("ready",))
        while True:
            request = request_connection.recv()
            if request is None:
                return
            while request_connection.poll():
                newer_request = request_connection.recv()
                if newer_request is None:
                    return
                request = newer_request
            request_id, target_pose, current_joints, gripper_width_m = request
            result = solver.solve_native_pose(
                target_pose,
                current_joints,
                gripper_width_m=gripper_width_m,
            )
            result_connection.send(("result", request_id, target_pose, result))
    except BaseException:
        try:
            result_connection.send(("error", traceback.format_exc()))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        request_connection.close()
        result_connection.close()


class OfficialPiperIKWorker:
    """Run official Piper IK outside the CAN/control-loop interpreter."""

    def __init__(
        self,
        urdf_path: str,
        package_dir: str,
        *,
        name: str = "piper",
        startup_timeout_s: float = 30.0,
    ) -> None:
        context = mp.get_context("spawn")
        request_receiver, self._request_sender = context.Pipe(duplex=False)
        self._result_receiver, result_sender = context.Pipe(duplex=False)
        self._process = context.Process(
            target=_official_ik_worker_main,
            args=(request_receiver, result_sender, urdf_path, package_dir),
            name=f"{name}-official-ik",
            daemon=True,
        )
        self._next_request_id = 0
        self._closed = False
        self._process.start()
        request_receiver.close()
        result_sender.close()
        if not self._result_receiver.poll(startup_timeout_s):
            self.close()
            raise TimeoutError(
                f"Timed out waiting {startup_timeout_s:.1f}s for {name} IK worker"
            )
        message = self._result_receiver.recv()
        if message[0] == "error":
            self.close()
            raise RuntimeError(f"Failed to initialize {name} IK worker:\n{message[1]}")
        if message != ("ready",):
            self.close()
            raise RuntimeError(f"Unexpected {name} IK worker startup response: {message!r}")

    @property
    def is_alive(self) -> bool:
        return not self._closed and self._process.is_alive()

    def update_target(
        self,
        pose_mm_axis_angle: tuple[float, ...],
        current_joints_rad: tuple[float, ...],
        *,
        gripper_width_m: float = 0.0,
    ) -> PiperIKUpdate | None:
        if not self.is_alive:
            raise RuntimeError("Piper official IK worker is not running")
        self._next_request_id += 1
        target_pose = tuple(float(value) for value in pose_mm_axis_angle)
        current_joints = tuple(float(value) for value in current_joints_rad)
        self._request_sender.send(
            (
                self._next_request_id,
                target_pose,
                current_joints,
                float(gripper_width_m),
            )
        )
        latest: PiperIKUpdate | None = None
        while self._result_receiver.poll():
            message = self._result_receiver.recv()
            if message[0] == "error":
                raise RuntimeError(f"Piper official IK worker failed:\n{message[1]}")
            if message[0] != "result":
                raise RuntimeError(f"Unexpected Piper IK worker response: {message!r}")
            _, request_id, solved_target, result = message
            latest = PiperIKUpdate(request_id, solved_target, result)
        return latest

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self._process.is_alive():
                self._request_sender.send(None)
                self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2.0)
        finally:
            self._request_sender.close()
            self._result_receiver.close()


class OfficialPiperIK:
    """Solve the official Pika gripper-center target for a Piper-X model."""

    def __init__(self, urdf_path: str, package_dir: str) -> None:
        resolved_urdf = Path(urdf_path).expanduser().resolve()
        resolved_package_dir = Path(package_dir).expanduser().resolve()
        if not resolved_urdf.is_file():
            raise FileNotFoundError(f"Piper IK URDF does not exist: {resolved_urdf}")
        if not resolved_package_dir.is_dir():
            raise FileNotFoundError(
                f"Piper IK package directory does not exist: {resolved_package_dir}"
            )

        self.urdf_path = str(resolved_urdf)
        self.package_dir = str(resolved_package_dir)
        self.robot = pin.RobotWrapper.BuildFromURDF(
            self.urdf_path,
            package_dirs=self.package_dir,
        )
        model_joint_names = {str(name) for name in self.robot.model.names}
        gripper_joints = [
            name for name in ("joint7", "joint8") if name in model_joint_names
        ]
        if gripper_joints:
            self.reduced_robot = self.robot.buildReducedRobot(
                list_of_joints_to_lock=gripper_joints,
                reference_configuration=np.zeros(self.robot.model.nq),
            )
        else:
            self.reduced_robot = self.robot
        if self.reduced_robot.model.nq != 6:
            raise ValueError(
                f"Expected a six-axis Piper model, got nq={self.reduced_robot.model.nq}"
            )
        if self.robot.model.nq not in (6, 8):
            raise ValueError(
                f"Expected a six-axis Piper model with optional gripper, got "
                f"nq={self.robot.model.nq}"
            )
        self._has_gripper_joints = self.robot.model.nq == 8

        # PikaAnyArm defines Ry(-90 deg) @ Tx(190 mm) for a standard Piper.
        # Piper-X mounts the same gripper at +90 deg around J6's Z axis.
        gripper_mount_rotation = pin.rpy.rpyToMatrix(0.0, 0.0, math.pi / 2.0)
        pika_tool_rotation = pin.rpy.rpyToMatrix(0.0, -math.pi / 2.0, 0.0)
        tool_rotation = gripper_mount_rotation @ pika_tool_rotation
        tool_translation = tool_rotation @ np.array([0.19, 0.0, 0.0])
        self.tool_placement = pin.SE3(tool_rotation, tool_translation)
        self.joint6_id = self.reduced_robot.model.getJointId("joint6")
        self.ee_frame_id = self.reduced_robot.model.addFrame(
            pin.Frame(
                "pika_ee",
                self.joint6_id,
                self.tool_placement,
                pin.FrameType.OP_FRAME,
            )
        )
        # RobotWrapper creates its data before the tool frame is added.
        self.reduced_data = self.reduced_robot.model.createData()

        self._build_optimizer()
        self._build_collision_model()
        self._last_solution: np.ndarray | None = None
        self._warm_up_solver()

    def _warm_up_solver(self) -> None:
        """Load IPOPT before CAN feedback timing becomes safety-critical."""
        seed = np.array([0.0, math.pi / 4.0, -math.pi / 2.0, 0.0, math.pi / 4.0, 0.0])
        seed = np.clip(
            seed,
            self.reduced_robot.model.lowerPositionLimit,
            self.reduced_robot.model.upperPositionLimit,
        )
        pose = self.native_pose_from_joints(tuple(float(value) for value in seed))
        self.solve_native_pose(
            pose,
            tuple(float(value) for value in seed),
            gripper_width_m=0.0,
        )
        self._last_solution = None

    def _build_optimizer(self) -> None:
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        target = casadi.SX.sym("target", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, cq)
        error = casadi.Function(
            "piper_ik_error",
            [cq, target],
            [
                cpin.log6(
                    self.cdata.oMf[self.ee_frame_id].inverse() * cpin.SE3(target)
                ).vector
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.param_target = self.opti.parameter(4, 4)
        error_vector = error(self.var_q, self.param_target)
        position_error = error_vector[:3]
        orientation_error = error_vector[3:]
        total_cost = casadi.sumsqr(position_error) + casadi.sumsqr(
            0.1 * orientation_error
        )
        regularization = casadi.sumsqr(self.var_q)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )
        self.opti.minimize(20.0 * total_cost + 0.01 * regularization)
        self.opti.solver(
            "ipopt",
            {
                "ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-4},
                "print_time": False,
            },
        )

    def _build_collision_model(self) -> None:
        self.geometry_model = pin.buildGeomFromUrdf(
            self.robot.model,
            self.urdf_path,
            pin.GeometryType.COLLISION,
            None,
            self.package_dir,
        )
        geometry_count = len(self.geometry_model.geometryObjects)
        for moving_index in range(4, min(10, geometry_count)):
            for base_index in range(min(3, geometry_count)):
                self.geometry_model.addCollisionPair(
                    pin.CollisionPair(moving_index, base_index)
                )
        self.geometry_data = pin.GeometryData(self.geometry_model)

    def gripper_target_from_native_pose(
        self, pose_mm_axis_angle: tuple[float, ...]
    ) -> pin.SE3:
        if len(pose_mm_axis_angle) != 6:
            raise ValueError("Piper native pose must contain six values")
        values = np.asarray(pose_mm_axis_angle, dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("Piper native pose must contain only finite values")
        native_pose = pin.SE3(pin.exp3(values[3:6]), values[:3] / 1000.0)
        return native_pose * self.tool_placement

    def native_pose_from_joints(self, joints_rad: tuple[float, ...]) -> tuple[float, ...]:
        joints = self._validate_joints(joints_rad)
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_data, joints)
        native_pose = self.reduced_data.oMi[self.joint6_id]
        rotation_vector = pin.log3(native_pose.rotation)
        return (
            *(float(value) * 1000.0 for value in native_pose.translation),
            *(float(value) for value in rotation_vector),
        )

    def solve_native_pose(
        self,
        pose_mm_axis_angle: tuple[float, ...],
        current_joints_rad: tuple[float, ...],
        *,
        gripper_width_m: float = 0.0,
    ) -> PiperIKResult | None:
        target = self.gripper_target_from_native_pose(pose_mm_axis_angle)
        current = self._validate_joints(current_joints_rad)
        initial = self._last_solution if self._last_solution is not None else current
        # If feedback no longer matches the previous branch, restart from feedback.
        if float(np.max(np.abs(initial - current))) > math.radians(30.0):
            initial = current

        self.opti.set_initial(self.var_q, initial)
        self.opti.set_value(self.param_target, target.homogeneous)
        try:
            solution = self.opti.solve_limited()
            joints = np.asarray(solution.value(self.var_q), dtype=float).reshape(-1)
        except Exception:
            return None
        if not np.all(np.isfinite(joints)):
            return None
        if np.any(joints < self.reduced_robot.model.lowerPositionLimit - 1e-6) or np.any(
            joints > self.reduced_robot.model.upperPositionLimit + 1e-6
        ):
            return None

        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_data, joints)
        solved_pose = self.reduced_data.oMf[self.ee_frame_id]
        position_error_m = float(np.linalg.norm(solved_pose.translation - target.translation))
        rotation_error_rad = float(
            np.linalg.norm(pin.log3(solved_pose.rotation.T @ target.rotation))
        )
        # PikaAnyArm rejects a result when any Cartesian axis misses by over 0.3 m.
        if float(np.max(np.abs(solved_pose.translation - target.translation))) > 0.3:
            return None
        if self._has_self_collision(joints, gripper_width_m):
            return None

        self._last_solution = joints.copy()
        return PiperIKResult(
            joints_rad=tuple(float(value) for value in joints),
            position_error_m=position_error_m,
            rotation_error_rad=rotation_error_rad,
        )

    def _has_self_collision(self, joints: np.ndarray, gripper_width_m: float) -> bool:
        if self._has_gripper_joints:
            width = min(0.07, max(0.0, float(gripper_width_m)))
            full_configuration = np.concatenate([joints, [width / 2.0, -width / 2.0]])
        else:
            full_configuration = joints
        pin.forwardKinematics(self.robot.model, self.robot.data, full_configuration)
        pin.updateGeometryPlacements(
            self.robot.model,
            self.robot.data,
            self.geometry_model,
            self.geometry_data,
        )
        return bool(pin.computeCollisions(self.geometry_model, self.geometry_data, False))

    def _validate_joints(self, joints_rad: tuple[float, ...]) -> np.ndarray:
        joints = np.asarray(joints_rad, dtype=float).reshape(-1)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("Piper joint seed must contain six finite radians")
        return joints

    @property
    def lower_limits_rad(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self.reduced_robot.model.lowerPositionLimit)

    @property
    def upper_limits_rad(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self.reduced_robot.model.upperPositionLimit)

    @property
    def last_solution(self) -> tuple[float, ...] | None:
        if self._last_solution is None:
            return None
        return tuple(float(value) for value in self._last_solution)
