"""CRTK Python client adapter used by the generic rqt arm widget."""

from __future__ import annotations

import numpy as np

from dvrk_simulator_base.config import RobotConfig
from dvrk_simulator_base.rotations import rotation_to_quaternion_xyzw
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose, Twist


def _pose_from_frame(frame) -> Pose:
    return Pose(
        np.asarray((frame.p[0], frame.p[1], frame.p[2]), dtype=float),
        np.asarray([[frame.M[row, column] for column in range(3)] for row in range(3)], dtype=float),
    )


def _frame_from_pose(pose: Pose):
    import crtk

    rotation = crtk.PyKDL.Rotation(*pose.orientation.reshape(9).tolist())
    position = crtk.PyKDL.Vector(*pose.position.tolist())
    return crtk.PyKDL.Frame(rotation, position)


class CRTKMonitorArm:
    """Adapt a CRTK Python client device to the generic rqt arm widget."""

    def __init__(self, ral, config: RobotConfig, namespace: str | None = None):
        import crtk
        from geometry_msgs.msg import PoseStamped

        self.config = config
        self.ral = ral.create_child(namespace or config.name)
        self._client = crtk.utils(self, self.ral, 1.0)
        self._client.add_operating_state()
        self._client.add_measured_js()
        self._client.add_setpoint_js()
        self._client.add_measured_cp()
        self._client.add_setpoint_cp()
        self._client.add_measured_cv()
        self._client.add_move_jp()
        self._jaw = _CRTKJaw(self.ral.create_child("jaw"), self) if config.type == "PSM" else None
        names = tuple(joint.name for joint in config.joints)
        self._default_js = JointState(names, np.zeros(len(names)), np.zeros(len(names)))
        self._default_pose = Pose(np.zeros(3), np.eye(3))
        self._default_twist = Twist(np.zeros(3), np.zeros(3))
        self._sequence = 0
        self._cartesian_frame = config.parent_frame
        self._PoseStamped = PoseStamped
        self._move_cp_publisher = self.ral.publisher("move_cp", PoseStamped, latch=False, queue_size=10)
        self._cartesian_frame_subscriber = self.ral.subscriber(
            "measured_cp", PoseStamped, self._cartesian_frame_callback, queue_size=10
        )

    @staticmethod
    def _value(method, fallback):
        try:
            return method(wait_timeout=0.0)
        except (RuntimeError, TimeoutError):
            return fallback

    def _joint_state(self, method, fallback: JointState) -> JointState:
        value = self._value(method, None)
        if value is None:
            return fallback
        position, velocity, _effort, _stamp = value
        position = np.asarray(position, dtype=float)
        if position.shape != fallback.position.shape or not np.all(np.isfinite(position)):
            return fallback
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape != position.shape or not np.all(np.isfinite(velocity)):
            velocity = np.zeros_like(position)
        return JointState(fallback.names, position, velocity)

    def _pose(self, method, fallback: Pose) -> Pose:
        value = self._value(method, None)
        if value is None:
            return fallback
        try:
            return _pose_from_frame(value[0])
        except (TypeError, ValueError):
            return fallback

    def snapshot(self) -> ArmSnapshot:
        measured_js = self._joint_state(self.measured_js, self._default_js)
        setpoint_js = self._joint_state(self.setpoint_js, measured_js)
        measured_cp = self._pose(self.measured_cp, self._default_pose)
        setpoint_cp = self._pose(self.setpoint_cp, measured_cp)
        velocity_value = self._value(self.measured_cv, None)
        measured_cv = self._default_twist
        if velocity_value is not None:
            velocity = np.asarray(velocity_value[0], dtype=float)
            if velocity.shape == (6,) and np.all(np.isfinite(velocity)):
                measured_cv = Twist(velocity[:3], velocity[3:])
        try:
            state = str(self.operating_state())
            operating = OperatingStateSnapshot(state, bool(self.is_homed()), bool(self.is_busy())) if state else OperatingStateSnapshot("UNKNOWN", False, False)
        except (RuntimeError, TimeoutError):
            operating = OperatingStateSnapshot("UNKNOWN", False, False)
        jaw_measured = self._jaw.position(self._jaw.measured_js) if self._jaw else None
        jaw_setpoint = self._jaw.position(self._jaw.setpoint_js) if self._jaw else None
        snapshot = ArmSnapshot(
            sequence=self._sequence, simulation_time=0.0, valid=True,
            measured_js=measured_js, setpoint_js=setpoint_js,
            measured_cp_world=measured_cp, setpoint_cp_world=setpoint_cp,
            measured_cv_world=measured_cv, jaw_measured=jaw_measured,
            jaw_setpoint=jaw_setpoint, operating_state=operating,
        )
        self._sequence += 1
        return snapshot

    def _cartesian_frame_callback(self, message) -> None:
        if message.header.frame_id:
            self._cartesian_frame = str(message.header.frame_id)

    @property
    def cartesian_frame(self) -> str:
        return self._cartesian_frame

    def command_state(self, command: str) -> None:
        self.state_command(command)

    def command_joints(self, position: list[float], jaw: float | None) -> None:
        self.move_jp(np.asarray(position, dtype=float))
        if jaw is not None and self._jaw is not None:
            self._jaw.move_jp(np.asarray((jaw,), dtype=float))

    def command_cartesian(self, pose: Pose) -> None:
        message = self._PoseStamped()
        message.header.frame_id = self._cartesian_frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
        quaternion = rotation_to_quaternion_xyzw(pose.orientation)
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = quaternion
        self.ral.set_timestamp(message)
        self._move_cp_publisher.publish(message)


class _CRTKJaw:
    def __init__(self, ral, operating_state_instance):
        import crtk

        self._client = crtk.utils(self, ral, 1.0, operating_state_instance=operating_state_instance)
        self._client.add_measured_js()
        self._client.add_setpoint_js()
        self._client.add_move_jp()

    @staticmethod
    def position(method) -> float | None:
        try:
            value, _velocity, _effort, _stamp = method(wait_timeout=0.0)
        except (RuntimeError, TimeoutError):
            return None
        if len(value) != 1 or not np.isfinite(value[0]):
            return None
        return float(value[0])