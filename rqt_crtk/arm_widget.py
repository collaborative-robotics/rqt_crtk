"""PyQt5 widget for a generic CRTK arm."""

from __future__ import annotations

import math

import numpy as np
from python_qt_binding import QtCore, QtWidgets

from dvrk_simulator_base.types import Pose


def _rpy(rotation):
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-8:
        return (math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
                math.atan2(rotation[1, 0], rotation[0, 0]))
    return (0.0, pitch, math.atan2(-rotation[0, 1], rotation[1, 1]))


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                       (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                       (-sp, cp * sr, cp * cr)), dtype=float)


class CRTKArmWidget(QtWidgets.QWidget):
    """Display standard CRTK state and send discrete joint/state commands."""

    def __init__(self, arm, parent=None):
        super().__init__(parent)
        self.arm = arm
        self._updating = False
        self._joint_dirty = False
        self._cartesian_dirty = False
        layout = QtWidgets.QVBoxLayout(self)
        controls = QtWidgets.QHBoxLayout()
        self.state = QtWidgets.QLabel("State: UNKNOWN")
        self.homed = QtWidgets.QLabel("Not homed")
        self.busy = QtWidgets.QLabel("Idle")
        self.state_command = QtWidgets.QComboBox()
        self.state_command.addItem("State command…", "")
        for command in ("enable", "disable", "pause", "resume", "home", "unhome", "fault", "clear_fault"):
            self.state_command.addItem(command, command)
        self.state_command.activated.connect(self._state_command)
        controls.addWidget(self.state_command)
        controls.addWidget(self.state)
        controls.addWidget(self.homed)
        controls.addWidget(self.busy)
        controls.addStretch()
        layout.addLayout(controls)

        group = QtWidgets.QGroupBox("Joint positions")
        group_layout = QtWidgets.QVBoxLayout(group)
        self.joints = QtWidgets.QTableWidget(2, len(arm.config.joints))
        self.joints.setVerticalHeaderLabels(["Measured", "Command"])
        self.joints.setHorizontalHeaderLabels([joint.name for joint in arm.config.joints])
        self._joint_fields = []
        for index, joint in enumerate(arm.config.joints):
            measured = QtWidgets.QTableWidgetItem("0.00")
            measured.setFlags(measured.flags() & ~QtCore.Qt.ItemIsEditable)
            self.joints.setItem(0, index, measured)
            field = QtWidgets.QDoubleSpinBox()
            if joint.type == "revolute":
                field.setRange(math.degrees(joint.lower), math.degrees(joint.upper))
            else:
                field.setRange(joint.lower * 1000.0, joint.upper * 1000.0)
            field.setDecimals(2)
            field.valueChanged.connect(self._mark_joint_dirty)
            self.joints.setCellWidget(1, index, field)
            self._joint_fields.append(field)
        self.joints.horizontalHeader().setStretchLastSection(True)
        group_layout.addWidget(self.joints)
        apply = QtWidgets.QPushButton("Move joints")
        apply.clicked.connect(self._move_joints)
        group_layout.addWidget(apply)
        layout.addWidget(group)

        if arm.config.type == "PSM":
            jaw_layout = QtWidgets.QHBoxLayout()
            jaw_layout.addWidget(QtWidgets.QLabel("Jaw (deg)"))
            self._jaw = QtWidgets.QDoubleSpinBox()
            self._jaw.setRange(-30.0, 90.0)
            self._jaw.setDecimals(2)
            self._jaw.valueChanged.connect(self._mark_joint_dirty)
            jaw_layout.addWidget(self._jaw)
            layout.addLayout(jaw_layout)
        else:
            self._jaw = None

        self._cartesian_group = QtWidgets.QGroupBox("Cartesian pose")
        cartesian_layout = QtWidgets.QGridLayout(self._cartesian_group)
        self._cartesian_fields = []
        for index, name in enumerate(("X (mm)", "Y (mm)", "Z (mm)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)")):
            cartesian_layout.addWidget(QtWidgets.QLabel(name), index // 3, (index % 3) * 2)
            field = QtWidgets.QDoubleSpinBox()
            field.setRange(-2000.0 if index < 3 else -180.0, 2000.0 if index < 3 else 180.0)
            field.setDecimals(2)
            field.valueChanged.connect(self._mark_cartesian_dirty)
            cartesian_layout.addWidget(field, index // 3, (index % 3) * 2 + 1)
            self._cartesian_fields.append(field)
        move_cartesian = QtWidgets.QPushButton("Move Cartesian")
        move_cartesian.clicked.connect(self._move_cartesian)
        cartesian_layout.addWidget(move_cartesian, 2, 0, 1, 6)
        layout.addWidget(self._cartesian_group)
        layout.addStretch()

    def _mark_joint_dirty(self, _value):
        if not self._updating:
            self._joint_dirty = True

    def _mark_cartesian_dirty(self, _value):
        if not self._updating:
            self._cartesian_dirty = True

    def _state_command(self, index):
        command = self.state_command.itemData(index)
        self.state_command.setCurrentIndex(0)
        if command:
            self.arm.command_state(command)

    def _move_joints(self):
        values = []
        for field, joint in zip(self._joint_fields, self.arm.config.joints):
            value = field.value()
            values.append(math.radians(value) if joint.type == "revolute" else value / 1000.0)
        jaw = math.radians(self._jaw.value()) if self._jaw is not None else None
        self.arm.command_joints(values, jaw)
        self._joint_dirty = False

    def _move_cartesian(self):
        values = [field.value() for field in self._cartesian_fields]
        pose = Pose(np.asarray(values[:3], dtype=float) / 1000.0,
                    _rotation(*np.radians(values[3:])))
        self.arm.command_cartesian(pose)
        self._cartesian_dirty = False

    def update_snapshot(self, snapshot):
        self._updating = True
        try:
            state = snapshot.operating_state
            self.state.setText("State: {}".format(state.state))
            self.homed.setText("Homed" if state.is_homed else "Not homed")
            self.busy.setText("Busy" if state.is_busy else "Idle")
            if not self._joint_dirty:
                for index, (joint, position) in enumerate(zip(self.arm.config.joints, snapshot.measured_js.position)):
                    display = math.degrees(position) if joint.type == "revolute" else position * 1000.0
                    self.joints.item(0, index).setText("{:.2f}".format(display))
                    self._joint_fields[index].setValue(display)
                if self._jaw is not None and snapshot.jaw_measured is not None:
                    self._jaw.setValue(math.degrees(snapshot.jaw_measured))
            if not self._cartesian_dirty:
                pose = snapshot.measured_cp_world
                self._cartesian_group.setTitle(
                    "Cartesian pose (wrt {})".format(self.arm.cartesian_frame)
                )
                values = [*(pose.position * 1000.0), *np.degrees(_rpy(pose.orientation))]
                for field, value in zip(self._cartesian_fields, values):
                    field.setValue(float(value))
        finally:
            self._updating = False
