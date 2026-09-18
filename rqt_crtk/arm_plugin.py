"""rqt plugin connecting the generic arm widget through CRTK Python."""

from __future__ import annotations

import argparse
from pathlib import Path

import crtk
from ament_index_python.packages import get_package_share_directory
from python_qt_binding import QtCore
from rqt_gui_py.plugin import Plugin

from dvrk_simulator_base.config import load_robot_config
from .arm_client import CRTKMonitorArm
from .arm_widget import CRTKArmWidget
from .ral_executor import QtRALExecutor


class ArmPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        options = self._options(context.argv())
        config_path = Path(options.robot_config) if options.robot_config else (
            Path(get_package_share_directory("dvrk_simulator_base")) / "share" / "arms" / f"{options.arm}.yaml"
        )
        self._ral = crtk.ral("rqt_crtk_arm")
        self._arm = CRTKMonitorArm(self._ral, load_robot_config(config_path), options.namespace)
        self._widget = CRTKArmWidget(self._arm)
        context.add_widget(self._widget)
        self._executor = QtRALExecutor(self._ral)
        self._spin_timer = QtCore.QTimer(self._widget)
        self._spin_timer.timeout.connect(self._executor.spin_once)
        self._spin_timer.start(10)
        self._timer = QtCore.QTimer(self._widget)
        self._timer.timeout.connect(lambda: self._widget.update_snapshot(self._arm.snapshot()))
        self._timer.start(50)

    @staticmethod
    def _options(arguments):
        parser = argparse.ArgumentParser(add_help=False)
        choices = parser.add_mutually_exclusive_group(required=True)
        choices.add_argument("--arm")
        choices.add_argument("--robot-config")
        parser.add_argument("--namespace")
        options, unknown = parser.parse_known_args(arguments)
        if unknown:
            raise ValueError("unknown plugin arguments: {}".format(" ".join(unknown)))
        return options

    def shutdown_plugin(self):
        self._timer.stop()
        self._spin_timer.stop()
        self._executor.shutdown()
