"""rqt panel for standard ROS diagnostics."""

from __future__ import annotations

import argparse
import threading

import crtk
from diagnostic_msgs.msg import DiagnosticArray
from python_qt_binding import QtCore, QtWidgets
from rqt_gui_py.plugin import Plugin
from .ral_executor import QtRALExecutor


class DiagnosticsPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument("--topic", default="/diagnostics")
        options, unknown = parser.parse_known_args(context.argv())
        if unknown:
            raise ValueError("unknown plugin arguments: {}".format(" ".join(unknown)))
        self._lock = threading.Lock()
        self._statuses = {}
        self._ral = crtk.ral("rqt_crtk_diagnostics")
        self._subscription = self._ral.subscriber(options.topic, DiagnosticArray, self._callback, queue_size=10)
        self._widget = QtWidgets.QTreeWidget()
        self._widget.setHeaderLabels(["Source", "Level", "Message", "Metrics"])
        self._widget.setRootIsDecorated(False)
        context.add_widget(self._widget)
        self._executor = QtRALExecutor(self._ral)
        self._spin_timer = QtCore.QTimer(self._widget)
        self._spin_timer.timeout.connect(self._executor.spin_once)
        self._spin_timer.start(10)
        self._timer = QtCore.QTimer(self._widget)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(250)

    def _callback(self, message):
        with self._lock:
            self._statuses.update({status.name: status for status in message.status})

    def _refresh(self):
        with self._lock:
            statuses = [self._statuses[name] for name in sorted(self._statuses)]
        self._widget.clear()
        levels = ("OK", "WARN", "ERROR", "STALE")
        for status in statuses:
            raw_level = status.level
            # Some ROS 2 Python message bindings expose uint8 as ``bytes``
            # rather than ``int``.  Normalize both forms for PyQt display.
            level = raw_level[0] if isinstance(raw_level, bytes) else int(raw_level)
            metrics = "; ".join("{}: {}".format(item.key, item.value) for item in status.values)
            self._widget.addTopLevelItem(QtWidgets.QTreeWidgetItem(
                [status.name, levels[min(max(level, 0), 3)], status.message, metrics]))

    def shutdown_plugin(self):
        self._timer.stop()
        self._spin_timer.stop()
        self._executor.shutdown()
