"""Run a CRTK RAL node safely from rqt's Qt event loop."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor


class QtRALExecutor:
    """Avoid CRTK RAL's background spin thread inside an rqt process.

    rqt owns the ROS context.  A background RAL executor can otherwise wake
    after Ctrl+C has shut that context down and print an RCLError traceback.
    This executor is advanced by a Qt timer in the GUI thread instead.
    """

    def __init__(self, ral):
        self._executor = SingleThreadedExecutor()
        self._node = ral._node  # CRTK RAL's node; it has no public accessor.
        self._executor.add_node(self._node)
        self._closed = False

    def spin_once(self) -> None:
        if self._closed or not rclpy.ok():
            return
        try:
            self._executor.spin_once(timeout_sec=0.0)
        except (ExternalShutdownException, rclpy._rclpy_pybind11.RCLError):
            # Ctrl+C can invalidate the shared rqt ROS context between the
            # ``ok`` check and spin_once.  rqt owns final shutdown.
            self._closed = True

    def shutdown(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._executor.remove_node(self._node)
        self._executor.shutdown()
