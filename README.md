# rqt_crtk

Generic ROS 2 rqt plugins for CRTK devices.  The package uses `python_qt_binding`
with PyQt5 and depends on `crtk_python_client` plus the currently shared
`dvrk_simulator_base` configuration/client adapter.  It has no dependency on
`dvrk_python`, Isaac Sim, or PyBullet.

```bash
rqt --standalone rqt_crtk/Arm --args --arm PSM1
rqt --standalone rqt_crtk/Diagnostics
```

The arm plugin accepts `--robot-config FILE` and optional `--namespace NAME`.
The diagnostics plugin displays `diagnostic_msgs/DiagnosticArray` on
`/diagnostics`.

The Arm panel sends standard CRTK state, joint, optional jaw, and Cartesian
commands. Cartesian commands retain the frame ID of the latest `measured_cp`
message, so a PSM displayed in `ECM_view` is commanded back in `ECM_view`.

For a Classic model patient cart, use the same PSM/ECM configurations as the
simulators:

```bash
rqt --standalone rqt_crtk/Arm --args --arm PSM1
rqt --standalone rqt_crtk/Arm --args --arm ECM
```
