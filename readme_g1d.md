# G1-D Integration Notes

## Status

This change adds the offline robot interface boundary in `core/g1d/g1d_interface.py`.
It maps a future G1-D transport to the four methods required by the shared real-arm
controller. `MockG1DTransport` and `tests/test_g1d_interface.py` verify that contract
without ROS, SDK access, networking, cameras, or physical motion.

G1-D is intentionally **not** registered as a supported real-hardware option in
`core/launch.py`. There is no validated G1-D transport implementation or calibrated
motion configuration in this repository yet. Selecting a G1-D real run must continue to
fail closed until those pieces are implemented and tested.

The mock applies a commanded pose immediately. It is only an API and wiring check; it
does not model G1-D kinematics, joint limits, dynamics, collision behavior, the gripper,
or camera geometry. The sample widths and pose in the test are mock-only values, not
robot calibration.

## Interface Contract

A transport implementation must provide these operations:

| Transport method | Meaning |
| --- | --- |
| `connect()` / `close()` | Start and release transport resources |
| `read_end_effector_pose()` | Measured base-frame pose `[x, y, z, qx, qy, qz, qw]`, meters |
| `read_gripper_width_m()` | Measured gripper opening in meters |
| `command_end_effector_pose(pose7)` | Send a Cartesian target in the same base frame |
| `command_gripper(closed)` | Close or open the active gripper |

The adapter validates finite pose values, nonzero quaternions, and nonnegative finite
gripper widths. It normalizes quaternions before passing them to the controller or
transport. A future ROS 2 client should implement `G1DTransport` and own the conversion
to the actual action, topic, or SDK messages. It must return measured feedback rather
than echoing commanded targets.

The shared controller expects a single controlled end effector. A dual-arm setup needs
an explicit arm selector and separate state/command routing; it must not silently send
commands to a default arm.

## Files Added or Relevant

- `core/g1d/__init__.py`: package exports.
- `core/g1d/g1d_interface.py`: transport protocol, validated adapter, and in-memory mock.
- `tests/test_g1d_interface.py`: offline adapter and shared-controller tests.
- `interpreters/real_atomic_controller.py`: existing transport-independent Cartesian
  action integration and safety floor.
- `interpreters/README.md`: existing real-arm controller contract.
- `core/piper/piper_session.py` and `core/piper/ros_camera.py`: examples of the current
  robot-session and RGB camera contracts. They are references, not G1-D dependencies.
- `core/launch.py`: current hardware factory. G1-D is not enabled there yet.

## Offline Verification

From the repository root:

```bash
python -m pytest tests/test_g1d_interface.py
```

The test exercises pose/gripper reads, command forwarding, connection checks, invalid
feedback rejection, and one shared-controller translation plus grasp. It does not
verify transport timing, ROS 2 QoS, action cancellation, camera delivery, or hardware
safety.

## Work Needed Before Real-Robot Integration

1. Confirm the controlled arm and end-effector frame, the base frame, and the exact
   measured pose source. Verify quaternion order, units, and timestamp/staleness behavior.
2. Confirm which G1-D motion interface the lab will support for Cartesian targets and
   how an in-flight command is stopped or canceled. Implement that as a separate
   `G1DTransport`; do not import or alter another workspace's bringup code from here.
3. Confirm the gripper command API, calibrated open/closed widths, feedback meaning, and
   empty-grasp behavior. Add explicit configuration instead of borrowing Franka/Piper
   thresholds.
4. Identify front and wrist RGB topics, encodings, frame transforms, resolution, and
   freshness limits. Build and test a session that returns `agentview`, `wrist`,
   `ee_pose`, and `gripper_width` in the existing observation contract.
5. Measure each atomic direction and yaw sign with the arm under an operator-controlled
   low-speed procedure. Create `configs/primitives_g1d.yaml`; do not copy another arm's
   axis signs or step sizes without verification.
6. Add an explicit G1-D session/controller/config route to `core/launch.py`, guarded by
   mock-only defaults until transport and calibration are reviewed. Add dry-run tests
   for config selection and real-mode refusal before enabling any hardware path.
7. Only after the lab validates the bridge: check workspace limits, collision settings,
   emergency stop, command timeout, watchdog behavior, and arm-select routing before a
   supervised low-speed test.

## Local Test Command

`python -m pytest tests/test_g1d_interface.py` is safe to run offline. Do not use this
mock as evidence that an autonomous real-arm rollout is ready.
