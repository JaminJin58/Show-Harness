"""Explicit one-arm G1-D profile; this module never opens a robot connection.

The profile is the single place that knows WHICH arm is selected and which ROS 2
endpoints belong to it. Everything downstream (the transport, the scripts, the
controllers) reads the endpoints from here, so a side can never be mixed up.

Endpoint namespaces
-------------------
Two G1-D bridge builds are in circulation in this lab, and they differ ONLY in the
namespace prefix of the Dex1 gripper endpoints:

=====================  ==============================  ==============================
build                  arm trajectory action           gripper action
=====================  ==============================  ==============================
``g1d_sdk_bridge``     ``/left_arm_controller/...``    ``/left_dex1/gripper_command``
``lzh_sdk_bridge``     ``/left_arm_controller/...``    ``/lzh_left_dex1/gripper_command``
=====================  ==============================  ==============================

Both publish ``/joint_states``; the Dex1 raw motor feedback is ``/dex1/motor_states``
or ``/lzh_dex1/motor_states`` respectively.

The arm trajectory actions are IDENTICAL in both builds, and they are the ones that
carry the motion -- so a wrong namespace cannot move the wrong arm, only break the
gripper. :data:`BRIDGE_NAMESPACES` names both builds and
:func:`SingleArmProfile.for_bridge` selects one; the default is ``"lzh"``, which is
the build currently running in this lab (``ROS_DOMAIN_ID=42``).

The side whitelist
------------------
The profile is the CLIENT-side half of the one-arm policy: it only ever names the
selected side's joints, group and Actions, and
:meth:`SingleArmProfile.validate_trajectory_joint_names` rejects a trajectory that
contains the other side's joints or both arms. The SERVER side of the policy is the
bridge's own gate (``enable_both_arms_real_execution``) plus the still-missing
side-specific real-execution flags -- see ``readme_g1d.md``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

ArmSide = Literal["left", "right"]

#: The two G1-D bridge builds present in this lab, keyed by name.
BRIDGE_NAMESPACES: dict[str, dict[str, str]] = {
    # The upstream package under ~/g1d_ros2_ws.
    "g1d": {
        "gripper_prefix": "",
        "dex1_motor_states": "/dex1/motor_states",
    },
    # This lab's own build under ~/g1d_ros2_lzh (the one currently running).
    "lzh": {
        "gripper_prefix": "lzh_",
        "dex1_motor_states": "/lzh_dex1/motor_states",
    },
}

#: The bridge build assumed when the caller does not say otherwise.
DEFAULT_BRIDGE_NAMESPACE = "lzh"

_LEFT_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
_RIGHT_JOINTS = tuple(name.replace("left_", "right_", 1) for name in _LEFT_JOINTS)


@dataclass(frozen=True)
class SingleArmProfile:
    """Names for exactly one G1-D arm and its matching Dex1 action."""

    side: ArmSide
    namespace: str = DEFAULT_BRIDGE_NAMESPACE

    def __post_init__(self) -> None:
        if self.side not in ("left", "right"):
            raise ValueError("side must be exactly 'left' or 'right'")
        if self.namespace not in BRIDGE_NAMESPACES:
            raise ValueError(
                f"namespace must be one of {sorted(BRIDGE_NAMESPACES)}, "
                f"got {self.namespace!r}"
            )

    @classmethod
    def for_bridge(cls, side: ArmSide, namespace: str = DEFAULT_BRIDGE_NAMESPACE) -> "SingleArmProfile":
        """The profile for ``side`` against a specific bridge build."""
        return cls(side=side, namespace=namespace)

    @property
    def _namespace(self) -> dict[str, str]:
        return BRIDGE_NAMESPACES[self.namespace]

    @property
    def group_name(self) -> str:
        return f"{self.side}_arm"

    @property
    def joint_names(self) -> tuple[str, ...]:
        return _LEFT_JOINTS if self.side == "left" else _RIGHT_JOINTS

    @property
    def trajectory_action(self) -> str:
        """The FollowJointTrajectory Action for this arm.

        IDENTICAL in both bridge builds -- the arm trajectory endpoint is the one
        that carries the motion, so it is deliberately NOT namespaced.
        """
        return f"/{self.side}_arm_controller/follow_joint_trajectory"

    @property
    def gripper_action(self) -> str:
        """The GripperCommand Action for this side's Dex1.

        The namespace is inserted BEFORE the side name (``/lzh_left_dex1/...``),
        which is how the ``lzh_sdk_bridge`` build names its servers.
        """
        prefix = self._namespace["gripper_prefix"]
        return f"/{prefix}{self.side}_dex1/gripper_command"

    @property
    def dex1_motor_states_topic(self) -> str:
        """The topic carrying this build's raw Dex1 motor feedback (radians)."""
        return self._namespace["dex1_motor_states"]

    @property
    def joint_states_topic(self) -> str:
        """The topic carrying the measured arm joints."""
        return "/joint_states"

    @property
    def wrist_link(self) -> str:
        """MoveIt SRDF tip currently in the checked-in G1-D config, not a TCP."""
        return f"{self.side}_wrist_yaw_link"

    @property
    def base_frame(self) -> str:
        """The frame the arm's poses are expressed in."""
        return "torso_link"

    def validate_trajectory_joint_names(self, names: list[str] | tuple[str, ...]) -> None:
        """Reject empty, duplicate, opposite-side, or multi-arm trajectories."""
        names = tuple(names)
        if len(names) != 7 or len(set(names)) != 7 or set(names) != set(self.joint_names):
            raise ValueError(
                f"{self.side} trajectory must contain exactly its 7 arm joints; got {names}"
            )
