#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from abc import ABC, abstractmethod
from enum import IntEnum


# ---------------------------------------------------------------------------
# control mode enums
# ---------------------------------------------------------------------------
class ArmCtrlMode(IntEnum):
    NONE = 0
    MIT = 1
    JNT = 2
    EE = 3


class GripCtrlMode(IntEnum):
    NONE = 0
    MIT = 1
    JNT = 2
    TAU = 3


# ---------------------------------------------------------------------------
# constants
# ---------------------------------------------------------------------------
ARM_DOF = 6
GRIP_DOF = 1

# joint position presets for arm (MIT / JNT modes)
ARM_POS_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_POS_EXTEND = [0.0, -0.8, -1.0, 0.0, 1.0, 0.0]
ARM_POS_RETRACT = [0.0, 0.8, 1.0, 0.0, -0.5, 0.0]
ARM_POS_PRESETS = [ARM_POS_HOME, ARM_POS_EXTEND, ARM_POS_RETRACT]

# end-effector pose presets for arm (EE mode) — [position], [quaternion xyzw]
ARM_EE_POSE_A = ([0.3, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0])
ARM_EE_POSE_B = ([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
ARM_EE_PRESETS = [ARM_EE_POSE_A, ARM_EE_POSE_B]

# grip position presets (MIT mode)
GRIP_POS_OPEN = [0.0]
GRIP_POS_CLOSE = [0.03]
GRIP_MIT_PRESETS = [GRIP_POS_OPEN, GRIP_POS_CLOSE]

# grip effort presets (JNT / TAU modes)
GRIP_EFF_LIGHT = [5.0]
GRIP_EFF_HEAVY = [20.0]
GRIP_EFF_PRESETS = [GRIP_EFF_LIGHT, GRIP_EFF_HEAVY]

# default gains — consistent with MujocoSim defaults
ARM_KP = [400.0, 400.0, 500.0, 200.0, 100.0, 100.0]
ARM_KD = [5.0, 5.0, 5.0, 5.0, 2.0, 2.0]
ARM_LIM_VEL = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
ARM_LIM_ACC = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]

GRIP_KP = [10.0]
GRIP_KD = [0.5]
GRIP_LIM_VEL = [0.5]
GRIP_LIM_ACC = [1.0]

# how long to hold each preset before advancing [s]
CYCLE_PERIOD = 3.0


# ---------------------------------------------------------------------------
# helpers – construct ros message fields
# ---------------------------------------------------------------------------
def _zeros_arm_jnt(jnt_cls):
    return jnt_cls(
        pos=[0.0] * ARM_DOF,
        vel=[0.0] * ARM_DOF,
        eff=[0.0] * ARM_DOF,
        kp=ARM_KP,
        kd=ARM_KD,
        lim_vel=ARM_LIM_VEL,
        lim_acc=ARM_LIM_ACC,
    )


def _zeros_grip_jnt(jnt_cls):
    return jnt_cls(
        pos=[0.0] * GRIP_DOF,
        vel=[0.0] * GRIP_DOF,
        eff=[0.0] * GRIP_DOF,
        kp=GRIP_KP,
        kd=GRIP_KD,
        lim_vel=GRIP_LIM_VEL,
        lim_acc=GRIP_LIM_ACC,
    )


def build_arm_ctrl(mode: ArmCtrlMode, cycle_idx: int, arm_cls, jnt_cls,
                   vec3_cls, pt_cls, quat_cls, pose_cls):
    ctrl = arm_cls()
    ctrl.ctrl_mode = mode.value
    ctrl.grav = vec3_cls(x=0.0, y=0.0, z=0.0)
    ctrl.pose = pose_cls(
        position=pt_cls(x=0.0, y=0.0, z=0.0),
        orientation=quat_cls(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    ctrl.jnt = _zeros_arm_jnt(jnt_cls)

    if mode == ArmCtrlMode.NONE:
        return ctrl

    if mode == ArmCtrlMode.MIT:
        preset = ARM_POS_PRESETS[cycle_idx % len(ARM_POS_PRESETS)]
        ctrl.jnt.pos = list(preset)

    elif mode == ArmCtrlMode.JNT:
        preset = ARM_POS_PRESETS[cycle_idx % len(ARM_POS_PRESETS)]
        ctrl.jnt.pos = list(preset)
        # clear gains — simulator applies its own defaults in JNT mode
        ctrl.jnt.kp = [0.0] * ARM_DOF
        ctrl.jnt.kd = [0.0] * ARM_DOF

    elif mode == ArmCtrlMode.EE:
        pos, quat = ARM_EE_PRESETS[cycle_idx % len(ARM_EE_PRESETS)]
        ctrl.pose = pose_cls(
            position=pt_cls(x=pos[0], y=pos[1], z=pos[2]),
            orientation=quat_cls(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )
        ctrl.jnt.kp = [0.0] * ARM_DOF
        ctrl.jnt.kd = [0.0] * ARM_DOF

    return ctrl


def build_grip_ctrl(mode: GripCtrlMode, cycle_idx: int, grip_cls, jnt_cls):
    ctrl = grip_cls()
    ctrl.ctrl_mode = mode.value
    ctrl.jnt = _zeros_grip_jnt(jnt_cls)

    if mode == GripCtrlMode.NONE:
        return ctrl

    if mode == GripCtrlMode.MIT:
        preset = GRIP_MIT_PRESETS[cycle_idx % len(GRIP_MIT_PRESETS)]
        ctrl.jnt.pos = list(preset)

    elif mode == GripCtrlMode.JNT:
        preset = GRIP_EFF_PRESETS[cycle_idx % len(GRIP_EFF_PRESETS)]
        ctrl.jnt.eff = list(preset)
        ctrl.jnt.lim_vel = [0.5]

    elif mode == GripCtrlMode.TAU:
        preset = GRIP_EFF_PRESETS[cycle_idx % len(GRIP_EFF_PRESETS)]
        ctrl.jnt.eff = list(preset)
        ctrl.jnt.lim_vel = [0.5]

    return ctrl


# ---------------------------------------------------------------------------
# abstract base
# ---------------------------------------------------------------------------
class TestCtrlBase(ABC):

    def __init__(self):
        self._arm_mode = ArmCtrlMode.MIT
        self._grip_mode = GripCtrlMode.MIT
        self._freq = 10.0

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("TestCtrlBase.ok")

    @abstractmethod
    def run(self):
        raise NotImplementedError("TestCtrlBase.run")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("TestCtrlBase.shutdown")

    @abstractmethod
    def logi(self, msg: str):
        raise NotImplementedError("TestCtrlBase.logi")
