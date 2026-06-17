#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import mujoco
import numpy as np
from mujoco import viewer

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntState,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrlMode,
    HexDcRoboArmCtrl,
    HexDcRoboGripCtrl,
    HexDcRoboArmState,
    HexDcRoboGripState,
    HexDcRoboManipState,
    HexDcRoboManipStateStamped,
    HexDcRoboManipCtrlStamped,
)
from hex_util_ros import HexDynUtilY6
from hex_util_ros import arm_pos_limit, grip_pos_limit, interp_joint

# Default gains used when only a position target is given (JNT mode).
ARM_KP_DEFAULT = np.array([400.0, 400.0, 500.0, 200.0, 100.0, 100.0])
ARM_KD_DEFAULT = np.array([5.0, 5.0, 5.0, 5.0, 2.0, 2.0])
GRIP_KP_DEFAULT = np.array([10.0])
GRIP_KD_DEFAULT = np.array([0.5])


class MujocoSim:

    def __init__(
        self,
        model_urdf: str,
        model_mjcf: str,
        sim_rate: float = 1000.0,
        prog_viewer: bool = True,
        frame_id: str = "base_link",
    ) -> None:
        self.__model_urdf = model_urdf
        self.__model_mjcf = model_mjcf
        self.__sim_rate = float(sim_rate)
        self.__prog_viewer = bool(prog_viewer)
        self.__frame_id = frame_id

        self.__init_vars()
        self.__init_mujoco()
        self.reset()

    ####################
    ### init helpers
    ####################
    def __init_vars(self) -> None:
        # idx
        self.__idx_dict = {
            "arm": slice(0, 6),
            "grip": slice(6, 7),
        }

        # viewer (synced from the run loop at ~60 hz)
        self.__viewer = None

        # comp
        self.__dyn_util = HexDynUtilY6(
            model_path=self.__model_urdf,
            last_link="gp100_base_link",
        )

        # cur
        self.__cur_ctrl = {
            "arm": None,
            "grip": None,
        }
        self.__cur_state = {
            "arm": None,
            "grip": None,
        }
        self.__cur_grav = None
        self.__cur_comp = None

    def __init_mujoco(self) -> None:
        # model init
        self.__model = mujoco.MjModel.from_xml_path(self.__model_mjcf)
        self.__data = mujoco.MjData(self.__model)
        self.__model.opt.timestep = 1.0 / self.__sim_rate
        mujoco.mj_resetData(self.__model, self.__data)

        # limits
        self.__arm_limits = self.__model.jnt_range[
            self.__idx_dict["arm"], :].copy().reshape(-1, 1, 2)
        self.__grip_limits = self.__model.jnt_range[
            self.__idx_dict["grip"], :].copy().reshape(-1, 1, 2)

        # state init
        keyframe_id = mujoco.mj_name2id(
            self.__model,
            mujoco.mjtObj.mjOBJ_KEY,
            "home",
        )
        self.__state_init = {
            "qpos": self.__model.key_qpos[keyframe_id],
            "qvel": np.zeros_like(self.__data.qvel),
            "ctrl": np.zeros_like(self.__data.ctrl),
        }
        self.__data.qpos = self.__state_init["qpos"]
        self.__data.qvel = self.__state_init["qvel"]
        self.__data.ctrl = self.__state_init["ctrl"]

        # current control
        arm_dof = self.__idx_dict["arm"].stop - self.__idx_dict["arm"].start
        grip_dof = self.__idx_dict["grip"].stop - self.__idx_dict["grip"].start
        self.__cur_ctrl = {
            "arm":
            HexDcRoboArmCtrl(
                ctrl_mode=HexDcRoboArmCtrlMode.MIT,
                grav=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
                jnt=HexDcBaseJntFull(
                    pos=self.__state_init["qpos"][self.__idx_dict["arm"]],
                    vel=np.zeros(arm_dof),
                    eff=np.zeros(arm_dof),
                    kp=ARM_KP_DEFAULT,
                    kd=ARM_KD_DEFAULT,
                    lim_vel=np.zeros(arm_dof),
                    lim_acc=np.zeros(arm_dof),
                ),
                pose=HexDcBasePose(
                    position=HexDcBaseVector3(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                    ),
                    orientation=HexDcBaseQuaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0,
                    ),
                ),
            ),
            "grip":
            HexDcRoboGripCtrl(
                ctrl_mode=HexDcRoboGripCtrlMode.MIT,
                jnt=HexDcBaseJntFull(
                    pos=self.__state_init["qpos"][self.__idx_dict["grip"]],
                    vel=np.zeros(grip_dof),
                    eff=np.zeros(grip_dof),
                    kp=GRIP_KP_DEFAULT,
                    kd=GRIP_KD_DEFAULT,
                    lim_vel=np.zeros(grip_dof),
                    lim_acc=np.zeros(grip_dof),
                ),
            ),
        }

        # current state
        self.__cur_state = {
            "arm":
            HexDcBaseJntState(
                position=self.__data.qpos[self.__idx_dict["arm"]],
                velocity=self.__data.qvel[self.__idx_dict["arm"]],
                effort=self.__data.qfrc_actuator[self.__idx_dict["arm"]],
            ),
            "grip":
            HexDcBaseJntState(
                position=self.__data.qpos[self.__idx_dict["grip"]],
                velocity=self.__data.qvel[self.__idx_dict["grip"]],
                effort=self.__data.qfrc_actuator[self.__idx_dict["grip"]],
            ),
        }

        # compensation
        self.__cur_comp = self.__dyn_util.compensation(
            self.__data.qpos[self.__idx_dict["arm"]],
            self.__data.qvel[self.__idx_dict["arm"]],
        )

        # viewer init
        mujoco.mj_forward(self.__model, self.__data)
        self.__viewer = None
        if self.__prog_viewer:
            self.__viewer = viewer.launch_passive(self.__model, self.__data)

    def __deinit_mujoco(self) -> None:
        if self.__viewer is not None:
            self.__viewer.close()
            self.__viewer = None

    ####################
    ### lifecycle
    ####################
    def reset(self) -> None:
        mujoco.mj_resetData(self.__model, self.__data)
        self.__data.qpos[:] = self.__state_init["qpos"]
        self.__data.qvel[:] = self.__state_init["qvel"]
        self.__data.ctrl[:] = self.__state_init["ctrl"]
        mujoco.mj_forward(self.__model, self.__data)
        if self.__viewer is not None:
            self.__viewer.sync()

    def step(self) -> None:
        self.__apply_manip_ctrl()
        mujoco.mj_step(self.__model, self.__data)
        self.__cur_state = {
            "arm":
            HexDcBaseJntState(
                position=self.__data.qpos[self.__idx_dict["arm"]],
                velocity=self.__data.qvel[self.__idx_dict["arm"]],
                effort=self.__data.qfrc_actuator[self.__idx_dict["arm"]],
            ),
            "grip":
            HexDcBaseJntState(
                position=self.__data.qpos[self.__idx_dict["grip"]],
                velocity=self.__data.qvel[self.__idx_dict["grip"]],
                effort=self.__data.qfrc_actuator[self.__idx_dict["grip"]],
            ),
        }
        self.__comp_grav = self.__dyn_util.compensation(
            self.__cur_state["arm"].position,
            self.__cur_state["arm"].velocity,
        )

    def sync_viewer(self) -> None:
        if self.__viewer is not None:
            self.__viewer.sync()

    def close(self) -> None:
        self.__deinit_mujoco()

    def sim_time_ns(self) -> int:
        return int(round(self.__data.time * 1e9))

    ####################
    ### control
    ####################
    def update_manip_ctrl(self, ctrl: HexDcRoboManipCtrlStamped) -> None:
        # arm ctrl
        if ctrl.manip_ctrl.arm_ctrl.ctrl_mode is not HexDcRoboArmCtrlMode.NONE:
            self.__cur_ctrl["arm"] = ctrl.manip_ctrl.arm_ctrl
            self.__dyn_util.set_gravity(ctrl.manip_ctrl.arm_ctrl.grav)
        if ctrl.manip_ctrl.grip_ctrl.ctrl_mode is not HexDcRoboGripCtrlMode.NONE:
            self.__cur_ctrl["grip"] = ctrl.manip_ctrl.grip_ctrl

    def __apply_manip_ctrl(self) -> None:
        if self.__cur_ctrl["arm"] is not None:
            self.__apply_arm_ctrl(self.__cur_ctrl["arm"])
        if self.__cur_ctrl["grip"] is not None:
            self.__apply_grip_ctrl(self.__cur_ctrl["grip"])

    def __apply_arm_ctrl(self, arm_ctrl: HexDcRoboArmCtrl) -> None:
        mode = arm_ctrl.ctrl_mode
        ctrl_jnt = arm_ctrl.jnt
        ctrl_pose = arm_ctrl.pose
        cur_jnt = self.__cur_state["arm"]

        if mode == HexDcRoboArmCtrlMode.MIT:
            tar_pos = arm_pos_limit(
                ctrl_jnt.pos,
                self.__arm_limits[:, 0, 0],
                self.__arm_limits[:, 0, 1],
            )
            tau_cmds = ctrl_jnt.kp * (
                tar_pos - cur_jnt.position) + ctrl_jnt.kd * (
                    ctrl_jnt.vel - cur_jnt.velocity) + self.__cur_comp

        elif mode == HexDcRoboArmCtrlMode.JNT:
            tar_pos = arm_pos_limit(
                ctrl_jnt.pos,
                self.__arm_limits[:, 0, 0],
                self.__arm_limits[:, 0, 1],
            )
            mid_pos = interp_joint(
                ctrl_jnt.pos,
                cur_jnt.position,
                # not a right formulation, just for test
                err_limit=ctrl_jnt.lim_vel * 1e-2,
            )
            tau_cmds = ARM_KP_DEFAULT * (
                mid_pos - cur_jnt.position) - ARM_KD_DEFAULT * (
                    ctrl_jnt.vel - cur_jnt.velocity) + self.__cur_comp

        elif mode == HexDcRoboArmCtrlMode.EE:
            ik_success, tar_pos = self.__dyn_util.inverse_kinematics_analytic(
                (ctrl_pose.position, ctrl_pose.orientation),
                cur_jnt.position,
            )
            if not ik_success:
                print("Inverse kinematics failed")
                return
            tar_pos = arm_pos_limit(
                tar_pos,
                self.__arm_limits[:, 0, 0],
                self.__arm_limits[:, 0, 1],
            )
            mid_pos = interp_joint(
                ctrl_jnt.pos,
                cur_jnt.position,
                # not a right formulation, just for test
                err_limit=ctrl_jnt.lim_vel * 1e-2,
            )
            tau_cmds = ARM_KP_DEFAULT * (
                mid_pos - cur_jnt.position) - ARM_KD_DEFAULT * (
                    ctrl_jnt.vel - cur_jnt.velocity) + self.__cur_comp

        else:
            raise ValueError(f"Unsupported arm control mode: {mode}")

        self.__data.ctrl[self.__idx_dict["arm"]] = tau_cmds

    def __apply_grip_ctrl(self, grip_ctrl: HexDcRoboGripCtrl) -> None:
        mode = int(grip_ctrl.ctrl_mode)
        ctrl_jnt = grip_ctrl.jnt
        cur_jnt = self.__cur_state["grip"]

        if mode == int(HexDcRoboGripCtrlMode.MIT):
            tar_pos = grip_pos_limit(
                ctrl_jnt.pos,
                self.__grip_limits[:, 0, 0],
                self.__grip_limits[:, 0, 1],
            )
            tau_cmds = ctrl_jnt.kp * (
                tar_pos - cur_jnt.position) + ctrl_jnt.kd * (
                    ctrl_jnt.vel - cur_jnt.velocity) + ctrl_jnt.eff

        elif mode == int(HexDcRoboGripCtrlMode.JNT):
            tau_abs = np.fabs(ctrl_jnt.eff)
            kd = tau_abs / ctrl_jnt.lim_vel
            pos_err = tar_pos - cur_jnt.position
            grip_tau = np.sign(pos_err) * tau_abs - kd * cur_jnt.velocity
            tau_cmds = np.clip(np.fabs(pos_err * 1e1), 0.0, 1.0) * grip_tau

        elif mode == int(HexDcRoboGripCtrlMode.TAU):
            kd = ctrl_jnt.eff / ctrl_jnt.lim_vel
            tau_cmds = ctrl_jnt.eff - kd * cur_jnt.velocity

        else:
            raise ValueError(f"Unsupported gripper control mode: {mode}")

        # both mimic actuators share the same command
        tau_full = np.repeat(
            tau_cmds,
            self.__idx_dict["grip"].stop - self.__idx_dict["grip"].start)
        self.__data.ctrl[self.__idx_dict["grip"]] = tau_full

    ####################
    ### state
    ####################
    def get_manip_state(self) -> HexDcRoboManipStateStamped:
        ts_ns = self.sim_time_ns()
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(ts_ns // 1_000_000_000),
                nsecs=int(ts_ns % 1_000_000_000),
            ),
            frame_id=self.__frame_id,
        )

        # arm ee pose
        ee_pos, ee_quat = self.__dyn_util.forward_kinematics(
            self.__cur_state["arm"].position)[-1]
        arm_pose = HexDcBasePose(
            position=HexDcBaseVector3(
                x=float(ee_pos[0]),
                y=float(ee_pos[1]),
                z=float(ee_pos[2]),
            ),
            orientation=HexDcBaseQuaternion(
                x=float(ee_quat[1]),
                y=float(ee_quat[2]),
                z=float(ee_quat[3]),
                w=float(ee_quat[0]),
            ),
        )

        # manip state
        manip_state = HexDcRoboManipState(
            arm_state=HexDcRoboArmState(
                jnt=self.__cur_state["arm"],
                pose=arm_pose,
            ),
            grip_state=HexDcRoboGripState(jnt=self.__cur_state["grip"]),
        )
        return HexDcRoboManipStateStamped(
            header=header,
            manip_state=manip_state,
        )
