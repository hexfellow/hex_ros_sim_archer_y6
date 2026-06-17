#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboArmCtrl,
    HexRosRoboGripCtrl,
    HexRosRoboManipCtrl,
    HexRosRoboManipCtrlStamped,
)

from .test_ctrl_base import (
    ArmCtrlMode,
    GripCtrlMode,
    CYCLE_PERIOD,
    TestCtrlBase,
    build_arm_ctrl,
    build_grip_ctrl,
)


class TestCtrl(TestCtrlBase, Node):

    def __init__(self):
        Node.__init__(self, 'test_ctrl')
        TestCtrlBase.__init__(self)

        # --- parameters ---
        self.declare_parameter('arm_mode', ArmCtrlMode.MIT.value)
        self.declare_parameter('grip_mode', GripCtrlMode.MIT.value)
        self.declare_parameter('frequency', 10.0)

        self._arm_mode = ArmCtrlMode(
            self.get_parameter('arm_mode').value)
        self._grip_mode = GripCtrlMode(
            self.get_parameter('grip_mode').value)
        self._freq = self.get_parameter('frequency').value

        self.logi(
            f"arm_mode={self._arm_mode.name}({self._arm_mode.value}), "
            f"grip_mode={self._grip_mode.name}({self._grip_mode.value}), "
            f"freq={self._freq} Hz")

        # --- publisher ---
        self._pub = self.create_publisher(
            HexRosRoboManipCtrlStamped, 'manip_ctrl', 10)

        # --- timer ---
        self._timer = self.create_timer(1.0 / self._freq, self._timer_cb)
        self._start_time = self.get_clock().now()

    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self):
        try:
            self.destroy_node()
        except Exception:
            pass

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

    ####################
    ### logging
    ####################
    def logi(self, msg: str):
        self.get_logger().info(msg)

    ####################
    ### timer callback
    ####################
    def _timer_cb(self):
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        cycle_idx = int(elapsed / CYCLE_PERIOD)

        ctrl = HexRosRoboManipCtrlStamped()
        ctrl.header = Header(stamp=now.to_msg())
        ctrl.manip_ctrl = HexRosRoboManipCtrl(
            arm_ctrl=build_arm_ctrl(
                self._arm_mode, cycle_idx,
                HexRosRoboArmCtrl, HexRosJnt,
                Vector3, Point, Quaternion, Pose,
            ),
            grip_ctrl=build_grip_ctrl(
                self._grip_mode, cycle_idx,
                HexRosRoboGripCtrl, HexRosJnt,
            ),
        )
        self._pub.publish(ctrl)
