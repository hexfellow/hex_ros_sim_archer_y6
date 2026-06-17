#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import threading

import rospy
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


class TestCtrl(TestCtrlBase):

    def __init__(self):
        TestCtrlBase.__init__(self)

        # --- ros node ---
        rospy.init_node('test_ctrl', anonymous=True)

        # --- parameters ---
        self._arm_mode = ArmCtrlMode(
            rospy.get_param('~arm_mode', ArmCtrlMode.MIT.value))
        self._grip_mode = GripCtrlMode(
            rospy.get_param('~grip_mode', GripCtrlMode.MIT.value))
        self._freq = rospy.get_param('~frequency', 10.0)

        self.logi(
            f"arm_mode={self._arm_mode.name}({self._arm_mode.value}), "
            f"grip_mode={self._grip_mode.name}({self._grip_mode.value}), "
            f"freq={self._freq} Hz")

        # --- publisher ---
        self._pub = rospy.Publisher(
            'manip_ctrl',
            HexRosRoboManipCtrlStamped,
            queue_size=10,
        )

        # --- timer ---
        self._start_time = rospy.Time.now()
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._freq), self._timer_cb)

        # --- spin thread ---
        self._shutting_down = False
        self._spin_thread = threading.Thread(target=self._spin)
        self._spin_thread.start()

    def _spin(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def ok(self) -> bool:
        return not rospy.is_shutdown()

    def shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self._timer.shutdown()
        except Exception:
            pass
        self._spin_thread.join()

    def run(self):
        try:
            self._spin_thread.join()
        except KeyboardInterrupt:
            pass

    ####################
    ### logging
    ####################
    def logi(self, msg: str):
        rospy.loginfo(msg)

    ####################
    ### timer callback
    ####################
    def _timer_cb(self, event):
        now = rospy.Time.now()
        elapsed = (now - self._start_time).to_sec()
        cycle_idx = int(elapsed / CYCLE_PERIOD)

        ctrl = HexRosRoboManipCtrlStamped()
        ctrl.header = Header(stamp=now)
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
