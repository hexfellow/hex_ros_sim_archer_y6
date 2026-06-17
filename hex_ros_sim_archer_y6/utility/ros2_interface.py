#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import rclpy
import rclpy.node
import threading

from std_msgs.msg import String
from std_msgs.msg import Int32

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 300.0)
        self._rate_param["ros"] = self.__node.get_parameter('rate_ros').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### pamameter
        # declare parameters
        self.__node.declare_parameter('str_prefix', "")
        self.__node.declare_parameter('int_range', [-1_000_000, 1_000_000])
        # str
        self._str_param = {
            "prefix": self.__node.get_parameter('str_prefix').value,
        }
        # int
        self._int_param = {
            "range": self.__node.get_parameter('int_range').value,
        }

        ### publisher
        self.__out_str_pub = self.__node.create_publisher(
            String,
            'out_str',
            10,
        )
        self.__out_int_pub = self.__node.create_publisher(
            Int32,
            'out_int',
            10,
        )

        ### subscriber
        self.__in_str_sub = self.__node.create_subscription(
            String,
            'in_str',
            self.__in_str_callback,
            10,
        )
        self.__in_int_sub = self.__node.create_subscription(
            Int32,
            'in_int',
            self.__in_int_callback,
            10,
        )
        self.__in_str_sub
        self.__in_int_sub

        ### spin thread
        self.__shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        if self.__shutting_down:
            return
        self.__shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    def pub_out_str(self, out: str):
        msg = String()
        msg.data = out
        self.__out_str_pub.publish(msg)

    def pub_out_int(self, out: int):
        msg = Int32()
        msg.data = out
        self.__out_int_pub.publish(msg)

    def __in_str_callback(self, msg: String):
        self._in_str_deque.append(msg.data)

    def __in_int_callback(self, msg: Int32):
        self._in_int_deque.append(msg.data)
