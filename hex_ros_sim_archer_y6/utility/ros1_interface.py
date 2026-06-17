#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import rospy

from std_msgs.msg import String
from std_msgs.msg import Int32

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rospy.init_node(name, anonymous=True)
        self._rate_param["ros"] = rospy.get_param('~rate_ros', 300.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### pamameter
        self._str_param = {
            "prefix": rospy.get_param('~str_prefix', ""),
        }
        self._int_param = {
            "range": rospy.get_param('~int_range', [-1_000_000, 1_000_000]),
        }

        ### publisher
        self.__out_str_pub = rospy.Publisher(
            'out_str',
            String,
            queue_size=10,
        )
        self.__out_int_pub = rospy.Publisher(
            'out_int',
            Int32,
            queue_size=10,
        )

        ### subscriber
        self.__in_str_sub = rospy.Subscriber(
            'in_str',
            String,
            self.__in_str_callback,
        )
        self.__in_int_sub = rospy.Subscriber(
            'in_int',
            Int32,
            self.__in_int_callback,
        )
        self.__in_str_sub
        self.__in_int_sub
        
        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

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
