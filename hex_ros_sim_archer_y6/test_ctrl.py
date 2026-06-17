#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import os

ROS_VERSION = os.environ.get('ROS_VERSION')

if ROS_VERSION == '1':
    from .test_utils.test_ctrl_ros1 import TestCtrl as TestCtrl
elif ROS_VERSION == '2':
    from .test_utils.test_ctrl_ros2 import TestCtrl as TestCtrl
else:
    raise ValueError("ROS_VERSION is not set")

__all__ = [
    "TestCtrl",
]


def main():
    ctrl = TestCtrl()
    try:
        ctrl.run()
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.shutdown()


if __name__ == '__main__':
    main()
