#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipStateStamped,
)

JOINT_STATE_NAME = [f"joint_{i}" for i in range(1, 7)] + ["gp100_joint_1"]


class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._rate_param = {}
        self._model_param = {}
        self._prog_param = {}

        ### rx msg queues
        self._manip_ctrl_deque = deque(maxlen=100)

        ### name
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_model_param(self) -> dict:
        return self._model_param

    def get_prog_param(self) -> dict:
        return self._prog_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_manip_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("InterfaceBase.pub_manip_state")

    @abstractmethod
    def pub_joint_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("InterfaceBase.pub_joint_state")

    @abstractmethod
    def pub_clock(self, stamp_ns: int):
        raise NotImplementedError("InterfaceBase.pub_clock")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # manip ctrl
    def get_manip_ctrl(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboManipCtrlStamped]:
        return self.deque_helper(self._manip_ctrl_deque, latest)
