#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import json
from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod


class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._rate_param = {}
        self._str_param = {}
        self._int_param = {}

        ### rx msg queues
        self._in_str_deque = deque()
        self._in_int_deque = deque()

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
    def _str_to_list(self, list_str: list) -> list:
        result = []
        for s in list_str:
            l = json.loads(s)
            result.append(l)
        return result

    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_str_param(self) -> dict:
        return self._str_param

    def get_int_param(self) -> dict:
        return self._int_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_out_str(self, out: str):
        raise NotImplementedError("InterfaceBase.pub_out_str")

    @abstractmethod
    def pub_out_int(self, out: int):
        raise NotImplementedError("InterfaceBase.pub_out_int")

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
            ret = dq[-1]
            dq.clear()
            return ret

    # in str
    def get_in_str(self, latest: bool = False) -> Optional[str]:
        return self.deque_helper(self._in_str_deque, latest)

    # in int
    def get_in_int(self, latest: bool = False) -> Optional[int]:
        return self.deque_helper(self._in_int_deque, latest)
