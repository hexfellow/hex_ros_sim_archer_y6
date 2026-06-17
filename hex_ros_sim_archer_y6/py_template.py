#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface


class PyTemplate:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("py_template")

        ### parameters
        self.__str_param = self.__data_interface.get_str_param()
        self.__int_param = self.__data_interface.get_int_param()
        print(f"str_param: {self.__str_param}")
        print(f"int_param: {self.__int_param}")

        ### variables
        self.__out_str = None
        self.__out_int = None

    def run(self):
        while self.__data_interface.ok():
            # Get data from ros every wake up
            total_str = ""
            for in_str in iter(self.__data_interface.get_in_str, None):
                print(f"in_str: {in_str}")
                total_str += in_str
            if total_str != "":
                self.__out_str = f"{self.__str_param['prefix']}: {total_str}"

            for in_int in iter(self.__data_interface.get_in_int, None):
                print(f"in_int: {in_int}")
                self.__out_int = in_int if in_int > self.__int_param['range'][
                    0] else self.__int_param['range'][0]
                self.__out_int = self.__out_int if self.__out_int < self.__int_param[
                    'range'][1] else self.__int_param['range'][1]

            if self.__out_str is not None:
                self.__data_interface.logi(f"out_str: {self.__out_str}")
                self.__data_interface.pub_out_str(self.__out_str)

            if self.__out_int is not None:
                self.__data_interface.logi(f"out_int: {self.__out_int}")
                self.__data_interface.pub_out_int(self.__out_int)

            self.__data_interface.sleep()


def main():
    py_template = PyTemplate()
    try:
        py_template.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
