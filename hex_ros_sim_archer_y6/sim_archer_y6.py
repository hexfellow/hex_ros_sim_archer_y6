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
from mujoco_sim import MujocoSim


class SimArcherY6:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("sim_archer_y6")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__prog_param = self.__data_interface.get_prog_param()
        self.__data_interface.logi(f"sim rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"state rate: {self.__rate_param['state']} hz")
        self.__data_interface.logi(f"model urdf: {self.__model_param['urdf']}")
        self.__data_interface.logi(f"model mjcf: {self.__model_param['mjcf']}")
        self.__data_interface.logi(
            f"model frame id: {self.__model_param['frame_id']}")

        ### simulator
        self.__sim = MujocoSim(
            model_urdf=self.__model_param["urdf"],
            model_mjcf=self.__model_param["mjcf"],
            sim_rate=self.__rate_param["ros"],
            prog_viewer=self.__prog_param["viewer"],
            frame_id=self.__model_param["frame_id"],
        )

        ### derived
        self.__state_decim = max(
            1,
            int(round(self.__rate_param["ros"] / self.__rate_param["state"])),
        )
        self.__viewer_decim = max(
            1,
            int(round(self.__rate_param["ros"] / 60.0)),
        )

    def run(self):
        state_count = 0
        viewer_count = 0
        while self.__data_interface.ok():
            # 1. always drain to the latest control frame
            ctrl = self.__data_interface.get_manip_ctrl(latest=True)
            if ctrl is not None:
                self.__sim.update_manip_ctrl(ctrl)

            # 2. advance the simulation by one step
            self.__sim.step()
            self.__data_interface.pub_clock(self.__sim.sim_time_ns())

            # 3. publish robot state at the requested rate
            state_count += 1
            if state_count >= self.__state_decim:
                state_count = 0
                manip_state = self.__sim.get_manip_state()
                self.__data_interface.pub_manip_state(manip_state)
                if self.__prog_param["rviz"]:
                    self.__data_interface.pub_joint_state(manip_state)

            # 4. render viewer at ~60 hz
            if self.__prog_param["viewer"]:
                viewer_count += 1
                if viewer_count >= self.__viewer_decim:
                    viewer_count = 0
                    self.__sim.sync_viewer()

            self.__data_interface.sleep()

    def shutdown(self):
        try:
            self.__sim.close()
        except Exception:
            pass
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass


def main():
    sim_archer_y6 = SimArcherY6()
    try:
        sim_archer_y6.run()
    except KeyboardInterrupt:
        pass
    finally:
        sim_archer_y6.shutdown()


if __name__ == '__main__':
    main()
