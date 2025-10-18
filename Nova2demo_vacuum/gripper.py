# -*- coding: utf-8 -*-
from threading import Thread
import time
import tkinter as tk
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_api import *
import json
import re
from datetime import datetime

import multiprocessing as mp
import multiprocessing.shared_memory
from multiprocessing import Process,Queue

import pymodbus.client
from pymodbus import FramerType, pymodbus_apply_logging_config


import sys
import psutil


class Gripper:
    def __init__(self):
        print("init")
        self.text_log = ""
    #connect button

    def init_realtime(self):
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)

    def main_loop(self):
        self.last = 0
        while self.loop:
            now = time.time()
            if self.last == 0:
                self.last = now
                print("Starting to Control!",self.pose)
                continue
            
            dt = (now - self.last)* 1000
 
            #ここからgripper関係　制御するように変更してからジョイントがガクガク 更新間隔を長くする -> 0.5sでもガクガクするので別プロセスにする？
            if(dt > 500):
                self.last = now
                
                gripper = self.pose[19] # here, 0-button open, 1-button close
                gripper_power = self.pose[21]
                gripper_min_pos = self.pose[22]
                gripper_min_pos = self.gripper_min_pos
                if 0 <= gripper and gripper <= 1:
                    gripper = 1 - gripper
                    gripper = int(gripper * 1000)
                    #下限の適用
                    if(gripper_min_pos > gripper):
                        gripper = gripper_min_pos
                    #print("gripper position to", gripper)
                    ret = self.modbus.write_register(0x103, gripper) # 0 (0x0000) - close,  (0x03E8) - open
                    
                """
                #gripper
                # read pos
                ret_pos = self.modbus.read_holding_registers(0x0202)
                if(ret_pos.isError()):
                    print("Reading registers returned an error!!!")
                else:
                    gripper_pos = ret_pos.registers[0]
                    #print(gripper_pos)

                # read status
                ret = self.modbus.read_holding_registers(0x0201)
                if(ret.isError()):
                    print("Reading registers returned an error!!!")
                else:
                    status = ret.registers[0]
                    #print(status)

                # set power
                if(self.last_power != gripper_power):
                    print(gripper_power)
                    #ret = self.modbus.write_register(0x0101, gripper_power)
                    print("--------------")
                    self.last_power = gripper_power

                # set min pos
                if(self.gripper_min_pos != gripper_min_pos):
                    self.gripper_min_pos = gripper_min_pos
                """
                
    def run_proc(self):
        #共有メモリにアクセス
        self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
        self.pose = np.ndarray((24,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.loop = True
        self.init_realtime()
        time.sleep(5)

        # needed????
        # load -> kg
        # center -> mm
        self.modbus = pymodbus.client.ModbusTcpClient(host="192.168.5.1", framer=FramerType.RTU, port=60000)
        self.modbus.connect()

        # full init(gripper moves) 0xA5
        # normal (grpper does not move) 0x01
        # when gripper close at initialize -> it opens
        self.modbus.write_register(0x100, 0x01) #初期化

        time.sleep(5)#sleepを挟まないと次のコマンドが正しい返答が来ない
        ret = self.modbus.write_register(0x0101, 0x14)
        self.last_power = 80
        print(ret)
        print("------------------ gripper start")

        self.gripper_min_pos = 0

        self.pose[21] = 80
        self.pose[22] = 0

        try:
            self.main_loop()
        except KeyboardInterrupt:
            self.modbus.close()
            print("modbus close")
            # need to close explicitly
            # when not closed, error at next time

