# -*- coding: utf-8 -*-
from threading import Thread
import time
import tkinter as tk
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_api import *
from datetime import datetime

import multiprocessing as mp
import multiprocessing.shared_memory
from multiprocessing import Process,Queue

import pymodbus.client
from pymodbus import FramerType, pymodbus_apply_logging_config


import sys
import psutil


class VACUUM:
    def __init__(self):
        print("init va")
        self.client_tool = DobotApiDashboard("192.168.5.1", 29999, self.text_log)
        print("init vacuum")
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
                print("Starting to Control Vacuum")
                continue
            
            dt = (now - self.last)* 1000
 
            if(dt > 500):
                self.last = now
                
                vacuum = self.pose[19] # here, 0-button open, 1-button close
                print(vacuum)
                if vacuum > 0:
                    ret = self.client_tool.sendRecvMsg("ToolDOExecute(1,1)")
                if vacuum == 0: 
                    ret = self.client_tool.sendRecvMsg("ToolDOExecute(1,0)")
                    
           
                
    def run_proc(self):
        #共有メモリにアクセス
        self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
        self.pose = np.ndarray((25,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.loop = True
        self.init_realtime()

        try:
            self.main_loop()
        except KeyboardInterrupt:
            self.modbus.close()
            print("vacuum close")
            # need to close explicitly
            # when not closed, error at next time

