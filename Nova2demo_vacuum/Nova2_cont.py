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


class Nova2_CON:
    def __init__(self):
        print("init")
        self.text_log = ""
    #connect button

    def my_connect(self):
        #servoJ用
        self.client_move = DobotApiDashboard("192.168.5.1", 30003, self.text_log)

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
        #dif_max * x_k = 4ぐらい？
        dif_max = 16
        x_k = 0.25

        default_pos_x_k = 0.05
        normmal_x_k = 0.25

        self.last = 0
        self.gripper_last = 0
        self.last_angle = [0,0,0,0,0,0]
        default_pos_flag = False
        while self.loop:
            # 現在情報を取得しているかを確認
            if self.pose[0:11].sum() == 0:
                time.sleep(0.3)
                print("Wait for monitoring..")
                continue

            #目標値のMQTTをサブスクライブしているか
            if self.pose[12:17].sum() == 0:
                time.sleep(0.3)
                print("Wait for target..")
                continue 
            
            now = time.time()
            if self.last == 0:
                self.last = now
                self.last_angle = self.pose[0:6].tolist()
                print("Starting to Control!",self.pose)
                continue
            
            dt = (now - self.last)* 1000            
            if(dt > 20):
                self.last = now
                now_angle = self.pose[0:6].tolist()
                #v_angle = self.pose[6:12].tolist() #feedback port おそらく三次元座標系での速度
                v_angle=[0,0,0,0,0,0]
                for i in range(6):
                    v_angle[i] = (now_angle[i] - self.last_angle[i]) / dt * 1000
                self.last_angle = now_angle
                in_angle = self.pose[12:].tolist()
                if(self.pose[20] == 1):
                    default_pos_flag = True

                dif_angle = [0,0,0,0,0,0]
                dif_max_num = 1
                goal_angle = [0,0,0,0,0,0]
                goal_dif = [0,0,0,0,0,0]
                
                if(default_pos_flag):
                    x_k = default_pos_x_k
                    default_pos_flag = False
                    self.pose[20] = 0
                    print("target fefault pose")
                
                for i in range(6):
                    dif_angle[i] = in_angle[i] - now_angle[i]
                    goal_dif[i] = dif_angle[i] * x_k #p制御
                    
                    if abs(goal_dif[i]) > abs(goal_dif[dif_max_num]):
                        dif_max_num = i # 最大変化ジョイント

                rounded_goal_dif = [round(num,3) for num in goal_dif]

                is_all_small = all(abs(x) <= 1 for x in rounded_goal_dif)
                if is_all_small:
                    x_k = normmal_x_k

                msg_angle = ["", "", "", "", "", ""]

                for i in range(6): 
                    goal_angle[i] = now_angle[i] + goal_dif[i]
                    msg_angle[i] = str(goal_angle[i])                
                ret = self.client_move.sendRecvMsg("ServoJ("+msg_angle[0]+","+msg_angle[1]+","+msg_angle[2]+","+msg_angle[3]+","+msg_angle[4]+","+msg_angle[5]+")")


                
           
    def run_proc(self):
        #共有メモリにアクセス
        self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
        self.pose = np.ndarray((26), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.my_connect()
        self.loop = True
        self.init_realtime()
        time.sleep(2)

        try:
            self.main_loop()
        except KeyboardInterrupt:
            print("stop cont")
            #self.modbus.close()
            # need to close explicitly
            # when not closed, error at next time