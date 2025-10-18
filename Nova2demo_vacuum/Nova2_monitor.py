# -*- coding: utf-8 -*-
import time
import tkinter as tk
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_api import *
import json
from paho.mqtt import client as mqtt
import re
from datetime import datetime

import multiprocessing as mp
import multiprocessing.shared_memory
from multiprocessing import Process,Queue

import sys
import psutil

class Nova2_MON:
    def __init__(self,verbose=False):
        self.text_log = ""

    #connect button
    def my_connect(self):
        #feedback用
        self.client_feed = DobotApiFeedBack("192.168.5.1", 30004,self.text_log)
        #servoJ用
        #self.client_move = DobotApiDashboard("192.168.5.1", 30003, self.text_log)

        #logの表示
        #self.global_state["connect"] = not self.global_state["connect"]
        #self.text_log.insert(tk.END,"Connect!"+str(self.global_state["connect"])+"\n")

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

    def connect_mqtt(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        #MQTTの接続設定
        #self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect("sora2.uclab.jp", 1883, 60) #vr_demo
        self.client.loop_start()   # 通信処理開始

    def on_connect(self,client, userdata, flag, rc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
        #self.client.subscribe("webxr/pose") #　connected -> subscribe
        #self.client.subscribe("webxr/joint")
        self.log_txt("Connected MQTT"+"\n")

    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def monitor_start(self):
        last = 0
        angle_publish_last=0
        while True:    
            now = time.time()
            if last == 0:
                last = now
                angle_publish_last=now
                continue
            
            dt = (now - last)* 1000
           
            if(dt > 20):
                last = now

                feed_back = self.client_feed.feedBackData()
                current_time = datetime.now()
                formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")
                #print(feed_back[0][23]) # 現在の関節位置 float64
                #print(feed_back[0][24]) # 現在の関節速度
                now_angle = feed_back[0][23]
                v_angle = feed_back[0][24]
                tcp = feed_back[0][27]

                msg = json.dumps({
                    "j1": now_angle[0],
                    "j2": now_angle[1],
                    "j3": now_angle[2],
                    "j4": now_angle[3],
                    "j5": now_angle[4],
                    "j6": now_angle[5]
                })
                if( now - angle_publish_last >0.2) :
                    result = self.client.publish("webxr/now_angle", msg)
                    angle_publish_last = now

                msg = json.dumps({
                    "x":tcp[0],
                    "y":tcp[1],
                    "z":tcp[2],
                    "rot_x":tcp[3],
                    "rot_y":tcp[4],
                    "rot_z":tcp[5],
                    "time":formatted_time
                })
                result = self.client.publish("new_robot_pose", msg)
                sub_ID = int(self.pose[18])
                msg = json.dumps({
                    "ID": sub_ID
                })
                result = self.client.publish("robothand_control/current", msg)

                if(self.pose[23] == 1):
                    self.pose[23] = 0
                    msg = json.dumps({
                        "touched": True
                    })
                    result = self.client.publish("webxr/ObjectTouched", msg)

                self.pose[0:6] = now_angle
                self.pose[6:12] = v_angle
                

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
        self.pose = np.ndarray((26), dtype=np.dtype("float32"), buffer=self.sm.buf)
        
        self.pose[18] = 1
        self.pose[23] = 0
        
        self.init_realtime()
        self.my_connect()
        self.connect_mqtt()
        print("monitor -------------")
        
        try:
            self.monitor_start()
        except KeyboardInterrupt:
            self.client.loop_stop()   # 通信処理開始
            print("monitor mqtt loop stop")
