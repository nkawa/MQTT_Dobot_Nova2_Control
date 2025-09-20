# -*- coding: utf-8 -*-
from threading import Thread
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

class Nova2_MQTT:
    def __init__(self):
        self.sub_topic = "webxr/VR1/joint"
        self.last_topic_num = 0
 #       self.log = open(fname,"w")

    #def init_rtde(self):
    #    self.rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

    def on_connect(self,client, userdata, flag, rc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
        #self.client.subscribe("lss4dof/state") #　connected -> subscribe
        self.client.subscribe("webxr/VR1/joint")
        self.client.subscribe("webxr/VR1/joint_250ms")
        self.client.subscribe("webxr/VR1/joint_500ms")
        self.client.subscribe("webxr/VR2/joint")
        self.client.subscribe("webxr/VR3/joint")
        self.client.subscribe("robothand_control/result")
        self.client.subscribe("webxr/VR_controller")
        self.client.subscribe("robothand_control/gripper_power")
        self.client.subscribe("robothand_control/gripper_min_pos")
        

    #ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
        if(self.last_topic_num != self.pose[24]):
            self.last_topic_num = self.pose[24]
            if(self.pose[24] == 0):
                self.sub_topic = "webxr/VR1/joint"
            if(self.pose[24] == 1.0):
                self.sub_topic = "webxr/VR1/joint_250ms"
            if(self.pose[24] == 2.0):
                self.sub_topic = "webxr/VR1/joint_500ms"
            print("now topic is ", self.sub_topic)
        if(msg.topic == self.sub_topic):
            js = json.loads(msg.payload)
            #print("Message!",js)
            #jointをsubscribeする
            angle = [0,0,0,0,0,0]
            if 'j1' in js:
                angle[0] = js['j1']
                angle[1] = js['j2']
                angle[2] = js['j3']
                angle[3] = js['j4']
                angle[4] = js['j5']
                angle[5] = js['j6']

                angle[1] = -angle[1]
                angle[2] = -angle[2]
                angle[3] = -angle[3]
                angle[5] = -angle[5] ##カメラが横についているため
                self.pose[12:18] = angle #12個の内後半6個が目標値
            if 'defaultFlag' in js:
                if js['defaultFlag']:
                    self.pose[20] = 1
            else:
                print("JSON",js)
                return
            
            
        if(msg.topic == "robothand_control/result"):
            js = json.loads(msg.payload)
            print(js,"change")
            if(js['ID'] == 1):
                self.sub_topic = "webxr/VR1/joint"
                self.pose[18] = 1
                print("change topic to 1")
            elif(js['ID'] == 2):
                self.sub_topic = "webxr/VR2/joint"
                self.pose[18] = 2
                print("change topic to 2")
            elif(js['ID'] == 3):
                self.sub_topic = "webxr/VR3/joint"
                self.pose[18] = 3
                print("change topic to 3")
        
        if(msg.topic == "webxr/VR_controller"):
            js = json.loads(msg.payload)
            if 'gripvalue' in js:
                gripper = js['gripvalue'] #gripper制御用
                self.pose[19] = gripper

        """
        if(msg.topic == "robothand_control/gripper_power"):
            js = json.loads(msg.payload)
            if 'power' in js:
                gripper = js['power'] #gripper制御用
                self.pose[21] = gripper
        
        if(msg.topic == "robothand_control/gripper_min_pos"):
            js = json.loads(msg.payload)
            if 'pos' in js:
                gripper = js['pos'] #gripper制御用
                self.pose[22] = gripper
        """

    def connect_mqtt(self):
        self.client = mqtt.Client()  
        # MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect("sora2.uclab.jp", 1883, 60)
        #self.client.connect("sora.uclab.jp", 1883, 60)
        #self.client.connect("urdemo.uclab.jp", 1883, 60)
        #self.client.loop_start()   # 通信処理開始
        self.client.loop_forever()   # 通信処理開始

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
        self.pose = np.ndarray((25,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.text_log = ""
        try:
            self.connect_mqtt()
        except KeyboardInterrupt:
            print("stop mqttrecv")
