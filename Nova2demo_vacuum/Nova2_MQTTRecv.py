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

# cobottaを参考にMetaworkMQTTのパラメータ
# MQTT_SERVER = "sora2.uclab.jp"
# MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
# ROBOT_UUID = os.getenv("ROBOT_UUID","dobot-nova2-real")
# ROBOT_MODEL = os.getenv("ROBOT_MODEL","dobot-nova2-real")
# MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
# MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
# MQTT_FORMAT = os.getenv("MQTT_FORMAT", "ROBODEX2025-demo-nova2")
# MQTT_MODE = os.getenv("MQTT_MODE", "metawork")

MQTT_SERVER = "sora2.uclab.jp"
MQTT_CTRL_TOPIC = "control"
ROBOT_UUID = "robodex2025-demo-nova2"
ROBOT_MODEL = "robodex2025-demo-nova2"
MQTT_MANAGE_TOPIC = "mgr"
MQTT_MANAGE_RCV_TOPIC = "dev/"+ROBOT_UUID
MQTT_FORMAT = "robodex2025-demo-nova2"
MQTT_MODE = "metawork"

class Nova2_MQTT:
    def __init__(self):
        self.mqtt_ctrl_topic = None
        self.last_registered = None

    def on_connect(self,client, userdata, flag, rc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
        date = datetime.now().strftime('%c')
        print(date)
        print(MQTT_MODE)
        if MQTT_MODE == "metawork":
            info = {
                "date": date,
                "device": {
                    "agent": "none",
                    "cookie": "none",
                },
                "devType": "robot",
                "type": ROBOT_MODEL,
                "version": "none",
                "devId": ROBOT_UUID,
            }
            self.client.publish(MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
            # with self.mqtt_control_lock:
            #     info["topic_type"] = "mgr/register"
            #     info["topic"] = MQTT_MANAGE_TOPIC + "/register"
            #     self.mqtt_control_dict.clear()
            #     self.mqtt_control_dict.update(info)
            # self.logger.info("publish to: " + MQTT_MANAGE_TOPIC + "/register")
            self.last_registered = time.time()
            self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
            # self.logger.info("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)
            print(MQTT_MANAGE_RCV_TOPIC)
        else:
            # self.logger.info("MQTT:Connected with result code " + str(rc),
            #                  "subscribe ctrl", MQTT_CTRL_TOPIC)
            self.mqtt_ctrl_topic = MQTT_CTRL_TOPIC
            self.client.subscribe(self.mqtt_ctrl_topic)
        

    #ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
        print(msg.topic)
        if msg.topic == self.mqtt_ctrl_topic:
            js = json.loads(msg.payload)
            # ここでsabscribeした内容を共有メモリに入れる
            
            # 以下が既存の入れ方
            # if(msg.topic == self.sub_topic):
            #     #print("Message!",js)
            #     #jointをsubscribeする
            #     angle = [0,0,0,0,0,0]
            #     if 'j1' in js:
            #         angle[0] = js['j1']
            #         angle[1] = js['j2']
            #         angle[2] = js['j3']
            #         angle[3] = js['j4']
            #         angle[4] = js['j5']
            #         angle[5] = js['j6']

            #         angle[1] = -angle[1]
            #         angle[2] = -angle[2]
            #         angle[3] = -angle[3]
            #         angle[5] = -angle[5] ##カメラが横についているため
            #         self.pose[12:18] = angle #12個の内後半6個が目標値
            #     if 'defaultFlag' in js:
            #         if js['defaultFlag']:
            #             self.pose[20] = 1
            #     else:
            #         print("JSON",js)
            #         return
                
            
            # if(msg.topic == "webxr/VR_controller"):
            #     if 'gripvalue' in js:
            #         gripper = js['gripvalue'] #gripper制御用
            #         self.pose[19] = gripper
                
        elif msg.topic == MQTT_MANAGE_RCV_TOPIC:
            if MQTT_MODE == "metawork":
                js = json.loads(msg.payload)
                print("=======================================================================")
                goggles_id = js["devId"]
                mqtt_ctrl_topic = MQTT_CTRL_TOPIC + "/" + goggles_id
                if mqtt_ctrl_topic != self.mqtt_ctrl_topic:
                    if self.mqtt_ctrl_topic is not None:
                        self.client.unsubscribe(self.mqtt_ctrl_topic)
                        print("unsubscribe!!!!")
                    self.mqtt_ctrl_topic = mqtt_ctrl_topic
                self.client.subscribe(self.mqtt_ctrl_topic)
                # self.logger.info("subscribe to: " + self.mqtt_ctrl_topic)
                
                # with self.mqtt_control_lock:
                #     js["topic_type"] = "dev"
                #     js["topic"] = msg.topic
                #     self.mqtt_control_dict.clear()
                #     self.mqtt_control_dict.update(js)
        else:
            # self.logger.warning("not subscribe msg" + msg.topic)
            print("not subscribe msg" + msg.topic)

    def connect_mqtt(self):
        self.client = mqtt.Client()  
        # MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()
        print(MQTT_SERVER)
        # self.client.loop_forever()

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
        self.pose = np.ndarray((26), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.text_log = ""
        self.connect_mqtt()
        while True:
            # 30分ごとに再登録
            now = time.time()
            if (self.last_registered is not None and 
                self.last_registered + 60 * 30 < now):
                date = datetime.now().strftime('%c')
                if MQTT_MODE == "metawork":
                    info = {
                        "date": date,
                        "device": {
                            "agent": "none",
                            "cookie": "none",
                        },
                        "devType": "robot",
                        "type": ROBOT_MODEL,
                        "version": "none",
                        "devId": ROBOT_UUID,
                    }
                    self.client.publish(
                        MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
                    # with self.mqtt_control_lock:
                    #     info["topic_type"] = "mgr/register"
                    #     info["topic"] = MQTT_MANAGE_TOPIC + "/register"
                    #     self.mqtt_control_dict.clear()
                    #     self.mqtt_control_dict.update(info)
                    # self.logger.info(
                    #     "re-publish to: " + MQTT_MANAGE_TOPIC + "/register")
                    # self.last_registered = now

            # プロセス終了時
            if False:
                if MQTT_MODE == "metawork":
                    info = {"devId": ROBOT_UUID}
                    self.client.publish(
                        MQTT_MANAGE_TOPIC + "/unregister", json.dumps(info))
                    # self.logger.info(
                    #     "publish to: " + MQTT_MANAGE_TOPIC + "/unregister")
                self.client.loop_stop()
                self.client.disconnect()
                self.sm.close()
                time.sleep(1)
                # self.logger.info("Process stopped")
                self.handler.close()
                break

            time.sleep(1)
