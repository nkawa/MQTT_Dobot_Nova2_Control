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

import pymodbus.client
from pymodbus import FramerType, pymodbus_apply_logging_config

LABEL_ROBOT_MODE = {
    1:	"ROBOT_MODE_INIT",
    2:	"ROBOT_MODE_BRAKE_OPEN",
    3:	"",
    4:	"ROBOT_MODE_DISABLED",
    5:	"ROBOT_MODE_ENABLE",
    6:	"ROBOT_MODE_BACKDRIVE",
    7:	"ROBOT_MODE_RUNNING",
    8:	"ROBOT_MODE_RECORDING",
    9:	"ROBOT_MODE_ERROR",
    10:	"ROBOT_MODE_PAUSE",
    11:	"ROBOT_MODE_JOG"
}

class Nova2_GUI:
    def __init__(self,root):
        self.lastErr = time.time()*1000 # epoch millisecond
        self.global_state = {}
        self.global_state["connect"] = False
        self.text_log = ""

        self.root = root
        self.root.title("MQTT-Nova2 Controller")
        self.root.geometry("600x800")

        self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
        self.pose = np.ndarray((26), dtype=np.dtype("float32"), buffer=self.sm.buf)
       
        self.info_frame = LabelFrame(self.root, text="info", labelanchor="nw",
                                     bg="#FFFFFF", width=550, height=150)
        self.info_frame.grid(row=1, column=0, padx=5,pady=5,columnspan=8)

        self.label_robot_mode = Label(self.info_frame, text="")
        self.label_robot_mode.place(rely=0.1, x=10)

        self.label_feed_speed = Label(self.info_frame,text="")
        self.label_feed_speed.place(rely=0.1, x=245)

        #self.set_label(self.frame_feed, text="%", rely=0.05, x=175)

        self.enb = Button(self.root, text="EnableRobot", padx=5,
                             command=self.enableRobot)
        self.enb.grid(row=2,column=1,padx=2,pady=10)

        self.enb = Button(self.root, text="DisableRobot", padx=5,
                             command=self.disableRobot)
        self.enb.grid(row=2,column=2,padx=2,pady=10)

        self.text_log = tk.scrolledtext.ScrolledText(self.root,width=70,height=60)
        self.text_log.grid(row=5,column =0, padx=10, pady=10,columnspan=7)

        self.text_log.insert(tk.END,"Start!!")
        self.client_feed = DobotApiFeedBack("192.168.5.1", 30004,self.text_log)

        self.global_state["connect"] = not self.global_state["connect"]
        self.text_log.insert(tk.END,"Connect!"+str(self.global_state["connect"])+"\n")

        self.set_feed_back()

    
    def log_txt(self,str):
        self.text_log.insert(tk.END,str)

    def defaultPose(self):
        self.clear_error()
        pose = self.defPose
        ret = self.client_move.sendRecvMsg("ServoP("+pose[0]+","+pose[1]+","+pose[2]+","+pose[3]+","+pose[4]+","+pose[5]+")")
        print(ret)

    
    def set_feed_back(self):
        if self.global_state["connect"]:
            thread = Thread(target=self.feed_back)
            thread.setDaemon(True)
            thread.start()

    def feed_back(self):
        hasRead = 0
        while True:
            #print("self.global_state(connect)", self.global_state["connect"])
            if not self.global_state["connect"]:
                break
            data = bytes()
            while hasRead < 1440:
                temp = self.client_feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0

            a = np.frombuffer(data, dtype=MyType)
            if hex((a['test_value'][0])) == '0x123456789abcdef':
                # print('tool_vector_actual',
                #       np.around(a['tool_vector_actual'], decimals=4))
                # print('q_actual', np.around(a['q_actual'], decimals=4))

                # Refresh Properties
                self.label_feed_speed["text"] = a["speed_scaling"][0]
                self.label_robot_mode["text"] = LABEL_ROBOT_MODE[a["robot_mode"][0]]


    def setDefPose(self):
        self.defPose = self.getPose()
        print("CurrentPose:",self.defPose)

    def enableRobot(self):
        #ret= self.client_dash.EnableRobot(load=1.0, centerX=0, centerY=0, centerZ=70)
        self.pose[25] = 1
    
    def disableRobot(self): 
       #ret = self.client_dash.DisableRobot()
        self.pose[25] = 0

    def clear_error(self):
       #self.client_dash.ClearError()
       print("clear error")
    
    def resetRobot(self):
        self.lx = 0
        self.ly = 0
        self.lz = 0
        self.lxd = 0
        self.lyd = 0
        self.lzd = 0
        self.text_log.delete('1.0', 'end-1c')
    
    def openGripper(self):
        print("0pen")



class GUI:
    def run_proc(self):
       
        try:
            root = tk.Tk()
            mqwin = Nova2_GUI(root)
            mqwin.root.lift()
            root.mainloop()
        except KeyboardInterrupt:
            print("close GUI")