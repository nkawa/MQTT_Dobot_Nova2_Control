# -*- coding: utf-8 -*-
import time
import tkinter as tk
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_api import *

import multiprocessing as mp
import multiprocessing.shared_memory
from multiprocessing import Process,Queue

import Nova2_cont_pro as Nova2_cont
import Nova2_monitor
import Nova2_MQTT_slerp as Nova2_MQTTRecv
import Nova2_gui
import gripper
# import experiment_gripper as gripper

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

class ProcessManager:
    def __init__(self):
        mp.set_start_method('spawn')
        sz = 32* np.dtype('float').itemsize
        self.sm = mp.shared_memory.SharedMemory(create=True,size = sz, name='Nova2_joint')
        self.ar = np.ndarray((25,), dtype=np.dtype("float32"), buffer=self.sm.buf) # 共有メモリ上の Array(0~5:現在角度、6~11:現在速度、12~17:目標角度, 18:current_topic_No, 19:grippervalue, 20:defaultFlag, 21:gripper_power, 22:gripper_min_pos, 23:touchflag, 24:topic_delay)
        self.processes = []

    def startRecvMQTT(self):
        self.recv = Nova2_MQTTRecv.Nova2_MQTT()
        self.recvP = Process(target=self.recv.run_proc, args=())
        print("MQTT pross start")
        self.recvP.start()
        print("ok?")
        self.processes.append(self.recvP)

    def startMonitor(self):
        self.mon = Nova2_monitor.Nova2_MON()
        self.monP = Process(target=self.mon.run_proc, args=())
        self.monP.start()
        self.processes.append(self.monP)

    def checkSM(self):
        while True:
            print(time.time(), self.ar)
            time.sleep(2)

    def startController(self):
        self.cont = Nova2_cont.Nova2_CON()
        self.contP = Process(target=self.cont.run_proc, args=())
        self.contP.start()
        self.processes.append(self.contP)

    def startGUI(self):
        self.gui = Nova2_gui.GUI()
        self.guiP = Process(target=self.gui.run_proc, args=())
        self.guiP.start()
        self.processes.append(self.guiP)

    def startGripper(self):
        self.gripper = gripper.Gripper()
        self.gripperP = Process(target=self.gripper.run_proc, args=())
        self.gripperP.start()
        self.processes.append(self.gripperP)

    def terminatelAll(self):
        for p in self.processes:
            p.join(timeout=5)
            if p.is_alive():
                p.terminate()
                print("kill", p)
        self.sm.close()
        self.sm.unlink()

if __name__ == '__main__':
    pm = ProcessManager()

    try:
        print("GUI")
        pm.startGUI()

        print("monitor")
        pm.startMonitor()

        print("controller")
        pm.startController()

        print("MQTT")
        pm.startRecvMQTT()

        #print("sum")
        #pm.checkSM()

        print("gripper")
        pm.startGripper()
        
        while True:
            time.sleep(1)
        
    except KeyboardInterrupt:
        print("ctr")
    finally:
        pm.terminatelAll()