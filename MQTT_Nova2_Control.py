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


class MQTTWin(object):
    def __init__(self,root):
        self.lastErr = time.time()*1000 # epoch millisecond
        self.defPose =  ['-508.798', '140.511', '283.92', '173.949', '-0.254', '84.949']
        self.lx = 0
        self.ly= 0
        self.lz  =0
        self.global_state = {}
        self.global_state["connect"] = False

        self.root = root
        self.root.title("Config MQTT")
        self.root.geometry("600x800")

        self.mqbutton = Button(self.root, text="Connect", padx=5,
                             command=self.my_connect)
        self.mqbutton.grid(row=0,column=0,padx=2,pady=10)

        self.mqbutton = Button(self.root, text="Connect MQTT", padx=5,
                             command=self.connect_mqtt)
        self.mqbutton.grid(row=0,column=1,padx=2,pady=10)

        self.enable = Button(self.root, text="Reset", padx=5,
                             command=self.resetRobot)
        self.enable.grid(row=0,column=2,padx=2,pady=10)

        self.mvbutton = Button(self.root, text="Test1", padx=5,
                             command=self.testMove)
        self.mvbutton.grid(row=0,column=3,padx=2,pady=10)
        self.mv2button = Button(self.root, text="Test2", padx=5,
                             command=self.testMove2)
        self.mv2button.grid(row=0,column=4,padx=2,pady=10)
        self.mv3button = Button(self.root, text="Test3", padx=5,
                             command=self.testMove3)
        self.mv3button.grid(row=0,column=5,padx=2,pady=10)
        self.mv4button = Button(self.root, text="Test4", padx=5,
                             command=self.testMove4)
        self.mv4button.grid(row=0,column=6,padx=2,pady=10)

        self.mv5button = Button(self.root, text="ClearErr", padx=5,
                             command=self.clear_error)
        self.mv5button.grid(row=0,column=7,padx=2,pady=10)

        self.info_frame = LabelFrame(self.root, text="info", labelanchor="nw",
                                     bg="#FFFFFF", width=550, height=150)
        self.info_frame.grid(row=1, column=0, padx=5,pady=5,columnspan=8)

        self.label_robot_mode = Label(self.info_frame, text="")
        self.label_robot_mode.place(rely=0.1, x=10)

        self.label_feed_speed = Label(self.info_frame,text="")
        self.label_feed_speed.place(rely=0.1, x=245)

#        self.set_label(self.frame_feed, text="%", rely=0.05, x=175)

        self.xplus = Button(self.root, text="DefaultPose", padx=5,
                             command=self.defaultPose)
        self.xplus.grid(row=2,column=0,padx=2,pady=10)

        self.yplus = Button(self.root, text="SetDefPose", padx=5,
                             command=self.setDefPose)
        self.yplus.grid(row=2,column=1,padx=2,pady=10)


        self.text_log = tk.scrolledtext.ScrolledText(self.root,width=70,height=60)
        self.text_log.grid(row=3,column =0, padx=10, pady=10,columnspan=7)

        self.text_log.insert(tk.END,"Start!!")

    def log_txt(self,str):
        self.text_log.insert(tk.END,str)

    def defaultPose(self):
        self.clear_error()
        pose = self.defPose
        ret = self.client_move.sendRecvMsg("ServoP("+pose[0]+","+pose[1]+","+pose[2]+","+pose[3]+","+pose[4]+","+pose[5]+")")
        print(ret)


    def setDefPose(self):
        self.defPose = self.getPose()
        print("CurrentPose:",self.defPose)

    def my_connect(self):
        # need try
        self.client_dash = DobotApiDashboard(
                    "192.168.5.1", 29999, self.text_log)
        self.client_feed = DobotApiFeedBack(
                    "192.168.5.1", 30004,self.text_log)
        self.client_move = DobotApiDashboard("192.168.5.1", 30003, self.text_log)


        self.global_state["connect"] = not self.global_state["connect"]
        self.text_log.insert(tk.END,"Connect!"+str(self.global_state["connect"])+"\n")

        self.set_feed_back()

    def set_feed_back(self):
        if self.global_state["connect"]:
            thread = Thread(target=self.feed_back)
            thread.setDaemon(True)
            thread.start()

    def feed_back(self):
        hasRead = 0
        while True:
 #           print("self.global_state(connect)", self.global_state["connect"])
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
#            print("robot_mode:", a["robot_mode"][0])
#            print("test_value:", hex((a['test_value'][0])))
            if hex((a['test_value'][0])) == '0x123456789abcdef':
                # print('tool_vector_actual',
                #       np.around(a['tool_vector_actual'], decimals=4))
                # print('q_actual', np.around(a['q_actual'], decimals=4))

                # Refresh Properties
                self.label_feed_speed["text"] = a["speed_scaling"][0]
                self.label_robot_mode["text"] = LABEL_ROBOT_MODE[a["robot_mode"][0]]

                # Refresh coordinate points
#                self.set_feed_joint(LABEL_JOINT, a["q_actual"])
#                self.set_feed_joint(LABEL_COORD, a["tool_vector_actual"])
#                print(a["q_actual"])
#                print(a)
                # check alarms
#                if a["robot_mode"] == 9:
#                    self.display_error_info()


# ブローカーに接続できたときの処理
    def on_connect(self,client, userdata, flag, rc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
        self.client.subscribe("webxr/pose") #　connected -> subscribe
        self.log_txt("Connected MQTT"+"\n")


# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")


    def on_message(self,client, userdata, msg):
        js = json.loads(msg.payload)
#        print("Message!",js)
        if 'pad' in js:
            pd = js['pad']
            if pd['bA']:
                print("reset")
                self.resetRobot()
            if pd['b0']==1:
                return

        if 'pos' in js:
            x = js['pos']['x']
            y = js['pos']['y']
            z = js['pos']['z']
            xd = js['ori']['x']
            yd = js['ori']['y']
            zd = js['ori']['z']
        else:
            print("JSON",js)
            return
        if self.lx ==0 and self.ly == 0 and self.lz ==0:
            self.lx = x
            self.ly = y
            self.lz = z
            self.lxd = xd
            self.lyd = yd
            self.lzd = zd
        if self.lx != x or self.ly !=y or self.lz != z:
            dx = x-self.lx
            dy = y-self.ly
            dz = z-self.lz
            dxd = xd-self.lxd
            dyd = yd-self.lyd
            dzd = zd-self.lzd
            print(dxd,dyd,dzd)
            sc = 1100
            dx *= sc
            dy *= sc
            dz *= sc
#            print(dx,dy,dz)
            self.relativeMove(dx,dy,dz,dxd*160,dyd*160,dzd*160)
            self.lx = x
            self.ly = y
            self.lz = z
            self.lxd = xd
            self.lyd = yd
            self.lzd = zd


            
    def connect_mqtt(self):
        self.client = mqtt.Client()  
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect("192.168.207.22", 1883, 60)
#  client.loop_start()   # 通信処理開始
        self.client.loop_start()   # 通信処理開始


# clear error 
    def clear_error(self):
        self.client_dash.ClearError()

    def getPose(self):
        now= time.time()*1000
        if now - self.lastErr < 500: # 最後のエラーから　500msec
            return None 
        pattern = r'\{(.*?)\}'
        ret= self.client_dash.GetPose()
        match = re.search(pattern,ret)
        if match:
            ext = match.group(1).split(',')
            if len(ext)==6:
                for i in range(6):
                    ext[i] = str(int(float(ext[i])*1000)/1000)
                print("Pose",ext)
                return ext
#            print("Err ret",ret)
            # エラーが連続した場合は、すぐに呼ばない
            self.lastErr = time.time()*1000
            return None
        else:
 #           print("No Match")
            return None

    def getJoints(self):
        pattern = r'\{(.*?)\}'
        ret= self.client_dash.GetAngle()
        match = re.search(pattern,ret)
        if match:
            ext = match.group(1).split(',')
            print("Angle",ext)
            return ext



    def relativeMove(self,y,z,x,xd,zd,yd):
        pose = self.getPose()
        
        if pose:
            pose[0]=str(float(pose[0])-x)
            pose[1]=str(float(pose[1])-y)
            pose[2]=str(float(pose[2])+z)
            pose[3]=str(float(pose[3])-xd)
            pose[4]=str(float(pose[4])-yd)
            pose[5]=str(float(pose[5])+zd)
            ret = self.client_move.sendRecvMsg("ServoP("+pose[0]+","+pose[1]+","+pose[2]+","+pose[3]+","+pose[4]+","+pose[5]+")")
            print(ret)
            self.log_txt("Relavtive x:"+str(int(xd*100))+"y: "+str(int(yd*100))+" z:"+str(int(zd*100))+"\n")


    def enableRobot(self):
        ret = self.client_move.sendRecvMsg("EnableRobot()")
        print(ret)

    def resetRobot(self):
        self.lx = 0
        self.ly = 0
        self.lz = 0
        self.lxd = 0
        self.lyd = 0
        self.lzd = 0
        self.text_log.delete('1.0', 'end-1c')



    def testMove(self):
#        self.client_move.MoveJog("J1+")
#        self.client_move.MovJ(-300.0,100,200,150,0,90,coordinateMode=0)
        pose = self.getPose()
#        pose[0]=str(float(pose[0])+10)
        pose[2]=str(float(pose[2])-10)

#        ret = self.client_move.sendRecvMsg("ServoP("+pose[0]+","+pose[1]+","+pose[2]+","+pose[3]+","+pose[4]+","+pose[5]+")")
        ret = self.client_move.sendRecvMsg("ServoP("+pose[0]+","+pose[1]+","+pose[2]+","+pose[3]+","+pose[4]+","+pose[5]+")")
        #
        print(ret)
    
    def testMove2(self):
        angle = self.getJoints()
        angle[0]=str(float(angle[0])+15)
        ret = self.client_move.sendRecvMsg("JointMovJ("+angle[0]+","+angle[1]+","+angle[2]+","+angle[3]+","+angle[4]+","+angle[5]+",100,100)")
        #
        print(ret)

#        ret = self.client_move.MoveJog()
#        print(ret)

    def testMove3(self):
#        self.client_move.RelMovLTool(10,0,0,0,0,0,0,0,0)
        ret=self.client_move.MoveJog()
#        ret = self.client_move.sendRecvMsg("RelJointMovJ(-30,10,20,15,0,5)")
        print(ret)
 
    def testMove4(self):
#        self.client_move.RelJointMovJ(-10,0,0,0,0,0)
#        ret = self.client_move.sendRecvMsg("RelMovLUser(10,10,10,0,0,0,0)")
        ret = self.client_move.MoveJog("J1-")
        print(ret)

root = tk.Tk()


mqwin = MQTTWin(root)
mqwin.root.lift()
root.mainloop()


