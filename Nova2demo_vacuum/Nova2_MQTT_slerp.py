
# -*- coding: utf-8 -*-
import time
import json
from paho.mqtt import client as mqtt
import multiprocessing as mp
import multiprocessing.shared_memory
import numpy as np


def run_mqtt_process():
    """MQTTプロセスのエントリーポイント"""
    try:
        from slerp import JointInterpolator
        mqtt_instance = Nova2_MQTT()
        mqtt_instance.run_proc()
    except Exception as e:
        print(f"MQTT process error: {e}")
        import traceback
        traceback.print_exc()


class Nova2_MQTT:
    def __init__(self):
        self.sub_topic = "webxr/VR1/joint"
        self.use_interpolation = True
        self.sm = None
        self.pose = None
        self.joint_interpolator = None


    def _initialize_interpolator(self):
        """補間器の初期化"""
        try:
            from slerp import JointInterpolator
            
            # 補間器作成
            self.joint_interpolator = JointInterpolator(joint_count=6)
            
            # 関節軸設定
            joint_axes = [
                np.array([0, 0, 1]),  # J1: Z軸回転
                np.array([0, -1, 0]),  # J2: X軸回転
                np.array([0, -1, 0]),  # J3: X軸回転
                np.array([0, -1, 0]),  # J4: Z軸回転
                np.array([0, 0, 1]),  # J5: X軸回転
                np.array([0, -1, 0])   # J6: Z軸回転
            ]
            self.joint_interpolator.set_joint_axes(joint_axes)
            
            # 共有メモリから初期角度を取得（一度だけ）
            if self.pose is not None:
                while self.pose[0:5].sum() == 0:
                    time.sleep(0.5)
                    print("Wait for monitoring..")
                
                try:
                    # 現在角度を読み取り
                    initial_angles = [float(self.pose[i]) for i in range(6)]
                    self.joint_interpolator.set_initial_angles(initial_angles)
                    print(f"Initial angles from shared memory: {[f'{a:.1f}' for a in initial_angles]}")
                except Exception as e:
                    # エラー時は0度で初期化
                    initial_angles = [0.0] * 6
                    self.joint_interpolator.set_initial_angles(initial_angles)
                    print(f"Using default initial angles due to error: {e}")
            else:
                # 共有メモリ未接続時は0度で初期化
                initial_angles = [0.0] * 6
                self.joint_interpolator.set_initial_angles(initial_angles)
                print("Using default initial angles: shared memory not available")
            
            print("Joint interpolator initialized successfully")
            return True
            
        except Exception as e:
            print(f"Failed to initialize joint interpolator: {e}")
            self.use_interpolation = False
            return False


    def on_connect(self, client, userdata, flag, rc):
        print(f"MQTT Connected with result code {rc}")
        topics = [
            "webxr/VR1/joint",
            "webxr/VR2/joint", 
            "webxr/VR3/joint",
            "robothand_control/result",
            "webxr/VR_controller",
            "robothand_control/gripper_power",
            "robothand_control/gripper_min_pos"
        ]
        for topic in topics:
            self.client.subscribe(topic)


    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("MQTT: Unexpected disconnection.")


    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == self.sub_topic:
                self._handle_joint_message(msg)
            elif msg.topic == "robothand_control/result":
                self._handle_robot_selection(msg)
            elif msg.topic == "webxr/VR_controller":
                self._handle_controller_message(msg)
            elif msg.topic == "robothand_control/gripper_power":
                self._handle_gripper_power(msg)
            elif msg.topic == "robothand_control/gripper_min_pos":
                self._handle_gripper_min_pos(msg)
        except Exception as e:
            print(f"MQTT message processing error: {e}")


    def _handle_joint_message(self, msg):
        """関節角度メッセージの処理"""
        try:
            js = json.loads(msg.payload)
            
            if all(key in js for key in ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
                angle = [
                    js['j1'],
                    -js['j2'],
                    -js['j3'],
                    -js['j4'],
                    js['j5'],
                    -js['j6']
                ]
                
                if self.use_interpolation and self.joint_interpolator is not None:
                    # 補間機能使用
                    self.joint_interpolator.set_target_angles(angle)
                    interpolated_angles = self.joint_interpolator.get_interpolated_angles()
                    
                    if self.pose is not None:
                        self.pose[12:18] = interpolated_angles
                    
                    # デバッグ出力
                    moving_joints = self.joint_interpolator.get_moving_joints()
                    if moving_joints:
                        print(f"Moving joints: {moving_joints}, Target: {[f'{a:.1f}' for a in angle]} , Current: {[f'{a:.1f}' for a in interpolated_angles]}")
                else:
                    # 直接設定
                    if self.pose is not None:
                        self.pose[12:18] = angle
            
            # デフォルトフラグ処理
            if 'defaultFlag' in js and js['defaultFlag']:
                if self.pose is not None:
                    self.pose[20] = 1
                    
        except Exception as e:
            print(f"Joint message handling error: {e}")


    def _handle_robot_selection(self, msg):
        """ロボット選択処理"""
        try:
            js = json.loads(msg.payload)
            robot_id = js.get('ID')
            
            if robot_id in [1, 2, 3]:
                self.sub_topic = f"webxr/VR{robot_id}/joint"
                if self.pose is not None:
                    self.pose[18] = robot_id
                print(f"Switched to robot {robot_id}")
                
                if self.use_interpolation and self.joint_interpolator is not None:
                    self.joint_interpolator.stop_all_joints()
                    
        except Exception as e:
            print(f"Robot selection handling error: {e}")


    def _handle_controller_message(self, msg):
        """コントローラーメッセージ処理"""
        try:
            js = json.loads(msg.payload)
            if 'gripvalue' in js and self.pose is not None:
                self.pose[19] = js['gripvalue']
        except Exception as e:
            print(f"Controller message handling error: {e}")


    def _handle_gripper_power(self, msg):
        """グリッパーパワー処理"""
        try:
            js = json.loads(msg.payload)
            if 'power' in js and self.pose is not None:
                self.pose[21] = js['power']
        except Exception as e:
            print(f"Gripper power handling error: {e}")


    def _handle_gripper_min_pos(self, msg):
        """グリッパー最小位置処理"""
        try:
            js = json.loads(msg.payload)
            if 'pos' in js and self.pose is not None:
                self.pose[22] = js['pos']
        except Exception as e:
            print(f"Gripper min pos handling error: {e}")


    def update_ongoing_interpolations(self):
        """進行中の補間を更新"""
        if self.use_interpolation and self.joint_interpolator is not None:
            if self.joint_interpolator.is_any_joint_moving():
                self.joint_interpolator.update_interpolation()
                interpolated_angles = self.joint_interpolator.get_interpolated_angles()
                if self.pose is not None:
                    self.pose[12:18] = interpolated_angles


    def connect_mqtt(self):
        """MQTT接続"""
        try:
            self.client = mqtt.Client()
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message
            
            self.client.connect("sora2.uclab.jp", 1883, 60)
            print("MQTT broker connected")
            
            # 非ブロッキングで開始
            self.client.loop_start()
            
            # 補間更新ループ
            try:
                while True:
                    self.update_ongoing_interpolations()
                    time.sleep(0.016)  # 60FPS
            except KeyboardInterrupt:
                print("MQTT: Received interrupt signal")
            finally:
                self.client.loop_stop()
            
        except Exception as e:
            print(f"MQTT connection error: {e}")
            raise


    def run_proc(self):
        """メインプロセス実行"""
        try:
            print("MQTT Process: Starting...")
            
            # 共有メモリ接続
            self.sm = mp.shared_memory.SharedMemory("Nova2_joint")
            self.pose = np.ndarray((24,), dtype=np.dtype("float32"), buffer=self.sm.buf)
            print("MQTT Process: Connected to shared memory")
            
            # 補間器初期化（共有メモリから初期角度取得）
            if self.use_interpolation:
                success = self._initialize_interpolator()
                if success:
                    print("MQTT Process: Interpolation enabled")
                else:
                    print("MQTT Process: Interpolation disabled")
            
            # MQTT接続開始
            print("MQTT Process: Starting MQTT connection...")
            self.connect_mqtt()
            
        except KeyboardInterrupt:
            print("MQTT Process: Stop signal received")
        except Exception as e:
            print(f"MQTT Process error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._cleanup()


    def _cleanup(self):
        """終了処理"""
        print("MQTT Process: Cleanup started...")
        try:
            if hasattr(self, 'client'):
                self.client.loop_stop()
                self.client.disconnect()
            
            if self.joint_interpolator is not None:
                self.joint_interpolator.stop_all_joints()
            
            if self.sm is not None:
                self.sm.close()
                print("MQTT Process: Shared memory closed")
        except Exception as e:
            print(f"MQTT Process: Cleanup error: {e}")
        print("MQTT Process: Cleanup completed")


if __name__ == "__main__":
    run_mqtt_process()
