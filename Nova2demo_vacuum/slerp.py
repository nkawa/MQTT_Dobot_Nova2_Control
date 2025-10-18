# joint_interpolator.py
# -*- coding: utf-8 -*-
"""
関節角度球面線形補間モジュール
初回のみ共有メモリ参照、以降は内部管理で連続補間
"""


import time
import numpy as np
from typing import List, Optional
from dataclasses import dataclass
import math


try:
    from scipy.spatial.transform import Rotation as R
    from scipy.spatial.transform import Slerp
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


@dataclass
class JointState:
    """関節状態データクラス"""
    target_angle_rad: float
    is_moving: bool = False
    start_time: float = 0.0
    start_quaternion: Optional[object] = None
    end_quaternion: Optional[object] = None
    move_time: float = 0.0
    end_time: float = 0.0
    start_angle_rad: float = 0.0
    target_angle_deg: float = 0.0


class JointInterpolator:
    """関節角度補間クラス"""
    
    def __init__(self, joint_count: int = 6):
        self.joint_count = joint_count
        
        # 補間パラメータ
        # self.target_move_speed = 1000.0
        # self.max_move_unit = 0.01
        self.target_move_speed = 300.0
        self.max_move_unit = 0.008
        self.target_move_distance = 1.0
        
        # 角度直接補間を使用（180度問題回避）
        self.use_angle_interpolation = True
        
        # 各関節の回転軸ベクトル
        self.rotation_axes = np.array([[0.0, 0.0, 1.0] for _ in range(joint_count)])
        
        # 各関節の状態
        self.joint_states = [JointState(target_angle_rad=0.0) for _ in range(joint_count)]
        
        # 内部角度管理（連続性確保のため）
        self.internal_current_angles_deg = [0.0] * joint_count
        
        # scipy利用時のクォータニオン管理
        if SCIPY_AVAILABLE and not self.use_angle_interpolation:
            self.current_quaternions = [R.from_euler('z', 0.0) for _ in range(joint_count)]
        
        # 角度管理
        self.current_angles_rad = [0.0] * joint_count
        
        # 補間結果
        self.interpolated_angles_deg = [0.0] * joint_count
    
    def set_joint_axes(self, axes_list: List[np.ndarray]):
        """関節回転軸を設定"""
        for i, axis in enumerate(axes_list):
            if i < self.joint_count:
                self.rotation_axes[i] = axis / np.linalg.norm(axis)
    
    def set_initial_angles(self, initial_angles_deg: List[float]):
        """初期角度を設定（共有メモリから一度だけ）"""
        for i, angle_deg in enumerate(initial_angles_deg):
            if i < self.joint_count:
                self.internal_current_angles_deg[i] = angle_deg
                self.interpolated_angles_deg[i] = angle_deg
                
                angle_rad = math.radians(angle_deg)
                self.current_angles_rad[i] = angle_rad
                
                if SCIPY_AVAILABLE and not self.use_angle_interpolation:
                    self.current_quaternions[i] = R.from_rotvec(
                        self.rotation_axes[i] * angle_rad
                    )
    
    def set_target_angles(self, joint_angles_deg: List[float]):
        """目標角度を設定して補間開始"""
        current_time = time.perf_counter() * 1000
        
        for i, angle_deg in enumerate(joint_angles_deg):
            if i < self.joint_count:
                target_angle_rad = math.radians(angle_deg)
                joint_state = self.joint_states[i]
                
                # 目標設定
                joint_state.target_angle_rad = target_angle_rad
                joint_state.target_angle_deg = angle_deg
                joint_state.is_moving = True
                joint_state.start_time = current_time
                
                # 開始位置は内部管理角度を使用
                current_angle_rad = math.radians(self.internal_current_angles_deg[i])
                joint_state.start_angle_rad = current_angle_rad
                self.current_angles_rad[i] = current_angle_rad
                
                # 最短経路計算
                angle_diff_rad = target_angle_rad - current_angle_rad
                while angle_diff_rad > math.pi:
                    angle_diff_rad -= 2 * math.pi
                while angle_diff_rad < -math.pi:
                    angle_diff_rad += 2 * math.pi
                
                angle_diff = abs(angle_diff_rad)
                
                # 移動時間計算
                move_time_1 = self.target_move_distance * self.target_move_speed
                move_time_2 = (angle_diff * self.max_move_unit) * 1000
                joint_state.move_time = max(move_time_1, move_time_2)
                joint_state.end_time = joint_state.start_time + joint_state.move_time
        
        self.update_interpolation()
    
    def update_interpolation(self):
        """補間状態を更新"""
        current_time = time.perf_counter() * 1000
        
        for i in range(self.joint_count):
            joint_state = self.joint_states[i]
            
            if joint_state.is_moving:
                if current_time < joint_state.end_time:
                    # 補間中
                    elapsed_time = current_time - joint_state.start_time
                    progress = max(0.0, min(1.0, elapsed_time / joint_state.move_time))
                    
                    # 角度直接補間（最短経路）
                    start_rad = joint_state.start_angle_rad
                    target_rad = joint_state.target_angle_rad
                    
                    angle_diff = target_rad - start_rad
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                        
                    self.current_angles_rad[i] = start_rad + angle_diff * progress
                    
                    # 度数に変換
                    angle_deg = math.degrees(self.current_angles_rad[i])
                    self.interpolated_angles_deg[i] = angle_deg
                    self.internal_current_angles_deg[i] = angle_deg
                    
                else:
                    # 補間完了
                    joint_state.is_moving = False
                    self.current_angles_rad[i] = joint_state.target_angle_rad
                    self.interpolated_angles_deg[i] = joint_state.target_angle_deg
                    self.internal_current_angles_deg[i] = joint_state.target_angle_deg
    
    def get_interpolated_angles(self) -> List[float]:
        """補間結果を取得"""
        return self.interpolated_angles_deg.copy()
    
    def is_any_joint_moving(self) -> bool:
        """いずれかの関節が動作中か"""
        return any(joint_state.is_moving for joint_state in self.joint_states)
    
    def get_moving_joints(self) -> List[int]:
        """動作中の関節IDリスト"""
        return [i for i, joint_state in enumerate(self.joint_states) if joint_state.is_moving]
    
    def stop_all_joints(self):
        """全関節停止"""
        for joint_state in self.joint_states:
            joint_state.is_moving = False
