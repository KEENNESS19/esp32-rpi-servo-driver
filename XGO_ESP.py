#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import threading
from enum import IntEnum
from typing import List, Optional, Dict, Any
import numpy as np

class PICommand(IntEnum):
    READ_SPEED = 0x01
    READ_POSITION = 0x02
    READ_ACCE = 0x03
    READ_GYRO = 0x04
    READ_TEMP = 0x05
    CALIBRATE = 0x06
    INIT_SERVOS = 0x07

class XGO:
    
    def __init__(self, port="/dev/ttyAMA0", baud=115200, version="xgomini4", verbose=True):
        self.ser = serial.Serial("/dev/ttyAMA0", baud, timeout=0.5)
        self.port = port
        self.baudrate = baud
        self.version = version
        self.verbose = verbose
        self.timeout = 0.1
        self.serial = None
        self.connected = False
        self.lock = threading.Lock()
        self.ser.flushOutput() 
        self.ser.flushInput()
        self.rx_data = bytearray(50)  
        self.angle = np.zeros(12, dtype=np.float32)
        self.angle_vel = np.zeros(12, dtype=np.float32)
        self.action = np.zeros(12, dtype=np.float32)
        
        self.servo_ids = [43, 42, 41, 33, 32, 31, 23, 22, 21, 13, 12, 11]
        
    
    def calculate_checksum(self, data):
        """计算校验和：求和取反"""
        if not data:
            return 0
        checksum = sum(data) & 0xFF     
        return (~checksum) & 0xFF        
    
    def build_packet(self, cmd, parameters=b""):
        """
        构建数据包
        
        协议格式:
        - 无命令包: FF + 参数 + CHECKSUM + FE
        - 命令包: FF + CMD + CHECKSUM + FE
        """
        packet = bytearray()
        
        packet.append(0xFF)

        if cmd is not None:
            packet.append(cmd)
        
        if parameters:
            packet.extend(parameters)
        
        checksum_data = packet[1:]  
        checksum = self.calculate_checksum(checksum_data)
        packet.append(checksum)
        packet.append(0xFE)
        
        return bytes(packet)
    
    def send_packet(self, packet):
        """发送数据包"""
        self.ser.write(packet)
        if self.verbose:
            print("tx_data: ", packet)
    
    def motor_data_decoder(self, data):
        """解析ESP32返回的数据包"""
        self.motor_data_list = []
        
        if not data:
            # print("没有数据可解析")
            return []
        
        if self.verbose:
            print(f"开始解析数据，长度: {len(data)}")
            print(f"原始数据: {data.hex()}")
        
        i = 0
        while i < len(data):
            if data[i] == 0xFF:

                packet_found = False
                
             
                for j in range(i+1, min(i+100, len(data))):
                    if data[j] == 0xFE:
                     
                        candidate_packet = data[i:j+1]
                        
                     
                        if len(candidate_packet) < 4:
                            continue
                            
                       
                        checksum = candidate_packet[-2]
                        calc_data = candidate_packet[1:-2]
                        calc = self.calculate_checksum(calc_data)
                        
                        if checksum == calc:
                           
                            self._process_valid_packet(candidate_packet)
                            i = j + 1 
                            packet_found = True
                            break
                        else:
                           
                            pass
                
                if not packet_found:
          
                    i += 1
            else:
                i += 1
        
        if self.verbose:
            print(f"解析完成，找到 {len(self.motor_data_list)} 个数据包")
        return self.motor_data_list

    def _process_valid_packet(self, packet):
        """处理已验证的数据包"""
        if self.verbose:
            print(f"提取有效数据包，长度: {len(packet)}")
            
        cmd = packet[1]
        params = packet[2:-2]
        
        if self.verbose:
            print(f"命令: 0x{cmd:02X}")
            print(f"参数长度: {len(params)}")
            # print(f"参数: {params.hex()}")

        parsed_params = []
        
        # 根据命令类型解析参数
        if cmd == PICommand.READ_POSITION.value:
            if len(params) == 24: 
                for j in range(0, len(params), 2):
                    if j+1 < len(params):
                        value = (params[j+1] << 8) | params[j]
                        parsed_params.append(value)
                        if self.verbose:
                            print(f"舵机位置 {j//2}: {value}")
            else:
                print(f"位置数据长度异常: {len(params)}")
        
        elif cmd == PICommand.READ_SPEED.value:
            if len(params) == 24: 
                for j in range(0, len(params), 2):
                    if j+1 < len(params):
                        value = (params[j+1] << 8) | params[j]
                        parsed_params.append(value)
                        if self.verbose:
                            print(f"舵机速度 {j//2}: {value}")
        
        elif cmd == PICommand.READ_TEMP.value:
            if len(params) == 12: 
                for j in range(len(params)):
                    value = params[j]
                    parsed_params.append(value)
                    if self.verbose:
                        print(f"舵机温度 {j}: {value}°C")
        
        elif cmd == PICommand.READ_ACCE.value or cmd == PICommand.READ_GYRO.value:
            if len(params) == 6:  
                for j in range(0, len(params), 2):
                    if j+1 < len(params):
                        raw_value = (params[j+1] << 8) | params[j]
                        
                        if params[j+1] & 0x80: 
                            raw_value = raw_value - 0x10000
                        
                        sensor_type = "加速度" if cmd == PICommand.READ_ACCE.value else "陀螺仪"
                        axis = ['X', 'Y', 'Z'][j//2]
                        
                        actual_value = 0.0
                        unit = ""
                        
                        if cmd == PICommand.READ_ACCE.value:
                            ACCE_SCALE_FACTOR = 16384.0 
                            actual_value = raw_value / ACCE_SCALE_FACTOR  
                            unit = "g"
                            
                        if cmd == PICommand.READ_GYRO.value:
                            GYRO_SCALE_FACTOR = 32.8 
                            actual_value = raw_value / GYRO_SCALE_FACTOR  
                            unit = "dps"
                        
                        parsed_params.append({
                            'raw': raw_value,
                            'value': actual_value,
                            'unit': unit
                        })
                        
                        if self.verbose:
                            print(f"{sensor_type} {axis}轴: RAW={raw_value:6d}, {actual_value:8.3f} {unit}")

        self.motor_data_list.append({
            'cmd': cmd,
            'checksum_ok': True,
            'params_raw': params.hex(),
            'params_parsed': parsed_params if parsed_params else [],
            'packet_length': len(packet)
        })



    def read_status(self, cmd, servo_indices=None):
        if servo_indices is None:
            servo_indices = self.servo_ids  
        
        packet = self.build_packet(cmd)
        self.ser.flushInput()   
        self.send_packet(packet)
        
        expected_length = 4
        if cmd == PICommand.READ_POSITION or cmd == PICommand.READ_SPEED:
            expected_length += 12 * 2  
        elif cmd == PICommand.READ_TEMP:
            expected_length += 12 * 1 
        elif cmd == PICommand.READ_ACCE or cmd == PICommand.READ_GYRO:
            expected_length += 6   
        elif cmd == PICommand.CALIBRATE or cmd == PICommand.INIT_SERVOS:
            expected_length += 1     
            
        if cmd == PICommand.CALIBRATE or cmd == PICommand.INIT_SERVOS:
            self.ser.timeout = 200 # 舵机标定时间可以在esp32更改，舵机初始化可能在1分钟之内
            print(f"发送系统命令 {cmd.name}，请等待执行完成...")

        try:
            data = self.ser.read(expected_length + 10) 
            self.motor_data_decoder(data)
        finally:
            if cmd == PICommand.CALIBRATE or cmd == PICommand.INIT_SERVOS:
                self.ser.timeout = 0.5 

    
    def send_action(self, positions):

        
        for i, pos in enumerate(positions):
            if pos < 0 or pos > 4095:
                print(f"警告: 舵机{i}位置{pos}超出有效范围(0-4095)")
                positions[i] = max(0, min(4095, pos))
        
        parameters = bytearray()
        for pos in positions:
            parameters.append(pos & 0xFF)        
            parameters.append((pos >> 8) & 0xFF)  
        
        packet = bytearray()
        packet.append(0xFF)        
        packet.extend(parameters)   
        
        checksum_data = packet[1:]  
        checksum = self.calculate_checksum(checksum_data)
        packet.append(checksum)      
        packet.append(0xFE)
        self.ser.flushInput()           
        self.send_packet(packet)
        
        if self.verbose:
            print(f" 原始数据: {packet.hex()}")
        
        # 成功时回复: FF 00 0C Checksum FE 
        data = self.ser.read(5)
        self.motor_data_decoder(data)
        

    
robot=XGO()
test_positions_1 = [
    1800, 2200, 2600,    
    1200, 1800, 2200,    
    1800, 2200, 2600,     
    1200, 1800, 2200      
]

test_positions_2 = [
    0,     4095,  2048,   
    100,   4000,  2000,
    500,   3500,  3000,
    4095,  0,     2048 
]

for i, positions in enumerate([test_positions_1, test_positions_2], 1):
    start = time.time()
    robot.read_status(PICommand.READ_POSITION)
    time.sleep(0.05)
    robot.read_status(PICommand.READ_SPEED)
    time.sleep(0.05)
    robot.read_status(PICommand.READ_TEMP)
    time.sleep(0.05)
    robot.read_status(PICommand.READ_ACCE)
    robot.read_status(PICommand.READ_GYRO)
    robot.read_status(PICommand.CALIBRATE)
    robot.read_status(PICommand.INIT_SERVOS)

    robot.send_action(positions)
    time.sleep(0.05)
    stop = time.time()
    
    print(stop - start)
    print(len(robot.motor_data_list))
    

