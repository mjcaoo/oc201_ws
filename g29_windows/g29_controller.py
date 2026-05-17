#!/usr/bin/env python3
"""
G29 方向盘遥控器 — Windows 端
读取 G29 输入，通过 UDP 发送控制指令到 RDK X3，同时显示摄像头实时画面。
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np
import pygame
import yaml

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
PACKET_SIZE = 12

# 按钮索引约定 (G29 G Hub 模式)
BTN_TRIANGLE = 3  # 紧急停止
BTN_CIRCLE = 2    # 零点校准
BTN_L_PADDLE = 4  # 左拨片 → 倒车
BTN_R_PADDLE = 5  # 右拨片 → 前进


class G29Input:
    """读取 Logitech G29 方向盘输入"""

    def __init__(self, steering_offset=0.0, steering_deadzone=0.03):
        pygame.init()
        pygame.joystick.init()

        self.steering_offset = steering_offset
        self.steering_deadzone = steering_deadzone
        self.joy = None
        self.steering = 0.0
        self.throttle = 0.0
        self.buttons = {}

        self._init_joystick()

    def _init_joystick(self):
        count = pygame.joystick.get_count()
        if count == 0:
            print("[G29] 未检测到方向盘，等待连接...")
            return

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        print(f"[G29] 已连接: {self.joy.get_name()}")
        print(f"[G29] 轴数量: {self.joy.get_numaxes()}, 按钮数量: {self.joy.get_numbuttons()}")

    def read(self):
        """读取当前状态，返回 (steering, throttle, buttons_dict)"""
        pygame.event.pump()

        if self.joy is None:
            count = pygame.joystick.get_count()
            if count > 0:
                self._init_joystick()
            return 0.0, 0.0, {}

        try:
            raw_steering = self.joy.get_axis(0)
            raw_throttle = self.joy.get_axis(1)
        except pygame.error:
            self.joy = None
            return 0.0, 0.0, {}

        steering = raw_steering + self.steering_offset
        if abs(steering) < self.steering_deadzone:
            steering = 0.0
        steering = max(-1.0, min(1.0, steering))

        throttle = (1.0 - raw_throttle) / 2.0

        buttons = {}
        try:
            for i in range(self.joy.get_numbuttons()):
                buttons[i] = self.joy.get_button(i)
        except pygame.error:
            pass

        self.steering = steering
        self.throttle = throttle
        self.buttons = buttons

        return steering, throttle, buttons

    def quit(self):
        pygame.quit()


class UDPClient:
    """UDP 客户端，发送控制指令到 RDK X3"""

    def __init__(self, target_ip, target_port):
        self.target = (target_ip, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def send(self, steering, throttle, button_flags):
        buttons_byte = 0
        if button_flags.get('emergency_stop', False):
            buttons_byte |= 0x01
        if button_flags.get('recalibrate', False):
            buttons_byte |= 0x02
        if button_flags.get('reverse', False):
            buttons_byte |= 0x04

        payload = struct.pack('<BffB', FRAME_HEADER, steering, throttle, buttons_byte)
        checksum = 0
        for b in payload:
            checksum ^= b
        packet = payload + struct.pack('BB', checksum, FRAME_TAIL)

        try:
            self.sock.sendto(packet, self.target)
        except OSError:
            pass

    def close(self):
        self.sock.close()


class CameraViewer:
    """通过 HTTP 接收 MJPEG 流并解码"""

    def __init__(self, url):
        self.url = url
        self._frame = None
        self._lock = threading.Lock()
        self._running = True
        self._connected = False
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def _receive_loop(self):
        while self._running:
            try:
                cap = cv2.VideoCapture(self.url)
                if not cap.isOpened():
                    self._connected = False
                    time.sleep(1)
                    continue

                self._connected = True
                while self._running:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    with self._lock:
                        self._frame = frame

                cap.release()
                self._connected = False
            except Exception:
                self._connected = False
                time.sleep(1)

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    @property
    def connected(self):
        return self._connected

    def stop(self):
        self._running = False


class HUD:
    """在画面上叠加控制信息"""

    @staticmethod
    def draw(frame, steering, throttle, reverse, speed, angle_deg, connected, button_flags):
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        h, w = frame.shape[:2]

        status_color = (0, 255, 0) if connected else (0, 0, 255)
        status_text = "CONNECTED" if connected else "NO SIGNAL"
        cv2.putText(frame, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        gear_text = "R" if reverse else "D"
        gear_color = (0, 128, 255) if reverse else (0, 255, 0)
        cv2.putText(frame, f"Gear: {gear_text}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, gear_color, 2)
        cv2.putText(frame, f"Speed: {speed:.2f} m/s", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Angle: {angle_deg:+.1f} deg", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        bar_y = h - 60
        bar_cx = w // 2
        bar_w = 200
        cv2.rectangle(frame, (bar_cx - bar_w, bar_y - 5),
                      (bar_cx + bar_w, bar_y + 25), (50, 50, 50), -1)
        steer_x = int(bar_cx + steering * bar_w)
        cv2.rectangle(frame, (steer_x - 8, bar_y - 5),
                      (steer_x + 8, bar_y + 25), (0, 200, 255), -1)
        cv2.line(frame, (bar_cx, bar_y - 10), (bar_cx, bar_y + 30), (200, 200, 200), 1)

        cv2.putText(frame, "THR", (10, h - 95), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)
        cv2.rectangle(frame, (10, h - 90), (30, h - 20), (50, 50, 50), -1)
        thr_h = int(throttle * 70)
        thr_color = (0, 128, 255) if reverse else (0, 200, 0)
        cv2.rectangle(frame, (10, h - 20 - thr_h), (30, h - 20), thr_color, -1)

        if button_flags.get('emergency_stop', False):
            cv2.putText(frame, "EMERGENCY STOP", (w // 2 - 120, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        return frame


def load_config(config_path):
    defaults = {
        'rdk_x3_ip': '192.168.1.100',
        'udp_port': 9999,
        'camera_url': 'http://192.168.1.100:8080',
        'max_speed': 1.0,
        'max_steering_angle': 32,
        'steering_offset': 0.0,
        'steering_deadzone': 0.03,
        'display_width': 640,
        'display_height': 480,
    }
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        if cfg:
            defaults.update(cfg)
    except FileNotFoundError:
        print(f"[Config] 配置文件 {config_path} 未找到，使用默认值")
    except Exception as e:
        print(f"[Config] 读取配置失败: {e}，使用默认值")
    return defaults


def main():
    parser = argparse.ArgumentParser(description='G29 方向盘遥控器')
    parser.add_argument('--ip', type=str, default=None, help='RDK X3 IP 地址')
    parser.add_argument('--port', type=int, default=None, help='UDP 端口')
    parser.add_argument('--camera', type=str, default=None, help='摄像头 URL')
    parser.add_argument('--config', type=str, default='g29_config.yaml', help='配置文件路径')
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.ip:
        cfg['rdk_x3_ip'] = args.ip
    if args.port:
        cfg['udp_port'] = args.port
    if args.camera:
        cfg['camera_url'] = args.camera

    print("=" * 50)
    print("  G29 方向盘遥控器")
    print("=" * 50)
    print(f"  目标地址: {cfg['rdk_x3_ip']}:{cfg['udp_port']}")
    print(f"  摄像头:   {cfg['camera_url']}")
    print(f"  最大速度: {cfg['max_speed']} m/s")
    print(f"  最大转角: {cfg['max_steering_angle']} deg")
    print("=" * 50)
    print("控制说明:")
    print("  方向盘      - 转向")
    print("  左边踏板    - 油门")
    print("  左拨片      - 倒车")
    print("  右拨片      - 前进")
    print("  三角按钮    - 紧急停止")
    print("  圆圈按钮    - 零点校准")
    print("  ESC/Q       - 退出")
    print("=" * 50)

    g29 = G29Input(
        steering_offset=cfg['steering_offset'],
        steering_deadzone=cfg['steering_deadzone']
    )
    udp = UDPClient(cfg['rdk_x3_ip'], cfg['udp_port'])
    cam = CameraViewer(cfg['camera_url'])

    max_speed = cfg['max_speed']
    max_angle = cfg['max_steering_angle']
    emg_stop = False
    reverse = False
    prev_btn_triangle = False
    prev_btn_circle = False

    clock = pygame.time.Clock()
    target_fps = 30

    try:
        while True:
            steering, throttle, buttons = g29.read()

            btn_triangle = buttons.get(BTN_TRIANGLE, False)
            btn_circle = buttons.get(BTN_CIRCLE, False)
            btn_l_paddle = buttons.get(BTN_L_PADDLE, False)
            btn_r_paddle = buttons.get(BTN_R_PADDLE, False)

            if btn_triangle and not prev_btn_triangle:
                emg_stop = not emg_stop
                print(f"[CTRL] 紧急停止: {'ON' if emg_stop else 'OFF'}")

            if btn_circle and not prev_btn_circle:
                g29.steering_offset = -steering
                print(f"[CTRL] 零点校准: offset={g29.steering_offset:.3f}")

            if btn_l_paddle:
                reverse = True
            elif btn_r_paddle:
                reverse = False

            prev_btn_triangle = btn_triangle
            prev_btn_circle = btn_circle

            speed = throttle * max_speed
            angle_deg = steering * max_angle

            button_flags = {
                'emergency_stop': emg_stop,
                'reverse': reverse,
            }
            udp.send(steering, throttle, button_flags)

            frame = cam.get_frame()
            display = HUD.draw(
                frame, steering, throttle, reverse,
                speed, angle_deg, cam.connected, button_flags
            )

            display_h, display_w = display.shape[:2]
            if display_w != cfg['display_width'] or display_h != cfg['display_height']:
                display = cv2.resize(display,
                                    (cfg['display_width'], cfg['display_height']))

            cv2.imshow('G29 Controller', display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

            clock.tick(target_fps)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[CTRL] 正在停止...")
        udp.send(0.0, 0.0, {'emergency_stop': True, 'reverse': False})
        time.sleep(0.1)
        udp.close()
        cam.stop()
        g29.quit()
        cv2.destroyAllWindows()
        print("[CTRL] 已退出")


if __name__ == '__main__':
    main()
