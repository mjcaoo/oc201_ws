#!/usr/bin/env python3

import math
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
PACKET_SIZE = 12


class G29BridgeNode(Node):
    def __init__(self):
        super().__init__('g29_bridge_node')

        self.declare_parameter('udp_port', 9999)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steering_angle', 45.0)
        self.declare_parameter('ackermann_topic', 'ackermann_cmd')
        self.declare_parameter('timeout_sec', 0.5)

        self.udp_port = self.get_parameter('udp_port').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        ackermann_topic = self.get_parameter('ackermann_topic').value

        self.publisher = self.create_publisher(
            AckermannDriveStamped, ackermann_topic, QoSProfile(depth=10))

        self.steering = 0.0
        self.throttle = 0.0
        self.buttons = 0
        self.last_recv_time = self.get_clock().now()

        self.running = True
        self.udp_thread = threading.Thread(target=self._udp_receiver, daemon=True)
        self.udp_thread.start()

        self.timer = self.create_timer(1.0 / 50.0, self._publish_cmd)

        self.get_logger().info(
            f'G29 Bridge started. UDP port={self.udp_port}, '
            f'max_speed={self.max_speed} m/s, max_angle={self.max_steering_angle} deg')

    def _udp_receiver(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)
        sock.bind(('0.0.0.0', self.udp_port))

        while self.running:
            try:
                data, addr = sock.recvfrom(128)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) != PACKET_SIZE:
                continue
            if data[0] != FRAME_HEADER or data[-1] != FRAME_TAIL:
                continue

            checksum = 0
            for i in range(9):
                checksum ^= data[i]
            if checksum != data[10]:
                continue

            steering_raw, throttle_raw = struct.unpack_from('<ff', data, 1)
            buttons_raw = data[9]

            self.steering = max(-1.0, min(1.0, steering_raw))
            self.throttle = max(0.0, min(1.0, throttle_raw))
            self.buttons = buttons_raw
            self.last_recv_time = self.get_clock().now()

        sock.close()

    def _publish_cmd(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_recv_time).nanoseconds / 1e9

        if elapsed > self.timeout_sec:
            msg = AckermannDriveStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'g29'
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.publisher.publish(msg)
            return

        emergency_stop = bool(self.buttons & 0x01)

        if emergency_stop:
            speed = 0.0
            angle = 0.0
        else:
            speed = self.throttle * self.max_speed
            angle = self.steering * math.radians(self.max_steering_angle)

        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'g29'
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.publisher.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = G29BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
