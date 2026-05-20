#!/usr/bin/env python3

import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'close')
        self.end_headers()

        while self.server.streamer.running:
            jpeg_bytes = self.server.streamer.get_jpeg_bytes()
            if jpeg_bytes is None:
                self.server.streamer.event.wait(timeout=0.1)
                continue

            try:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg_bytes)}\r\n'.encode())
                self.wfile.write(b'\r\n')
                self.wfile.write(jpeg_bytes)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    def log_message(self, format, *args):
        pass


class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_streamer')

        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('http_port', 8080)

        image_topic = self.get_parameter('image_topic').value
        self.http_port = self.get_parameter('http_port').value

        self._jpeg_bytes = None
        self._lock = threading.Lock()
        self.event = threading.Event()
        self.running = True

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            CompressedImage, image_topic, self._image_callback, qos)

        self.http_thread = threading.Thread(target=self._run_http_server, daemon=True)
        self.http_thread.start()

        self.get_logger().info(
            f'Camera streamer started. '
            f'Subscribing to [{image_topic}] (CompressedImage), HTTP port={self.http_port}')

    def _image_callback(self, msg):
        with self._lock:
            self._jpeg_bytes = bytes(msg.data)
        self.event.set()

    def get_jpeg_bytes(self):
        with self._lock:
            return self._jpeg_bytes

    def _run_http_server(self):
        server = HTTPServer(('0.0.0.0', self.http_port), MJPEGHandler)
        server.streamer = self
        server.timeout = 1.0
        self.get_logger().info(f'HTTP MJPEG server listening on port {self.http_port}')
        while self.running:
            server.handle_request()
        server.server_close()

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
