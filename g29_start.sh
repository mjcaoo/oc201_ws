#!/bin/bash
# G29 小车一键启动脚本
# 用法: bash g29_start.sh

echo "=== G29 小车启动 ==="

# 1. 杀掉旧进程
pkill -9 -f g29_bridge_node 2>/dev/null
pkill -9 -f origincar_base 2>/dev/null
screen -wipe 2>/dev/null
sleep 1

# 2. 环境
source /opt/tros/setup.bash
source /root/dev_ws/install/setup.bash

# 3. 启动 bridge（UDP 端口 9999 → ROS2 /ackermann_cmd）
screen -dmS bridge bash -c '
source /opt/tros/setup.bash
source /root/dev_ws/install/setup.bash
python3 /root/dev_ws/install/g29_bridge/lib/g29_bridge/g29_bridge_node.py \
    --ros-args \
    -p steering_smooth:=0.3 \
    -p max_steering_angle:=32.0 \
    -p max_speed:=1.0
'

# 4. 启动相机驱动（USB 摄像头 → ROS2）
screen -dmS camera_driver bash -c '
source /opt/tros/setup.bash
source /root/dev_ws/install/setup.bash
ros2 run hobot_usb_cam hobot_usb_cam --ros-args \
    -r __node:=hobot_usb_cam \
    -p video_device:=/dev/video8 \
    -p pixel_format:=mjpeg \
    -p framerate:=30 \
    -p image_width:=640 \
    -p image_height:=480 \
    -r image_raw:=/image \
    --log-level warn
'

# 5. 启动相机推流（ROS2 image → HTTP MJPEG）
screen -dmS camera_stream bash -c '
source /opt/tros/setup.bash
source /root/dev_ws/install/setup.bash
python3 /root/dev_ws/install/g29_bridge/lib/g29_bridge/camera_streamer.py \
    --ros-args \
    -p image_topic:=/image \
    -p http_port:=8080
'

# 6. 启动 origincar（ROS2 → STM32 串口）
screen -dmS origincar bash -c '
source /opt/tros/setup.bash
source /root/dev_ws/install/setup.bash
exec /root/dev_ws/install/origincar_base/lib/origincar_base/origincar_base_node \
    --ros-args \
    -r __node:=origincar_base \
    -p use_direct_angle:=true \
    -p serial_baud_rate:=115200
'

sleep 2

# 5. 状态检查
echo ""
echo "ROS2 节点:"
ros2 node list 2>/dev/null

echo ""
echo "Screen 会话:"
screen -ls 2>/dev/null

echo ""
echo "=== 启动完成 ==="
echo "Windows 端运行: python g29_controller.py --ip 192.168.0.148"
