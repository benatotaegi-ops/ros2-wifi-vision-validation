# Setup Guide: Raspberry Pi (Remote Offload Mode with Docker)

This document details the structure and configuration required on the **Raspberry Pi 4** (*Sensor Edge Node*) to capture video and calibration data (`CameraInfo`) from the **Intel RealSense D435I** camera, compress the image stream into JPEG format, and transmit it over Wi-Fi to the Host PC—**all running in an isolated and reproducible Docker container**.

---

## 1. Workspace Structure on the Raspberry Pi

Create the following folder structure on the Raspberry Pi (for example, in `~/pi_camera_ws/`):

```text
pi_camera_ws/
├── docker/
│   ├── config/
│   │   ├── cyclone_dds_config.xml
│   │   └── entrypoint.bash
│   ├── docker-compose.yaml
│   └── Dockerfile
└── src/
    └── simple_compressor/
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        ├── resource/
        │   └── simple_compressor
        └── simple_compressor/
            ├── __init__.py
            └── simple_compressor_node.py
```

To quickly create the directory tree from the Raspberry Pi terminal, run:

```bash
mkdir -p ~/pi_camera_ws/docker/config
mkdir -p ~/pi_camera_ws/src/simple_compressor/resource
mkdir -p ~/pi_camera_ws/src/simple_compressor/simple_compressor
touch ~/pi_camera_ws/src/simple_compressor/resource/simple_compressor
touch ~/pi_camera_ws/src/simple_compressor/simple_compressor/__init__.py
```

---

## 2. Network Configuration (CycloneDDS Unicast)

The Raspberry Pi container will use its own IP address (`20.30.40.84`) and point directly (*Unicast*) to the Host PC (`20.30.40.51`), while also including the local loopback interface (`127.0.0.1`) so the camera node and compressor can communicate with each other inside the container.

**File:** `~/pi_camera_ws/docker/config/cyclone_dds_config.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <Interfaces>
        <!-- Local IP of the Raspberry Pi Wi-Fi interface -->
        <NetworkInterface address="20.30.40.84"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>1400B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
      <Peers>
        <!-- IP of the Host PC running the YOLO container -->
        <Peer address="20.30.40.51"/>
        <!-- Loopback and self IP so the camera and compressor discover each other inside Docker -->
        <Peer address="127.0.0.1"/>
        <Peer address="20.30.40.84"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

---

## 3. Docker Configuration Files

To grant the container on the Raspberry Pi direct access to the RealSense camera's USB 3.0 ports and the host system's Wi-Fi interface, `privileged: true` and `network_mode: "host"` are used.

### A. `Dockerfile`

**File:** `~/pi_camera_ws/docker/Dockerfile`

```dockerfile
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install RealSense drivers, CycloneDDS, CvBridge, and OpenCV
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-realsense2-camera \
    ros-humble-tf2-ros \
    python3-opencv \
    python3-numpy \
    python3-colcon-common-extensions \
    sudo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app/pi_camera_ws
```

### B. `docker-compose.yaml`

**File:** `~/pi_camera_ws/docker/docker-compose.yaml`

```yaml
version: '3.9'
services:
  pi_camera_node:
    build:
      context: ..
      dockerfile: docker/Dockerfile
    container_name: pi_camera_offload
    network_mode: "host"
    privileged: true # Required to access USB devices (/dev/video* and /dev/bus/usb)
    volumes:
      - /dev:/dev
      - ../src:/app/pi_camera_ws/src
      - ./config/cyclone_dds_config.xml:/root/cyclone_dds_config.xml
      - ./config/entrypoint.bash:/app/entrypoint.bash
    entrypoint: /bin/bash /app/entrypoint.bash
```

### C. `entrypoint.bash`

*(Critical note: The camera driver is launched in the background with `&` and the compressor node is launched in the foreground with `exec` to keep the container running indefinitely).*

**File:** `~/pi_camera_ws/docker/config/entrypoint.bash`

```bash
#!/bin/bash

export ROS_DOMAIN_ID=87
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/cyclone_dds_config.xml

# Increase kernel network buffers to prevent DDS stalls during UDP bursts
sudo sysctl -w net.core.rmem_max=20971520
sudo sysctl -w net.core.wmem_max=20971520

source /opt/ros/humble/setup.bash
ros2 daemon stop

cd /app/pi_camera_ws
rm -rf ./build/ ./install/ ./log/
colcon build --symlink-install --executor sequential
source ./install/setup.bash

# 1. Launch the Intel RealSense driver in the background (depth disabled to optimize USB/CPU)
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=cam_3 \
    camera_name:=camera \
    enable_depth:=false \
    rgb_camera.color_profile:=640x480x30 &

# Wait 3 seconds for the camera to initialize its topics
sleep 3

# 2. Launch the compressor node in the foreground to keep the container alive
exec ros2 run simple_compressor simple_compressor_node
```

Remember to grant execute permissions on the Raspberry Pi:

```bash
chmod +x ~/pi_camera_ws/docker/config/entrypoint.bash
```

---

## 4. Compressor Package Code (`simple_compressor`)

This ROS 2 package captures the raw image (`/cam_3/camera/color/image_raw`) and compresses it in memory to JPEG format at $80\%$ quality, publishing it on `/cam_3/camera/color/image_raw/compressed` using the `qos_profile_sensor_data` (*Best Effort*) profile.

### A. Node Code (`simple_compressor_node.py`)

**File:** `~/pi_camera_ws/src/simple_compressor/simple_compressor/simple_compressor_node.py`

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data


class SimpleCompressor(Node):
    def __init__(self):
        super().__init__('simple_compressor')
        self.bridge = CvBridge()

        # Subscription to the local raw image published by the RealSense (Best Effort)
        self.create_subscription(
            Image,
            '/cam_3/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publisher for the compressed image sent over Wi-Fi (Best Effort)
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/cam_3/camera/color/image_raw/compressed',
            qos_profile_sensor_data
        )
        self.get_logger().info('Node /simple_compressor started. Publishing JPEG at 80%.')

    def image_callback(self, msg):
        try:
            # Convert ROS message to OpenCV BGR matrix
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Compress to JPEG (quality 80 to balance bandwidth and sharpness)
            success, encoded_image = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])

            if success:
                msg_compressed = CompressedImage()
                msg_compressed.header = msg.header
                msg_compressed.format = "jpeg"
                msg_compressed.data = encoded_image.tobytes()
                self.publisher_.publish(msg_compressed)

        except Exception as e:
            self.get_logger().error(f"Error compressing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### B. ROS 2 Packaging Files (`package.xml`, `setup.py`, `setup.cfg`)

**File:** `~/pi_camera_ws/src/simple_compressor/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>simple_compressor</name>
  <version>0.0.1</version>
  <description>Lightweight JPEG compressor node for Wi-Fi offload</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**File:** `~/pi_camera_ws/src/simple_compressor/setup.py`

```python
from setuptools import setup

package_name = 'simple_compressor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Lightweight JPEG compressor node for Wi-Fi offload',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simple_compressor_node = simple_compressor.simple_compressor_node:main',
        ],
    },
)
```

**File:** `~/pi_camera_ws/src/simple_compressor/setup.cfg`

```ini
[develop]
script_dir=$base/lib/simple_compressor
[install]
install_scripts=$base/lib/simple_compressor
```

---

## 5. Execution and Verification Procedure

Once the Host PC and Raspberry Pi are connected to the same Wi-Fi network:

1. **Connect the RealSense camera** to a USB 3.0 (blue) port on the Raspberry Pi and connect via SSH.
2. **Navigate to the Docker folder** of the Raspberry Pi workspace:
   ```bash
   cd ~/pi_camera_ws/docker
   ```
3. **Build and start the container** in the background:
   ```bash
   docker compose down --remove-orphans
   docker compose up --build -d
   ```
4. **Verify startup** by checking the container logs in real time:
   ```bash
   docker logs pi_camera_offload -f
   ```
   *You should see the camera initialization messages (`RealSense Node Is Up!`) followed by `Node /simple_compressor started. Publishing JPEG at 80%.`*

5. **Verification from the Host PC:**
   Before launching the YOLO container on the PC, you can verify from a Host PC terminal that the Raspberry Pi topics are arriving properly:
   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=87
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file://$HOME/pc_dds.xml

   ros2 topic hz /cam_3/camera/color/image_raw/compressed
   ```