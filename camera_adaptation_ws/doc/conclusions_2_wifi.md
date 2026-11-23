# Benchmarking Results Report: Local Architecture (USB) vs. Wireless Edge Computing (Wi-Fi)

This document presents the comparative performance analysis of the 3D cable detection system (`cable_pose_yolo`) in ROS 2 Humble, evaluating the impact of migrating from a centralized workstation connected via USB cable to a distributed *Edge Computing* architecture running on a Raspberry Pi communicating over Wi-Fi.

---

## 1. Evaluated Test Environments

1. **Local Architecture (USB Baseline):**
   * **Hardware:** Host PC workstation (30.77 GiB RAM).
   * **Connectivity:** Intel RealSense D435I camera connected directly via USB 3.2 bus.
   * **Deployment:** Local Docker container handling both inference and visualization on the same machine.

2. **Remote Architecture (Edge Wi-Fi):**
   * **Edge Node Hardware:** Raspberry Pi (7.75 GiB RAM, 4-core ARM processor).
   * **Base Station Hardware:** Receiving Host PC for monitoring and topic consumption.
   * **Connectivity:** Corporate Wi-Fi network with client isolation. Communication via **CycloneDDS Unicast** tunnel (`AllowMulticast: false`, `MaxMessageSize: 1400B` fragmentation, and container running in `network_mode: host`).

---

## 2. Metrics Comparison Table

Data was extracted after stabilizing the message stream over continuous 120-second windows on the compressed topic `/camera/color/image_annotated/compressed`.

| Metric | Local System (PC + USB) | Remote System (RPi + Wi-Fi) | Variation / Impact |
| :--- | :--- | :--- | :--- |
| **Average YOLO Inference Time** | ~14.2 ms *(range: 11.8 - 19.7 ms)* | ~145.2 ms *(range: 135.1 - 253.6 ms)* | **10.2x slower** (ARM CPU limit) |
| **Average Total Callback Time** | ~17.1 ms *(range: 14.6 - 22.5 ms)* | ~153.1 ms *(range: 142.4 - 261.5 ms)* | **8.9x slower** |
| **Post-processing Overhead** | ~2.9 ms *(Callback - Inference)* | ~7.9 ms *(Callback - Inference)* | +5.0 ms in memory management/compression |
| **Frame Rate (Effective FPS)** | **~30.0 FPS** *(Sensor cap)* | **~6.5 FPS** *(Compute cap)* | 78.3% reduction in temporal smoothness |
| **Output Bandwidth (BW)** | ~625.0 KB/s *(stabilized)* | ~142.0 KB/s *(stabilized)* | **-77.3% network traffic** |
| **Average Message Size (JPEG)** | ~20.7 KB | ~24.8 KB | +4.1 KB due to scene variations |
| **Container CPU Usage** | 436.56% *(~4.3 threads on PC)* | 277.40% *(~2.8 of 4 cores on RPi)* | High thermal/computational load on RPi |
| **RAM Consumption** | 629.8 MiB (2.00% of total) | 670.6 MiB (8.45% of total) | Identical and very lightweight memory footprint |

*(Technical note: The `0B / 0B` reading in the `NET I/O` metric of `docker stats` is due to using `network_mode: host`, where the container directly shares the host system's physical network interface without passing through a virtual Docker bridge).*

---

## 3. Technical Analysis of Results

### A. Computational Performance and Latency
The fundamental difference between both systems lies in raw compute capacity:
* In the **Local System**, the total callback time (~17.1 ms) is lower than the camera's sampling period at 30 FPS (33.3 ms). The system operates in **strict real time** with no frame drops caused by processing.
* In the **Remote System (Raspberry Pi)**, inference relies entirely on the ARM CPU (~145.2 ms per image). This acts as a natural rate limiter, capping the detector node's output at **~6.5 FPS**. A single latency spike of up to ~253.6 ms was observed (likely caused by Python garbage collection or brief *thermal throttling*), after which the system recovered immediately and autonomously.

### B. Wireless Network Efficiency (CycloneDDS)
The bandwidth analysis proves that **the Wi-Fi network is not the bottleneck**:
* The throughput transmitted by the Raspberry Pi remains remarkably stable between **135 KB/s and 152 KB/s** (barely **~1.15 Mbps**).
* A standard Wi-Fi network (802.11n/ac) easily supports between 50 and 300 Mbps of real throughput, meaning the annotated and compressed video stream consumes **less than 2% of available wireless capacity**.
* Configuring UDP fragmentation (`MaxMessageSize: 1400B`) and Unicast discovery completely eliminated kernel stalls (`retcode -1`) and corporate router Multicast restrictions, achieving continuous packet delivery with zero connection drops.

---

## 4. Conclusions and Viability Verdict

1. **Wireless Viability Confirmed:** Migrating to a wireless architecture using ROS 2 and CycloneDDS Unicast is **fully viable and robust for industrial/corporate environments**. The Wi-Fi link carries the detections and compressed video with marginal bandwidth usage (~142 KB/s).
2. **Suitability for the Use Case:**
   * If the cable detection task is performed while the robot is stationary (*stop-and-look*) or moving at moderate speeds, a refresh rate of **~6.5 FPS (~153 ms latency) is fully functional** and completely eliminates the physical tether of USB cables.
3. **Future Optimization Paths (Optional):**
   * Since the Wi-Fi network has plenty of headroom to support higher throughput, if higher FPS on the Raspberry Pi is required in the future, efforts should not focus on the network, but on hardware-accelerated inference (e.g., exporting the PyTorch `.pt` model to ARM-optimized formats such as **ONNX**, **NCNN**, or **OpenVINO**, or adding an NPU accelerator like Hailo-8L).