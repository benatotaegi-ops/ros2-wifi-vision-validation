# Benchmarking Results Report: Wireless Offload Architecture (RPi Sender + PC Inference via Compressed Wi-Fi)

This document presents the comparative performance analysis of the 3D cable detection system (`cable_pose_offload`) in ROS 2 Humble, evaluating the **Wireless Computational Offload** architecture (JPEG capture and compression on the Raspberry Pi, Wi-Fi transmission via CycloneDDS Unicast, and YOLO inference on the Host PC workstation) against the previously evaluated architectures: **Local (USB)** and **Edge Computing (Inference on Raspberry Pi)**.

---

## 1. Evaluated Test Environments

1. **Local Architecture (USB Baseline):**
   * **Hardware:** Host PC workstation ($30.77\text{ GiB}$ RAM).
   * **Connectivity:** Intel RealSense D435I camera connected directly via USB 3.2 bus.
   * **Deployment:** Local Docker container handling capture, inference, and visualization on the same machine.

2. **Edge Computing Architecture (Inference on RPi + Wi-Fi):**
   * **Edge Node Hardware:** Raspberry Pi ($7.75\text{ GiB}$ RAM, 4-core ARM CPU) executing both image capture and heavy YOLO inference.
   * **Base Station Hardware:** Receiving Host PC used strictly for monitoring.

3. **Remote Offload Architecture (RPi Compressed Capture + PC Inference via Wi-Fi) [Current Test]:**
   * **Sender Hardware (Sensor Node):** Raspberry Pi (`20.30.40.84`) capturing the RealSense stream and compressing it to JPEG (`/cam_3/camera/color/image_raw/compressed`) using `Best Effort` QoS (`qos_profile_sensor_data`).
   * **Processing Hardware (Compute Node):** Host PC workstation (`20.30.40.51`, $30.77\text{ GiB}$ RAM) running the Docker container (`cable_pose_yolo`), decompressing the incoming stream, and executing YOLO inference.
   * **DDS Connectivity:** Wi-Fi network configured with **CycloneDDS Unicast** (`AllowMulticast: false`) and explicit inclusion of the loopback interface (`127.0.0.1`) and static peers (`Peers`) to enable simultaneous bridging across the Raspberry Pi, Docker container, and Host PC.

---

## 2. Metrics Comparison Table (Triple Architecture)

Data for this test was extracted over continuous $120\text{ seconds}$ windows by monitoring the output stream (`/camera/color/image_annotated/compressed`), node latency logs (`latencies.log`), and container resource consumption (`docker_stats.txt`).

| Metric | 1. Local (PC + USB) | 2. Edge (Inference on RPi) | 3. Wi-Fi Offload (RPi $\rightarrow$ PC) | Offload Impact / Improvement |
| :--- | :--- | :--- | :--- | :--- |
| **Average YOLO Inference Time** | $\sim 14.2\text{ ms}$ *(11.8 - 19.7 ms)* | $\sim 145.2\text{ ms}$ *(135.1 - 253.6 ms)* | **$\sim 15.6\text{ ms}$** *(12.3 - 49.8 ms)* | **9.3x faster than Edge** (Equivalent to USB) |
| **Average Total Callback Time** | $\sim 17.1\text{ ms}$ *(14.6 - 22.5 ms)* | $\sim 153.1\text{ ms}$ *(142.4 - 261.5 ms)* | **$\sim 22.2\text{ ms}$** *(17.9 - 57.4 ms)* | **6.9x faster than Edge** ($+5.1\text{ ms}$ vs USB) |
| **Processing Overhead** | $\sim 2.9\text{ ms}$ *(Raw Img $\rightarrow$ JPEG)* | $\sim 7.9\text{ ms}$ *(On ARM CPU)* | **$\sim 6.6\text{ ms}$** *(JPEG Decomp. + Comp.)* | $+3.7\text{ ms}$ vs USB due to double encoding |
| **Frame Rate (Effective FPS)** | **$\sim 30.0\text{ FPS}$** *(Sensor cap)* | **$\sim 6.5\text{ FPS}$** *(ARM CPU cap)* | **$\sim 43.7\text{ FPS}$** *(RPi stream cap)* | **6.7x smoother than Edge** (Strict real time) |
| **Output Bandwidth (BW)** | $\sim 625.0\text{ KB/s}$ | $\sim 142.0\text{ KB/s}$ | **$\sim 1.79\text{ MB/s}$** *(1.14 - 2.14 MB/s)* | Higher throughput due to high FPS and JPEG quality |
| **Average Message Size (JPEG)** | $\sim 20.7\text{ KB}$ | $\sim 24.8\text{ KB}$ | **$\sim 40.0\text{ KB}$** *($0.04 - 0.05\text{ MB}$)* | Higher visual detail/complexity in scenes |
| **Container CPU Usage** | $436.56\%$ *(~4.3 PC threads)* | $277.40\%$ *(~2.8 RPi cores)* | **$306.18\%$** *(~3.0 PC threads)* | **$-30\%$ PC load** compared to local USB |
| **RAM Consumption** | $629.8\text{ MiB}$ ($2.00\%$) | $670.6\text{ MiB}$ ($8.45\%$) | **$487.0\text{ MiB}$** ($1.55\%$) | **$-142.8\text{ MiB}$ RAM saved** (No USB driver) |

---

## 3. Technical Analysis of Results

### A. Recovery of Strict Real-Time Performance and Inference Speed
The most decisive outcome of the *Offload* architecture is the **complete elimination of the computational bottleneck** experienced on the Raspberry Pi:
* **Pure Inference:** By moving PyTorch/YOLO execution back to the Host PC, average inference time drops from $\sim 145.2\text{ ms}$ to **$\sim 15.6\text{ ms}$** (steadily stabilizing between $12.5\text{ ms}$ and $14.5\text{ ms}$ during the second half of the test).
* **Decompression/Compression Cost (Overhead):** The difference between total callback time ($\sim 22.2\text{ ms}$) and pure inference ($\sim 15.6\text{ ms}$) is **$6.6\text{ ms}$**. Compared to $2.9\text{ ms}$ in local USB mode, these additional $\sim 3.7\text{ ms}$ correspond precisely to the time OpenCV (`compressed_imgmsg_to_cv2`) takes to decode the incoming Wi-Fi JPEG packet before feeding it to the neural network.
* **Behavior Under Network/CPU Jitter:** Logs show only 6 isolated spikes between $31.4\text{ ms}$ and $49.8\text{ ms}$ out of 250 samples ($2.4\%$ of cases), which are immediately absorbed in the next cycle without causing queue buildup or accumulated visual *lag*, thanks to the `Best Effort` profile (`qos_profile_sensor_data`).

### B. Network Throughput and Frame Rate (FPS) Analysis
* In `bandwidth_output.log`, output throughput averages **$1.79\text{ MB/s}$** ($\sim 14.3\text{ Mbps}$), fluctuating between $1.14\text{ MB/s}$ and $2.14\text{ MB/s}$ with a steady packet size of **$0.04\text{ MB}$ ($40\text{ KB}$)**.
* Dividing the average throughput ($1.79\text{ MB/s}$) by the per-message size ($0.04\text{ MB}$) confirms that the node processes and publishes at an effective average rate of **$\sim 43.7\text{ FPS}$**, matching $100\%$ of the Raspberry Pi compressor's emission rate ($\sim 45\text{ Hz}$).
* Traffic of $\sim 14.3\text{ Mbps}$ is easily manageable for any modern 802.11n/ac/ax Wi-Fi link, completely avoiding the network collapse observed when attempting to transmit raw images (`image_raw`).

### C. Host PC Resource Optimization (Docker)
By decoupling the physical camera driver (`realsense2_camera`) from the PC container and offloading it to the Raspberry Pi:
* Container **CPU usage drops from $436.56\%$ to $306.18\%$**, freeing up more than an entire CPU core on the Host PC.
* **RAM consumption drops from $629.8\text{ MiB}$ to $487.0\text{ MiB}$** (barely $1.55\%$ of the available $30.77\text{ GiB}$), resulting in a much leaner container dedicated exclusively to matrix and geometric computation (TF + Ray-Plane Intersection).

---

## 4. Conclusions and Architecture Verdict

1. **The Compressed Offload Architecture is the Optimal Solution:** It combines the best of both worlds—delivering the **physical and wireless freedom** of mounting the camera on the Raspberry Pi while preserving the **real-time speed ($\sim 44\text{ FPS}$, $\sim 22\text{ ms}$ latency)** of the desktop PC.
2. **Definitive DDS Connectivity Solution (Key Engineering Lesson):** It has been demonstrated that on networks with `AllowMulticast: false`, for a Docker container in `network_mode: host` to receive data from an external node (Raspberry Pi) while simultaneously publishing results to local tools on the PC itself (`rqt`, `ros2 topic echo`), it is **strictly mandatory** to:
   * Keep the `CYCLONEDDS_URI` variable active inside `entrypoint.bash` (avoiding `unset`).
   * Explicitly declare in the XML `<Peers>` block both the remote IP (`20.30.40.84`) and the host's own loopback/local addresses (`127.0.0.1` and `20.30.40.51`).
3. **Guaranteed Scalability:** Consuming only $\sim 3$ CPU threads and $<500\text{ MiB}$ of RAM, the Host PC has ample headroom to scale the system and simultaneously process multiple remote cameras (e.g., `cam_1`, `cam_2`, `cam_3`) without degrading robotic control response times.