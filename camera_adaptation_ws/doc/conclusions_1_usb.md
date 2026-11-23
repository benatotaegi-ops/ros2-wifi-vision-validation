# Validation and Performance Report: Compressed Image Transport Pipeline (YOLO)

This document compiles the technical conclusions, performance metrics, and comparisons obtained during the implementation and validation of the YOLO-based cable detection system using ROS 2 and Docker containers.

---

## 1. Executive Summary and Objectives
The main objective of the project was to design, implement, and validate an optimized pipeline for transmitting and processing images from a RealSense camera (`/cam_3/camera/color/image_raw`) within an isolated ROS 2 Docker environment.

The original (uncompressed) system suffered from severe bottlenecks due to CPU saturation when processing massive high-resolution image streams. To mitigate this, an architecture based on JPEG compression (`image_transport`) and frame filtering was developed, evaluating three main scenarios:
1. **Standard Version (Uncompressed / 30 FPS):** System baseline.
2. **Compressed Version with Throttling (10 FPS - 1 out of every 3 frames):** Intermediate test configuration to relieve processor load.
3. **Full Compressed Version (30 FPS unrestricted):** Final optimized architecture.

---

## 2. Performance and Bandwidth Comparison

Below are the output bandwidth metrics (`/camera/color/image_annotated/compressed` or equivalent), inference latency, and resource consumption for each phase:

### A. Standard Version (Uncompressed - 30 FPS)
* **Output Bandwidth:** ~24.0 MB/s - 27.4 MB/s constant.
* **Message Size:** Fixed at ~0.92 MB per frame.
* **Inference Latency (YOLO):** 28 - 40 ms per frame (showing signs of saturation and queue buildup under heavy load).
* **CPU Usage:** ~800% - 827% (clear processor saturation).
* **Conclusion:** Unfeasible for distributed or bandwidth-constrained network environments due to massive bandwidth consumption and processor saturation.

### B. Compressed Version with Throttling (10 FPS)
* **Input/Output Bandwidth:** Reduced to a stable ~1.5 MB/s thanks to temporal filtering.
* **Inference Latency:** 28 - 35 ms with high stability and no message loss from queue overflow.
* **Conclusion:** Demonstrated that flow control drastically relieves CPU load, serving as an intermediate validation step.

### C. Full Compressed Version (30 FPS without Throttling)
* **Output Bandwidth:** ~600 KB/s - 670 KB/s on the `/compressed` topic.
* **Message Size:** Drastically reduced to an average of ~20 KB per JPEG-compressed frame.
* **Inference Latency (YOLO):** Exceptionally fast, ranging between **12 and 18 ms** per frame, with a total callback time below 22 ms.
* **CPU Usage:** Stabilized around **436%**, well below critical saturation.
* **Conclusion:** This is the definitive optimal configuration. It maintains the camera's full 30 FPS frame rate with a bandwidth reduction of over **97%** and a massive drop in YOLO processing time.

---

## 3. Key Technical Conclusions

1. **Environment Isolation in ROS 2:**
   * Naming and path conflicts in `PYTHONPATH` and shared `build/` and `install/` artifacts were successfully resolved by restructuring the packages (`cable_pose_yolo` vs `cable_pose_yolo_compress`) and using an independent entry script (`entrypoint.bash`) for each version.
2. **Image Transport Dependencies:**
   * It was strictly necessary to install the `ros-humble-image-transport-plugins` and `ros-humble-compressed-image-transport` packages inside the `Dockerfile` to enable JPEG compression codecs and prevent loading errors in viewers (`rqt`).
3. **JPEG Compression Efficiency:**
   * Native compression via `image_transport` proved highly efficient, reducing image size from 0.92 MB to ~20 KB without compromising the visual quality required for YOLO cable detection.

---

## 4. Final Project Status
The system is fully validated, stable, and operational under the 30 FPS compressed architecture, meeting all performance, robustness, and network optimization requirements established for the robotic system.