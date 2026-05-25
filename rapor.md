# LLM-Based Natural Language Controlled Mobile Manipulation Robot
## Progress Report — May 2026

---

## 1. Project Overview

This project develops a simulation of a mobile manipulation robot controlled by natural language commands using Large Language Models (LLMs). The system integrates a Clearpath Husky A200 mobile platform, Universal Robots UR5e arm, Robotiq 2F-85 gripper, and Intel RealSense D435 depth camera in a ROS2 Humble + Gazebo Ignition environment.

The goal is to enable a user to type a command such as "Pick up the coke can and place it on the other table" and have the robot autonomously execute the full pick-and-place task.

---

## 2. System Architecture

The system is organized into three layers, each composed of independent ROS2 nodes communicating via topics.

### 2.1 LLM Layer
| File | Role | Status |
|------|------|--------|
| `prompt.py` | Class Config — model name, system prompt, options | ✅ Done |
| `llm_inference.py` | Ollama integration, JSON parsing, /llm_task publisher | ✅ Done |

**Flow:** User types English command → Ollama (llama3.2:1b) → JSON output `{"task": "pick_and_place", "object": "coke"}` → publishes object name to `/llm_task` topic.

### 2.2 Perception Layer
| File | Role | Status |
|------|------|--------|
| `camera.py` | Subscribes to Gazebo camera, publishes to /camera_view | ✅ Done |
| `detector.py` | YOLOv8 custom model, publishes JSON detections to /detection | ✅ Done |
| `scanner.py` | shoulder_pan scan, locks on target, publishes /scanner_status | ✅ Done |

**Flow:** camera.py → /camera_view → detector.py → /detection (JSON: label, conf, cx, cy, w, h) → scanner.py subscribes to /llm_task + /detection → scans with shoulder_pan → publishes SCANNING / FOUND:coke / LOCKED.

### 2.3 Navigation Layer
| File | Role | Status |
|------|------|--------|
| `navigator.py` | Visual servoing approach, /cmd_vel publisher | ✅ Done |

**Flow:** navigator.py subscribes to /scanner_status (LOCKED) + /detection + /depth → visual servoing (angular + linear simultaneously) → stops at STOP_DIST=0.75m → publishes ARRIVED to /navigator/status.

### 2.4 Manipulation Layer
| File | Role | Status |
|------|------|--------|
| `arm_controller.py` | P-controller based pick, gripper control | 🔄 In Progress |

**Planned flow:** IDLE → ALIGNING (shoulder_pan + wrist_1 centering) → REACHING (shoulder_lift + elbow extension) → GRIPPING → LIFTING → PICKED.

---

## 3. Completed Work

### 3.1 LLM Integration
- Local LLM deployment via Ollama (no cloud dependency)
- Model: llama3.2:1b (fast, low memory)
- Turkish and English command support tested
- Structured JSON output: `{"task": "pick_and_place", "object": "coke"}`
- thinking mode disabled for deterministic output
- ROS2 node publishes parsed object name to `/llm_task`

### 3.2 Custom YOLOv8 Training
- Simulation dataset collected manually using camera.py with 's' key saving
- ~80 frames collected, two object classes: coke, box
- Model trained with YOLOv8n, confidence threshold 0.2
- Achieves 0.92-0.99 confidence on simulation objects
- Real-time detection at 30 FPS

### 3.3 Scan and Detection Pipeline
- shoulder_pan_joint sweeps -1.5 to +1.5 rad
- YOLO detects target object during sweep
- Arm locks at detection angle
- Status published: SCANNING → FOUND:coke → LOCKED
- Miss counter prevents false SCAN returns (5 consecutive misses required)

### 3.4 Visual Servoing Navigation
- Simultaneous angular + linear velocity control
- align_factor ensures smooth curved approach
- Depth camera (32FC1) used for real distance measurement
- STOP_DIST tuned to 0.75m for arm reach
- ARRIVED published when target distance reached

### 3.5 Lidar Integration
- sick_lms1xx added to robot.yaml
- /a200_0000/sensors/lidar2d_0/scan topic active
- Foundation for future SLAM/Nav2 integration

### 3.6 ROS2 Package Structure
- All nodes registered as executables in setup.py
- simulation.launch.py launches all nodes with timed startup
- llm_inference runs separately (requires terminal input)

---

## 4. Current Challenge — Arm Controller

### 4.1 Problem
Picking requires precise arm positioning. Two approaches were attempted:

**Attempt 1: ikpy Inverse Kinematics**
- IK chain loaded from URDF
- Coordinate transform from camera optical frame → arm_base_link required TF
- ROS2 TF topics published as /a200_0000/tf (non-standard prefix)
- TransformListener could not connect → coordinate transform failed
- IK error remained ~1m, arm moved to wrong positions

**Attempt 2: P-controller (current)**
- No IK, no TF required
- Only 4 joints controlled: shoulder_pan, shoulder_lift, elbow, wrist_1
- ALIGNING phase: center object in camera frame using cx/cy
- REACHING phase: extend arm using shoulder_lift + elbow together
- Manually found REACH_JOINTS = [0.0, -1.40, 0.17, -1.57, -1.57, 0.0]

### 4.2 Next Step
Complete arm_controller.py with:
- Target filtering by label from /scanner_status
- ALIGNING → REACHING → GRIPPING → LIFTING state machine
- Depth-based gripper trigger (depth < 0.35m)
- PICKED status published for future place module

---

## 5. Technical Issues Encountered

| Issue | Description | Resolution |
|-------|-------------|------------|
| TF prefix mismatch | TF published on /a200_0000/tf, not /tf | Workaround: manual FK-based transform (partial) |
| vision_msgs Pose2D | BoundingBox2D.center type conflict | Replaced with JSON String messages |
| YOLO low confidence | Simulation objects not in training data | Custom dataset collected and trained |
| LLM label mismatch | llama3.2 returned "coke_can", YOLO label was "coke" | Fixed via system prompt with exact label names |
| MoveIt2 not installed | No motion planning available | Used ikpy then switched to P-controller |
| Terminal input in launch | input() does not work in launch context | llm_inference.py runs in separate terminal |

---

## 6. File Structure

```
ros2_ws/src/llm_robot/llm_robot/
├── camera.py           # Camera publisher
├── detector.py         # YOLOv8 detection
├── scanner.py          # Arm scan + lock
├── navigator.py        # Visual servoing navigation
├── arm_controller.py   # Pick manipulation (in progress)
├── llm_inference.py    # LLM command interface
├── prompt.py           # LLM configuration
└── best.pt             # Custom YOLOv8 weights
```

---

## 7. Future Work

- Complete arm_controller.py pick module
- Place module: navigate to destination table, release object
- Full pipeline integration: single command → complete pick-and-place
- SLAM integration with slam_toolbox for obstacle-aware navigation
- Nav2 path planning for autonomous navigation
- Larger training dataset for robust detection
- Tool-calling LLM (Qwen2.5 / SmolLM2) for direct node invocation

---

## 8. Demo Status

The following pipeline works end-to-end:
1. User types: "Pick up the coke can and place it on the other table"
2. LLM parses → object: "coke"
3. Robot scans → detects coke → arm locks
4. Robot navigates → stops 0.75m from object
5. Arm controller begins pick sequence (alignment in progress)