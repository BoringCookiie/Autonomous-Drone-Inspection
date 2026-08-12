# Knowledge-Grounded Zero-Shot Defect Inspection with Confidence-Triggered Revisit Flight for Autonomous UAV Monitoring of Earthen Heritage Architecture

[![ROS2](https://img.shields.io/badge/ROS2-Humble%2F%20Jazzy-blue.svg)](https://docs.ros.org/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-red.svg)](https://px4.io/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.0%2B-orange.svg)](https://pytorch.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An end-to-end autonomous UAV defect inspection framework specifically tailored for earthen heritage architecture (e.g., mudbrick walls, rammed earth structures). The system combines decoupled waypoint camera captures, multi-backend AI defect detection (Raw VLM, RAG-grounded VLM, and YOLOv11), and a detector-agnostic confidence-triggered revisit flight loop with A* path planning.

---

## 🏛️ Project Architecture Overview

```
                        +---------------------------------------+
                        |  1. Decoupled Waypoint Capture Node   |
                        |   (Captures RGB & Depth @ Waypoint)   |
                        +-------------------+-------------------+
                                            |
                                            v
                        +-------------------+-------------------+
                        |       2. AI Detection Node            |
                        |  [Raw VLM | RAG VLM | YOLOv11]        |
                        |  Outputs: (BBox, Class, Confidence C) |
                        +-------------------+-------------------+
                                            |
                                            v
                   +------------------------+------------------------+
                   |  Confidence Evaluation: C in [0.4, 0.7]?        |
                   +------------------------+------------------------+
                             /                            \
                      YES   /                              \   NO
                           v                                v
         +----------------------------------+    +--------------------------+
         | 3. Revisit Waypoint Generator    |    | Continue Standard        |
         |  • 3D BBox Projection            |    | Coverage Waypoints       |
         |  • Dynamic Standoff Calculation  |    +--------------------------+
         |  • Revisit Waypoint Queue to A*  |
         +----------------------------------+
```

---

## 👥 3-Person Team Roles & Structure

- **Person 1 (AI / VLM Lead)**: Zero-shot VLM integration (`raw_vlm`, `rag_vlm`), CLIP knowledge base embedding generation, prompt optimization, VLM benchmark script (`scripts/eval_vlm_benchmark.py`).
- **Person 2 (YOLO / Data Lead)**: Dataset curation (`SDNET2018`, `MBDD2025`, earthen-augmented sets), YOLOv11 model training (`scripts/train_yolov11.py`), metrics logging.
- **Person 3 (UAV / ROS2 & Sim Lead)**: Gazebo simulation environment setup (`gazebo_simulation/`), confidence-triggered revisit generator (`revisit_waypoint_generator.py`), A* path planner integration, PX4 SITL orchestration.

---

## 📁 Repository Directory Structure

```
Autonomous-Drone-Inspection/
├── docker/                 # Container definitions & startup scripts
├── docs/                   # System architecture & setup documentation
├── data/                   # Dataset manifests (SDNET2018, MBDD2025, evaluation set)
├── gazebo_simulation/      # Gazebo worlds, SDF mudbrick models & procedural crack textures
├── knowledge_base/         # Defect taxonomy JSON, prompts YAML & reference crops
├── models/                 # Cached CLIP embeddings & trained YOLO weights
├── results/                # 6-condition sweep logs, Table 4 summaries & Figure 6 plots
├── rosbags/                # ROS2 flight recording bags
├── scripts/                # Offline embedding builder, YOLO trainer, 6-condition sweep scripts
└── src/
    └── uas_earthen_inspection/   # Main ROS2 Python package
        ├── config/               # Parameters configuration (ambiguity thresholds, standoff math)
        ├── launch/               # Parameterized launch files (3x2 matrix evaluation)
        └── uas_earthen_inspection/ # Python ROS2 node sources
```

---

## 🚀 Quick Start Guide

### 1. Build ROS2 Workspace
```bash
cd Autonomous-Drone-Inspection
colcon build --symlink-install --packages-select uas_earthen_inspection
source install/setup.bash
```

### 2. Pre-compute CLIP Knowledge Base Embeddings (Person 1)
```bash
python scripts/build_clip_embeddings.py \
    --ontology knowledge_base/defect_ontology.json \
    --output models/embeddings/clip_kb_embeddings.pt
```

### 3. Launch Inspection Pipeline (3x2 Matrix Setup)
```bash
# Example: Launching RAG VLM detector with Revisit Flight Loop
ros2 launch uas_earthen_inspection inspection_pipeline.launch.py \
    detector_backend:=rag_vlm \
    flight_strategy:=revisit
```

### 4. Run Full 6-Condition Automated Evaluation Sweep
```bash
python scripts/run_6condition_sweep.py --output-dir results/sweeps
python scripts/generate_table4_figure6.py --input-dir results/sweeps --output-dir results/
```

---

## 📄 License & Attribution
Developed for Autonomous Earthen Heritage Inspection research. MIT License.
