# 25-30 Day Team Sprint Workflow & Track Mapping

## Team Structure & Individual Ownership

| Member | Primary Track | Key Responsibilities | Deliverables |
|--------|---------------+----------------------+--------------|
| **Person 1 (Me)** | AI / VLM | Zero-shot VLM integration, RAG knowledge retrieval, CLIP feature embeddings, evaluation scripts | `rag_knowledge_base.py`, `detection_node.py` (VLM backends), `build_clip_embeddings.py` |
| **Person 2** | YOLO / Data | Dataset curation, domain augmentation, YOLOv11 model training, metrics extraction | `train_yolov11.py`, `detection_node.py` (YOLO backend), Table 4 YOLO benchmarks |
| **Person 3** | UAV / ROS2 & Sim | Gazebo mudbrick world setup, revisit waypoint generator, A* path planning integration | `revisit_waypoint_generator.py`, `per_waypoint_capture_node.py`, Gazebo world assets |

---

## 🚨 Critical Risk Flag & Mitigation Strategy (Person 3 Bottleneck)

> [!WARNING]
> **Sim/Nav Track Overlap Warning**: Person 3 is assigned both **Gazebo World Setup (Sim)** and **Revisit Generator / A* Navigation (Nav)**.
> 
> Because the Revisit Generator depends directly on the Gazebo mudbrick wall world and depth simulation being operational, collapsing both tracks onto Person 3 creates a high-risk sequential bottleneck between **Days 5 and 18**.

### Fallback & Parallel Support Plan:
1. **Days 1–5 (Early Assistance)**: Person 1 and Person 2 will assist Person 3 with procedural texture creation (crack morphology images, erosion normal maps) for the Gazebo mudbrick wall model while Person 1/2 complete early dataset/ontology definitions.
2. **Day 6 Checkpoint**: Ensure basic Gazebo mudbrick wall SDF model loads in PX4 SITL before Person 3 proceeds to A* planner integration and depth unprojection math.
3. **Mocking Depth Data**: `revisit_waypoint_generator.py` includes simulated depth map fallback options so Nav testing can proceed even if Gazebo depth rendering faces configuration delays.

---

## Sprint Timeline & Key Milestones

- **Days 1–5**: Infrastructure setup, ontology definition, dataset download, Gazebo mudbrick wall initial model setup.
- **Days 6–12**: Node development (`detection_node`, `rag_knowledge_base`, `per_waypoint_capture_node`, `revisit_waypoint_generator`).
- **Days 13–18**: Integration testing in Gazebo, standalone VLM benchmarking, YOLOv11 training complete.
- **Days 19–25**: Execute full 6-condition automated sweep (`run_6condition_sweep.py`), log ROS bags, analyze performance.
- **Days 26–30**: Aggregate final Table 4 metrics and Figure 6 plots (`generate_table4_figure6.py`), document findings.
