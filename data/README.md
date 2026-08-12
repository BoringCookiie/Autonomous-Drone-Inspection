# Datasets Directory Manifest

This directory houses dataset assets used for YOLOv11 training and evaluation of zero-shot VLM defect detection.

## Structure
- `SDNET2018/`: Concrete and masonry crack dataset used for pre-training/fine-tuning.
- `MBDD2025/`: Masonry & Building Defect Dataset 2025.
- `earthen_augmented/`: Domain-adapted synthetic and augmented earthen heritage defect crops (cracks, erosion, moisture stains).
- `evaluation_set/`: 30–50 hand-labeled high-resolution ground truth evaluation frames captured from Gazebo mudbrick world and real-world earthen walls.

## Usage
Datasets should be downloaded into their respective folders. Large binary files are excluded from Git version control via `.gitignore`.
