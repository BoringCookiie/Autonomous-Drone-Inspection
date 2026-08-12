#!/usr/bin/env python3
"""
train_yolov11.py
YOLOv11 Training Script for Earthen Heritage Defect Detection.

Author: Person 2 (YOLO / Data Lead)
Description:
    Trains YOLOv11 detector on curated SDNET2018 + MBDD2025 + earthen augmented datasets,
    saving fine-tuned model weights to `models/yolo/yolo_earthen_v11.pt`.
"""

import os
import argparse


def train_yolo(data_yaml: str, epochs: int, batch_size: int, output_weights: str):
    print("=" * 60)
    print("YOLOv11 Earthen Heritage Defect Detector Training Pipeline")
    print(f" Data Config   : {data_yaml}")
    print(f" Epochs        : {epochs}")
    print(f" Batch Size    : {batch_size}")
    print(f" Target Weights: {output_weights}")
    print("=" * 60)

    try:
        from ultralytics import YOLO
        # Load baseline YOLOv11 small model
        model = YOLO('yolov11s.pt')

        if os.path.exists(data_yaml):
            print("[INFO] Starting PyTorch training loop...")
            model.train(
                data=data_yaml,
                epochs=epochs,
                imgsz=640,
                batch=batch_size,
                project='models/yolo',
                name='train_run',
                exist_ok=True
            )
            # Save final trained weights
            final_pt = os.path.join('models/yolo', 'train_run', 'weights', 'best.pt')
            if os.path.exists(final_pt):
                os.makedirs(os.path.dirname(output_weights), exist_ok=True)
                os.system(f"cp {final_pt} {output_weights}")
                print(f"[SUCCESS] Trained weights saved to {output_weights}")
        else:
            print(f"[WARNING] Data yaml file '{data_yaml}' not found. Please populate data/ directory first.")

    except ImportError:
        print("[WARNING] 'ultralytics' package not installed. Install via `pip install ultralytics`.")


def main():
    parser = argparse.ArgumentParser(description="Train YOLOv11 Earthen Defect Model")
    parser.add_argument("--data-yaml", default="data/earthen_defects.yaml", help="Path to YOLO dataset YAML")
    parser.add_argument("--epochs", type=int, default=50, help="Number of training epochs")
    parser.add_argument("--batch-size", type=int, default=16, help="Training batch size")
    parser.add_argument("--output", default="models/yolo/yolo_earthen_v11.pt", help="Target output weights path")
    args = parser.parse_args()

    train_yolo(args.data_yaml, args.epochs, args.batch_size, args.output)


if __name__ == '__main__':
    main()
