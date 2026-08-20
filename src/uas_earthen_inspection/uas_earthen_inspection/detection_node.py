#!/usr/bin/env python3
"""
detection_node.py
Core AI Defect Detection ROS2 Node with 4-bit Quantized Qwen2.5-VL, YOLOv11, and Strict Grounding Parsing.

Author: Person 1 (AI / VLM Lead)
Description:
    ROS2 Node wrapping the verified inspection detector backends (`raw_vlm`, `rag_vlm`, `yolo`).
    Subscribes strictly to `/inspection/captured_frame` (decoupled per-waypoint frame capture),
    enforces strict lazy loading (instantiating ONLY the selected backend), parses structured JSON
    or spatial grounding outputs, and publishes `vision_msgs/msg/Detection2DArray` messages
    to `/inspection/detections` preserving input ROS header timestamp and frame_id.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import json
import re
from PIL import Image as PILImage
from abc import ABC, abstractmethod
from typing import List, Dict, Any

# Import RAG Knowledge Base helper module
from uas_earthen_inspection.rag_knowledge_base import RAGKnowledgeBase, resolve_project_path


class BaseDetector(ABC):
    """Abstract Base Interface for Defect Detectors."""

    @abstractmethod
    def detect(self, cv_image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Runs detection on an OpenCV RGB image.
        Returns:
            list of dicts: [{
                'bbox': [xmin, ymin, xmax, ymax],
                'class_id': str,
                'confidence': float  # 0.0 to 1.0
            }]
        """
        pass


class Qwen25VLDetector(BaseDetector):
    """
    Qwen2.5-VL 4-Bit Quantized Vision-Language Model Detector with Lazy Loading & Structured JSON Output Parsing.
    """

    MODEL_ID = "Qwen/Qwen2.5-VL-3B-Instruct"

    def __init__(self, mode: str = "rag_vlm", rag_kb: RAGKnowledgeBase = None):
        self.mode = mode  # 'raw_vlm' or 'rag_vlm'
        self.rag_kb = rag_kb
        self.device = "cuda" if (os.environ.get('FORCE_CPU') != '1' and torch_cuda_is_available()) else "cpu"

        print(f"[Qwen25VLDetector] Initializing {self.MODEL_ID} in mode: [{self.mode.upper()}] on {self.device}")

        self.model = None
        self.processor = None
        self._load_quantized_model()

    def _load_quantized_model(self):
        """Loads Qwen2.5-VL-3B-Instruct with 4-bit BitsAndBytes quantization & accelerate auto-mapping."""
        try:
            import torch
            from transformers import BitsAndBytesConfig, AutoProcessor

            quantization_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_compute_dtype=torch.float16,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_use_double_quant=True
            )

            try:
                from transformers import Qwen2_5_VLForConditionalGeneration
                model_cls = Qwen2_5_VLForConditionalGeneration
            except ImportError:
                from transformers import AutoModelForCausalLM
                model_cls = AutoModelForCausalLM

            print(f"[Qwen25VLDetector] Loading 4-bit model weights...")
            self.model = model_cls.from_pretrained(
                self.MODEL_ID,
                quantization_config=quantization_config,
                device_map="auto",
                torch_dtype=torch.float16,
                trust_remote_code=True
            )

            self.processor = AutoProcessor.from_pretrained(
                self.MODEL_ID,
                trust_remote_code=True
            )
            print(f"[Qwen25VLDetector] Successfully loaded 4-bit quantized {self.MODEL_ID}.")

        except Exception as e:
            print(f"[ERROR] [Qwen25VLDetector] Failed to load 4-bit model ({e}). VLM inference unavailable.")
            self.model = None
            self.processor = None

    def construct_prompt(self, cv_image: np.ndarray) -> str:
        """Constructs prompt requesting structured JSON output format."""
        if self.mode == "rag_vlm" and self.rag_kb is not None:
            top_k_defects = self.rag_kb.retrieve_context(cv_image, top_k=1)
            if top_k_defects:
                top_d = top_k_defects[0]
                retrieved_context = f"{top_d['name']}: {top_d['description']}"
            else:
                retrieved_context = "Structural Crack: Linear structural crack fracture on earthen wall."

            prompt = (
                f"Context from Earthen Architecture Knowledge Base: {retrieved_context}.\n\n"
                f"Analyze the image using this context. Output strictly as a JSON object in this format:\n"
                f'{{"detections": [{{"class_id": "<class_name>", "bbox_xyxy": [xmin, ymin, xmax, ymax], "confidence": <float_score>}}]}}'
            )
        else:
            prompt = (
                f"Inspect this earthen heritage wall for defects (structural_crack, surface_erosion, moisture_stain).\n"
                f"Output strictly as a JSON object in this format:\n"
                f'{{"detections": [{{"class_id": "<class_name>", "bbox_xyxy": [xmin, ymin, xmax, ymax], "confidence": <float_score>}}]}}'
            )
        return prompt

    def detect(self, cv_image: np.ndarray) -> List[Dict[str, Any]]:
        h, w, _ = cv_image.shape
        prompt = self.construct_prompt(cv_image)

        if self.model is None or self.processor is None:
            print("[ERROR] [Qwen25VLDetector] Model not initialized. Returning empty detection list.")
            return []

        try:
            import torch
            pil_img = PILImage.fromarray(cv_image[:, :, ::-1])  # BGR to RGB
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "image", "image": pil_img},
                        {"type": "text", "text": prompt}
                    ]
                }
            ]
            text_input = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
            inputs = self.processor(text=[text_input], images=[pil_img], return_tensors="pt").to(self.device)

            with torch.no_grad():
                generated_ids = self.model.generate(**inputs, max_new_tokens=256)
                output_text = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

            return self.parse_vlm_response(output_text, w, h)

        except Exception as e:
            print(f"[ERROR] [Qwen25VLDetector] Inference failed: {e}")
            return []

    def parse_vlm_response(self, text: str, img_w: int, img_h: int) -> List[Dict[str, Any]]:
        """
        Parses model response into unified detection structure:
        {"bbox": [xmin, ymin, xmax, ymax], "class_id": str, "confidence": float}
        Supports structured JSON first, with spatial regex as fallback.
        Returns [] if parsing fails.
        """
        # 1. Try JSON parsing
        try:
            json_match = re.search(r"\{.*\"detections\".*\}", text, re.DOTALL)
            if json_match:
                data = json.loads(json_match.group(0))
                raw_dets = data.get("detections", [])
                parsed_list = []
                for d in raw_dets:
                    bbox = d.get("bbox_xyxy", d.get("bbox", [0, 0, img_w, img_h]))
                    cls_id = str(d.get("class_id", "defect")).strip("{}")
                    conf = float(d.get("confidence", 0.85))
                    parsed_list.append({
                        "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
                        "class_id": cls_id,
                        "confidence": min(1.0, max(0.0, conf))
                    })
                if parsed_list:
                    return parsed_list
        except Exception:
            pass

        # 2. Fallback spatial regex parsing: [ymin, xmin, ymax, xmax] {class_name} Confidence: {score}
        box_pattern = r"(?:<box>)?\s*\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\](?:\s*</box>)?\s*(?:\{([^}]+)\}|([^{\n]+))\s*(?:Confidence:?\s*([\d.]+))?"
        matches = re.findall(box_pattern, text, re.DOTALL | re.IGNORECASE)

        if matches:
            parsed_list = []
            for match in matches:
                ymin_raw, xmin_raw, ymax_raw, xmax_raw, label_braced, label_unbraced, conf_raw = match
                ymin = max(0, min(img_h, int(ymin_raw)))
                xmin = max(0, min(img_w, int(xmin_raw)))
                ymax = max(0, min(img_h, int(ymax_raw)))
                xmax = max(0, min(img_w, int(xmax_raw)))
                conf = float(conf_raw) if conf_raw else 0.85

                raw_label = label_braced if label_braced else label_unbraced
                clean_label = raw_label.strip()

                parsed_list.append({
                    "bbox": [xmin, ymin, xmax, ymax],
                    "class_id": clean_label,
                    "confidence": min(1.0, max(0.0, conf))
                })
            return parsed_list

        print(f"[ERROR] [Qwen25VLDetector] Failed to parse VLM response: '{text.strip()}'")
        return []


class YOLOv11Detector(BaseDetector):
    """
    Supervised YOLOv11 Detector Backend.
    Lazy loaded: Does NOT import or load Transformers, CLIP, or Qwen models into VRAM.
    """

    def __init__(self, weights_path: str):
        resolved_weights = resolve_project_path(weights_path)
        self.weights_path = resolved_weights
        self.model = None
        print(f"[YOLOv11Detector] Initializing YOLOv11 backend from: '{resolved_weights}'")

        if os.path.exists(resolved_weights):
            try:
                from ultralytics import YOLO
                self.model = YOLO(resolved_weights)
                print(f"[YOLOv11Detector] Successfully loaded YOLOv11 model weights.")
            except Exception as e:
                print(f"[ERROR] [YOLOv11Detector] Failed to load YOLO weights ({e}).")
        else:
            print(f"[ERROR] [YOLOv11Detector] Weights file not found: '{resolved_weights}'.")

    def detect(self, cv_image: np.ndarray) -> List[Dict[str, Any]]:
        h, w, _ = cv_image.shape
        if self.model is None:
            print("[ERROR] [YOLOv11Detector] Model not initialized. Returning empty detection list.")
            return []

        try:
            results = self.model(cv_image)[0]
            detections = []
            for box in results.boxes:
                xyxy = box.xyxy[0].cpu().numpy().tolist()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                cls_name = self.model.names[cls_id]
                detections.append({
                    "bbox": [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])],
                    "class_id": cls_name,
                    "confidence": conf
                })
            return detections
        except Exception as e:
            print(f"[ERROR] [YOLOv11Detector] Inference failed: {e}")
            return []


def torch_cuda_is_available() -> bool:
    """Safely check if CUDA is available without throwing exception if PyTorch is uninstalled."""
    try:
        import torch
        return torch.cuda.is_available()
    except ImportError:
        return False


class DetectionNode(Node):
    """
    ROS2 Node for AI Defect Detection wrapping RAG-VLM / Raw-VLM / YOLO backends.
    Subscribes strictly to `/inspection/captured_frame` (decoupled per-waypoint frame capture)
    and publishes `vision_msgs/msg/Detection2DArray` to `/inspection/detections`.
    """

    def __init__(self):
        super().__init__('detection_node')

        # Declare parameters
        self.declare_parameter('detector_backend', 'rag_vlm')  # 'raw_vlm' | 'rag_vlm' | 'yolo'
        self.declare_parameter('captured_frame_topic', '/inspection/captured_frame')
        self.declare_parameter('detections_topic', '/inspection/detections')
        self.declare_parameter('ontology_json_path', 'knowledge_base/defect_ontology.json')
        self.declare_parameter('clip_embeddings_path', 'models/embeddings/clip_kb_embeddings.pt')
        self.declare_parameter('yolo_weights_path', 'models/yolo/yolo_earthen_v11.pt')

        # Retrieve parameter values
        self.backend_type = self.get_parameter('detector_backend').value
        self.frame_topic = self.get_parameter('captured_frame_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value

        self.bridge = CvBridge()
        self.detector = None

        self.get_logger().info(f"[DetectionNode] Initializing active backend ONLY: [{self.backend_type.upper()}]")

        # Strict Backend Separation & Lazy Loading
        if self.backend_type == 'raw_vlm':
            self.detector = Qwen25VLDetector(mode='raw_vlm', rag_kb=None)
        elif self.backend_type == 'rag_vlm':
            ontology_path = self.get_parameter('ontology_json_path').value
            embeddings_path = self.get_parameter('clip_embeddings_path').value
            rag_kb = RAGKnowledgeBase(ontology_path, embeddings_path)
            self.detector = Qwen25VLDetector(mode='rag_vlm', rag_kb=rag_kb)
        elif self.backend_type == 'yolo':
            weights_path = self.get_parameter('yolo_weights_path').value
            self.detector = YOLOv11Detector(weights_path)
        else:
            raise ValueError(f"Unknown detector backend: {self.backend_type}")

        # Subscriptions & Publishers
        self.sub_frame = self.create_subscription(
            Image, self.frame_topic, self.frame_callback, 10
        )
        self.pub_detections = self.create_publisher(
            Detection2DArray, self.detections_topic, 10
        )

        self.get_logger().info(f"[DetectionNode] Subscribed to captured frame topic: '{self.frame_topic}'")
        self.get_logger().info(f"[DetectionNode] Publishing detections to: '{self.detections_topic}'")

    def frame_callback(self, msg: Image):
        """
        Callback processing captured camera frame into vision_msgs/msg/Detection2DArray.
        Preserves original ROS header timestamp and frame_id.
        """
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Conversion Error: {e}")
            return

        raw_detections = self.detector.detect(cv_img)
        self.get_logger().info(f"[{self.backend_type.upper()}] Detected {len(raw_detections)} defect(s) in captured frame.")

        detection_array_msg = Detection2DArray()
        # Preserve original ROS header timestamp and frame_id from captured_frame
        detection_array_msg.header = msg.header

        for det in raw_detections:
            d2d = Detection2D()
            d2d.header = msg.header

            xmin, ymin, xmax, ymax = det['bbox']
            d2d.bbox.center.position.x = float((xmin + xmax) / 2.0)
            d2d.bbox.center.position.y = float((ymin + ymax) / 2.0)
            d2d.bbox.size_x = float(abs(xmax - xmin))
            d2d.bbox.size_y = float(abs(ymax - ymin))

            hyp = ObjectHypothesisWithPose()
            cls_id = det.get('class_id', det.get('class_name', 'defect'))
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = float(det['confidence'])
            d2d.results.append(hyp)

            detection_array_msg.detections.append(d2d)

            self.get_logger().info(
                f"  -> Defect Class: '{cls_id}' | Conf C: {det['confidence']:.2f} | BBox Center: ({d2d.bbox.center.position.x:.1f}, {d2d.bbox.center.position.y:.1f}), Size: ({d2d.bbox.size_x:.1f}x{d2d.bbox.size_y:.1f})"
            )

        self.pub_detections.publish(detection_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DetectionNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
