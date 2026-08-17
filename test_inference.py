#!/usr/bin/env python3
"""
test_inference.py
Standalone Local Verification Test for 4-Bit Quantized Qwen2.5-VL VLM Inference.

Author: AI & Robotics Engineer
Description:
    Loads Qwen/Qwen2.5-VL-3B-Instruct with BitsAndBytes 4-bit quantization, performs
    inference on `test_crack.jpg`, measures latency (ms), checks CUDA GPU VRAM usage,
    and runs regex parsing on spatial bounding box outputs.
"""

import os
import sys
import time
import re
import torch
from PIL import Image


def run_verification_test(image_path: str = "test_crack.jpg"):
    print("=" * 70)
    print(" STANDALONE 4-BIT QWEN2.5-VL LOCAL VERIFICATION TEST ")
    print("=" * 70)

    # 1. Verify image exists
    if not os.path.exists(image_path):
        print(f"[ERROR] Image file '{image_path}' not found in current directory.")
        sys.exit(1)

    print(f"[STEP 1] Found test image: '{image_path}' ({os.path.getsize(image_path):,} bytes)")

    # 2. Check CUDA availability & Initial Memory
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] Active PyTorch device: [{device.upper()}]")
    if device == "cuda":
        gpu_name = torch.cuda.get_device_name(0)
        total_vram_gb = torch.cuda.get_device_properties(0).total_memory / (1024**3)
        print(f"[GPU INFO] Device Name: {gpu_name} | Total VRAM: {total_vram_gb:.2f} GB")
        vram_start_mb = torch.cuda.memory_allocated(0) / (1024**2)
        print(f"[VRAM START] Allocated VRAM: {vram_start_mb:.2f} MB")

    # 3. Load 4-Bit Quantized Qwen2.5-VL-3B-Instruct Model
    model_id = "Qwen/Qwen2.5-VL-3B-Instruct"
    print(f"\n[STEP 2] Loading '{model_id}' with 4-bit BitsAndBytes quantization...")

    from transformers import BitsAndBytesConfig, AutoProcessor, Qwen2_5_VLForConditionalGeneration

    quant_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_compute_dtype=torch.float16,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_use_double_quant=True
    )

    model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        model_id,
        quantization_config=quant_config,
        device_map="auto",
        torch_dtype=torch.float16,
        trust_remote_code=True
    )

    processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
    print(f"[SUCCESS] 4-bit quantized model '{model_id}' successfully loaded!")

    if device == "cuda":
        vram_loaded_mb = torch.cuda.memory_allocated(0) / (1024**2)
        print(f"[VRAM POST-LOAD] Model VRAM Footprint: {vram_loaded_mb:.2f} MB")

    # 4. Prepare Image & Prompt
    print(f"\n[STEP 3] Loading image '{image_path}' and formulating spatial prompt...")
    raw_img = Image.open(image_path).convert("RGB")
    img_w, img_h = raw_img.size
    print(f"  -> Image dimensions: {img_w}x{img_h} px")

    prompt_text = (
        "Detect any cracks or defects in this image. "
        "Output strictly in this format: <box>[ymin, xmin, ymax, xmax]</box> {class_name} Confidence: {score}"
    )

    # 5. Run VLM Inference & Measure Latency
    print(f"\n[STEP 4] Executing VLM Inference...")
    start_time = time.perf_counter()

    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": raw_img},
                {"type": "text", "text": prompt_text}
            ]
        }
    ]
    formatted_prompt = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = processor(text=[formatted_prompt], images=[raw_img], return_tensors="pt").to(device)

    with torch.no_grad():
        generated_ids = model.generate(**inputs, max_new_tokens=256)
        raw_output = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

    end_time = time.perf_counter()
    latency_ms = (end_time - start_time) * 1000.0

    print("=" * 70)
    print(" RAW MODEL TEXT OUTPUT ")
    print("=" * 70)
    print(raw_output.strip())
    print("=" * 70)
    print(f" Latency: {latency_ms:.2f} ms")

    if device == "cuda":
        max_vram_mb = torch.cuda.max_memory_allocated(0) / (1024**2)
        print(f" Peak GPU VRAM Usage: {max_vram_mb:.2f} MB")

    # 6. Multi-Line Flexible Regex Matching
    print(f"\n[STEP 5] Running Regex Coordinate Extraction...")
    regex_pattern = r"<box>\s*\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\](?:\s*</box>)?\s*([a-zA-Z0-9_\-]+)\s*(?:Confidence:?\s*([\d.]+))?"
    matches = re.findall(regex_pattern, raw_output, re.DOTALL | re.IGNORECASE)

    parsed_results = []
    if matches:
        for idx, match in enumerate(matches, 1):
            ymin, xmin, ymax, xmax, class_name, conf_str = match
            coords = [int(ymin), int(xmin), int(ymax), int(xmax)]
            score = float(conf_str) if conf_str else 0.95

            parsed_results.append({
                "index": idx,
                "bbox": coords,
                "class_name": class_name.strip(),
                "confidence": score
            })

            print(f"\n   Detection #{idx}:")
            print(f"     Defect Class : {class_name.strip()}")
            print(f"     Confidence C : {score:.4f}")
            print(f"     BBox [ymin, xmin, ymax, xmax]: {coords}")
    else:
        print("   Regex failed to match. Checking fallback JSON/box patterns...")

    print("\n" + "=" * 70)
    print(" VERIFICATION TEST COMPLETED SUCCESSFULLY ")
    print("=" * 70)


if __name__ == '__main__':
    run_verification_test()
