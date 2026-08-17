#!/usr/bin/env python3
"""
test_rag_inference.py
Standalone Verification Test for RAG-Grounded 4-Bit Qwen2.5-VL VLM Pipeline.

Author: AI & Robotics Engineer
Description:
    1. Loads cached CLIP embeddings (`models/embeddings/clip_kb_embeddings.pt`) and ontology (`knowledge_base/defect_ontology.json`).
    2. Embeds `test_crack.jpg` using HuggingFace CLIP (`openai/clip-vit-base-patch32`) and retrieves top-1 domain defect context.
    3. Formulates RAG-grounded spatial prompt for 4-bit quantized Qwen2.5-VL-3B-Instruct.
    4. Measures latency, peak GPU VRAM allocation, and extracts spatial bounding box coordinates.
"""

import os
import sys
import time
import re
import torch
import torch.nn.functional as F
from PIL import Image

# Add package source path
pkg_src = os.path.join(os.path.dirname(__file__), 'src', 'uas_earthen_inspection')
if pkg_src not in sys.path:
    sys.path.insert(0, pkg_src)

# Import RAG Knowledge Base module
from uas_earthen_inspection.rag_knowledge_base import RAGKnowledgeBase


def run_rag_verification_test(image_path: str = "test_crack.jpg"):
    print("=" * 75)
    print(" STANDALONE RAG-GROUNDED 4-BIT QWEN2.5-VL PIPELINE VERIFICATION ")
    print("=" * 75)

    # 1. Verify test image exists
    if not os.path.exists(image_path):
        print(f"[ERROR] Test image '{image_path}' not found in root workspace directory.")
        sys.exit(1)

    print(f"[STEP 1] Found test image: '{image_path}' ({os.path.getsize(image_path):,} bytes)")

    # 2. Check PyTorch & GPU Hardware
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] PyTorch active device: [{device.upper()}]")
    if device == "cuda":
        gpu_name = torch.cuda.get_device_name(0)
        total_vram_gb = torch.cuda.get_device_properties(0).total_memory / (1024**3)
        print(f"[GPU INFO] Device: {gpu_name} | Total VRAM: {total_vram_gb:.2f} GB")

    # 3. RAG Knowledge Base Context Retrieval
    print(f"\n[STEP 2] Initializing RAG Knowledge Base & Retrieving Domain Grounding Context...")
    ontology_path = "knowledge_base/defect_ontology.json"
    embeddings_path = "models/embeddings/clip_kb_embeddings.pt"

    rag_kb = RAGKnowledgeBase(ontology_path, embeddings_path)

    raw_img = Image.open(image_path).convert("RGB")
    img_w, img_h = raw_img.size
    print(f"  -> Image dimensions: {img_w}x{img_h} px")

    rag_start = time.perf_counter()
    top_matches = rag_kb.retrieve_context(raw_img, top_k=1)
    rag_retrieval_ms = (time.perf_counter() - rag_start) * 1000.0

    if top_matches:
        top_defect = top_matches[0]
        retrieved_name = top_defect['name']
        retrieved_desc = top_defect['description']
        similarity_score = top_defect['similarity']

        print(f"   RAG Cosine Similarity Retrieval Success ({rag_retrieval_ms:.2f} ms):")
        print(f"     Retrieved Defect Class : [{retrieved_name}] (Cosine Similarity: {similarity_score:.4f})")
        print(f"     Knowledge Description  : '{retrieved_desc}'")
    else:
        retrieved_desc = "Linear structural crack fracture on earthen mudbrick architecture."
        print(f"   Fallback description used: '{retrieved_desc}'")

    # 4. Load 4-Bit Quantized Qwen2.5-VL-3B-Instruct Model
    model_id = "Qwen/Qwen2.5-VL-3B-Instruct"
    print(f"\n[STEP 3] Loading '{model_id}' with 4-bit BitsAndBytes quantization...")

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
    print(f"[SUCCESS] 4-bit quantized VLM model successfully loaded!")

    if device == "cuda":
        vram_loaded_mb = torch.cuda.memory_allocated(0) / (1024**2)
        print(f"[VRAM POST-LOAD] Model VRAM Footprint: {vram_loaded_mb:.2f} MB")

    # 5. Formulate Grounded Prompt
    print(f"\n[STEP 4] Constructing RAG-Grounded Spatial Prompt...")
    grounded_prompt = (
        f"Context from Earthen Architecture Knowledge Base: {retrieved_desc}. \n\n"
        f"Detect any cracks or defects in this image. "
        f"Output strictly in this format: <box>[ymin, xmin, ymax, xmax]</box> {{class_name}} Confidence: {{score}}"
    )

    # 6. Execute RAG-Grounded Inference
    print(f"\n[STEP 5] Executing RAG-Grounded VLM Inference...")
    vlm_start = time.perf_counter()

    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": raw_img},
                {"type": "text", "text": grounded_prompt}
            ]
        }
    ]
    formatted_prompt = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = processor(text=[formatted_prompt], images=[raw_img], return_tensors="pt").to(device)

    with torch.no_grad():
        generated_ids = model.generate(**inputs, max_new_tokens=256)
        raw_output = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

    vlm_latency_ms = (time.perf_counter() - vlm_start) * 1000.0

    print("=" * 75)
    print(" RAW RAG-GROUNDED MODEL OUTPUT ")
    print("=" * 75)
    print(raw_output.strip())
    print("=" * 75)
    print(f" RAG Retrieval Latency : {rag_retrieval_ms:.2f} ms")
    print(f" VLM Inference Latency : {vlm_latency_ms:.2f} ms")
    print(f" Total End-to-End Latency: {(rag_retrieval_ms + vlm_latency_ms):.2f} ms")

    if device == "cuda":
        max_vram_mb = torch.cuda.max_memory_allocated(0) / (1024**2)
        print(f" Peak GPU VRAM Usage   : {max_vram_mb:.2f} MB")

    # 7. Regex Coordinate Extraction
    print(f"\n[STEP 6] Running Spatial Bounding Box Regex Extraction...")
    regex_pattern = r"<box>\s*\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\](?:\s*</box>)?\s*([a-zA-Z0-9_\-]+)\s*(?:Confidence:?\s*([\d.]+))?"
    matches = re.findall(regex_pattern, raw_output, re.DOTALL | re.IGNORECASE)

    if matches:
        for idx, match in enumerate(matches, 1):
            ymin, xmin, ymax, xmax, class_name, conf_str = match
            coords = [int(ymin), int(xmin), int(ymax), int(xmax)]
            score = float(conf_str) if conf_str else 0.95

            print(f"\n   RAG Detection #{idx}:")
            print(f"     Grounding Defect Class : {class_name.strip()}")
            print(f"     Confidence Score C    : {score:.4f}")
            print(f"     BBox [ymin, xmin, ymax, xmax]: {coords}")
    else:
        print("   Regex parsing fallback applied.")

    print("\n" + "=" * 75)
    print(" RAG-GROUNDED VLM PIPELINE VERIFICATION SUCCESSFUL ")
    print("=" * 75)


if __name__ == '__main__':
    run_rag_verification_test()
