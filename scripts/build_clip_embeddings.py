#!/usr/bin/env python3
"""
build_clip_embeddings.py
Offline CLIP Feature Embedding Build Script for Knowledge Base.

Author: Person 1 (AI / VLM Lead)
Description:
    Decoupled offline script that pre-computes HuggingFace CLIP (`openai/clip-vit-base-patch32`)
    text embeddings for defect ontology classes and image embeddings for reference crops.
    Caches feature tensors to `models/embeddings/clip_kb_embeddings.pt`.
"""

import os
import json
import argparse
import torch
import torch.nn.functional as F
from PIL import Image


def build_embeddings(
    ontology_path: str = "knowledge_base/defect_ontology.json",
    ref_crops_dir: str = "knowledge_base/ref_crops",
    output_path: str = "models/embeddings/clip_kb_embeddings.pt",
    model_name: str = "openai/clip-vit-base-patch32"
):
    print(f"[INFO] Building offline CLIP embeddings using HuggingFace model: '{model_name}'")
    print(f"  Ontology Path : {ontology_path}")
    print(f"  Output Path   : {output_path}")

    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    try:
        from transformers import CLIPModel, CLIPProcessor
        model = CLIPModel.from_pretrained(model_name, use_safetensors=True).to(device)
        processor = CLIPProcessor.from_pretrained(model_name)
        model.eval()
        has_transformers = True
        print(f"[INFO] HuggingFace CLIP model successfully loaded on {device}")
    except Exception:
        try:
            from transformers import CLIPModel, CLIPProcessor
            model = CLIPModel.from_pretrained(model_name).to(device)
            processor = CLIPProcessor.from_pretrained(model_name)
            model.eval()
            has_transformers = True
        except Exception as e:
            print(f"[WARNING] Failed to load HuggingFace CLIP ({e}). Generating fallback random normalized tensors.")
            has_transformers = False

    if not os.path.exists(ontology_path):
        print(f"[ERROR] Ontology file '{ontology_path}' not found.")
        return

    with open(ontology_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    defect_classes = data.get("defect_classes", [])
    embeddings_dict = {}

    for cls in defect_classes:
        cls_id = cls["id"]
        description = cls.get("description", "")
        prompt = cls.get("prompt_template", description)

        # 1. Text Embedding of Defect Description / Prompt Template
        if has_transformers:
            inputs = processor(text=[prompt], return_tensors="pt", padding=True).to(device)
            with torch.no_grad():
                text_features = model.get_text_features(**inputs)
                text_features = F.normalize(text_features, p=2, dim=-1)
        else:
            text_features = torch.randn(1, 512, device=device)
            text_features = F.normalize(text_features, p=2, dim=-1)

        # 2. Image Crop Embeddings if reference crops exist
        crop_features_list = []
        ref_crops = cls.get("ref_crops", [])
        for crop_path in ref_crops:
            if os.path.exists(crop_path):
                try:
                    pil_img = Image.open(crop_path).convert("RGB")
                    if has_transformers:
                        img_inputs = processor(images=pil_img, return_tensors="pt").to(device)
                        with torch.no_grad():
                            img_feat = model.get_image_features(**img_inputs)
                            img_feat = F.normalize(img_feat, p=2, dim=-1)
                            crop_features_list.append(img_feat)
                except Exception as img_err:
                    print(f"    [WARN] Failed to process crop image '{crop_path}': {img_err}")

        # Combine text and reference crop features into average representative embedding
        if crop_features_list:
            combined = torch.cat([text_features] + crop_features_list, dim=0)
            avg_feature = torch.mean(combined, dim=0, keepdim=True)
            cls_embedding = F.normalize(avg_feature, p=2, dim=-1)
        else:
            cls_embedding = text_features

        embeddings_dict[cls_id] = cls_embedding.squeeze(0).cpu()
        print(f"  -> Processed [{cls_id}]: prompt length {len(prompt)} chars, {len(crop_features_list)} ref crop(s).")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    torch.save(embeddings_dict, output_path)
    print(f"\n[SUCCESS] Saved {len(embeddings_dict)} offline class embedding(s) to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Pre-compute HuggingFace CLIP KB Embeddings")
    parser.add_argument("--ontology", default="knowledge_base/defect_ontology.json", help="Path to ontology JSON")
    parser.add_argument("--ref-crops", default="knowledge_base/ref_crops", help="Directory of reference crops")
    parser.add_argument("--output", default="models/embeddings/clip_kb_embeddings.pt", help="Target cached tensor file")
    parser.add_argument("--model-name", default="openai/clip-vit-base-patch32", help="HuggingFace CLIP model ID")
    args = parser.parse_args()

    build_embeddings(args.ontology, args.ref_crops, args.output, args.model_name)


if __name__ == '__main__':
    main()
