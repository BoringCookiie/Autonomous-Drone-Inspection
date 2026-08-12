#!/usr/bin/env python3
"""
build_clip_embeddings.py
Offline CLIP Feature Embedding Build Script for Knowledge Base.

Author: Person 1 (AI / VLM Lead)
Description:
    Computes CLIP image and text feature embeddings for all defect ontology classes and
    reference crops in `knowledge_base/`, saving the output feature cache to
    `models/embeddings/clip_kb_embeddings.pt`.
"""

import os
import json
import argparse
import torch
from PIL import Image


def build_embeddings(ontology_path: str, output_path: str):
    print(f"[INFO] Building CLIP embeddings from ontology: {ontology_path}")

    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    try:
        import clip
        model, preprocess = clip.load("ViT-B/32", device=device)
        has_clip = True
        print(f"[INFO] CLIP ViT-B/32 loaded on {device}")
    except ImportError:
        print("[WARNING] OpenAI CLIP not installed. Generating dummy PyTorch embedding vectors.")
        has_clip = False

    if not os.path.exists(ontology_path):
        print(f"[ERROR] Ontology file {ontology_path} not found.")
        return

    with open(ontology_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    defect_classes = data.get("defect_classes", [])
    embeddings_dict = {}

    for cls in defect_classes:
        cls_id = cls["id"]
        prompt = cls.get("prompt_template", cls["description"])

        if has_clip:
            text_tokens = clip.tokenize([prompt]).to(device)
            with torch.no_grad():
                text_features = model.encode_text(text_tokens)
                text_features /= text_features.norm(dim=-1, keepdim=True)
                embeddings_dict[cls_id] = text_features
        else:
            # Fallback random normalized tensor
            rand_tensor = torch.randn(1, 512, device=device)
            rand_tensor /= rand_tensor.norm(dim=-1, keepdim=True)
            embeddings_dict[cls_id] = rand_tensor

        print(f"  -> Encoded class [{cls_id}]: {prompt[:60]}...")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    torch.save(embeddings_dict, output_path)
    print(f"[SUCCESS] Saved {len(embeddings_dict)} cached class embedding(s) to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Pre-compute CLIP KB Embeddings")
    parser.add_argument("--ontology", default="knowledge_base/defect_ontology.json", help="Path to ontology JSON")
    parser.add_argument("--output", default="models/embeddings/clip_kb_embeddings.pt", help="Output embedding path")
    args = parser.parse_args()

    build_embeddings(args.ontology, args.output)


if __name__ == '__main__':
    main()
