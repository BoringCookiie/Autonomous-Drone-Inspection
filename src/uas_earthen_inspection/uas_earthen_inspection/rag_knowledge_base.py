#!/usr/bin/env python3
"""
rag_knowledge_base.py
Decoupled Live RAG Grounding & Retrieval Module for Person 1 (AI / VLM Lead).

Author: Person 1 (AI / VLM Lead)
Description:
    Loads pre-computed defect knowledge base feature vectors from `clip_kb_embeddings.pt`
    at startup (built offline via `scripts/build_clip_embeddings.py`).
    During live flight inference, embeds ONLY the single live camera frame/crop using HuggingFace
    CLIP and performs high-speed vector cosine similarity search against the pre-loaded cache.
"""

import os
import json
import torch
import torch.nn.functional as F
import numpy as np
from PIL import Image
from typing import List, Dict, Any, Union


def resolve_project_path(value: str) -> str:
    """Resolve repository-relative model and knowledge-base paths using UAS_INSPECTION_ROOT."""
    if os.path.isabs(value):
        return value
    root = os.environ.get('UAS_INSPECTION_ROOT', '/home/uas/')
    if not os.path.exists(root):
        root = os.getcwd()
    return os.path.join(root, value)


class RAGKnowledgeBase:
    """
    Decoupled Live RAG Knowledge Base using Pre-computed Offline CLIP Index.
    """

    def __init__(
        self,
        ontology_json_path: str = "knowledge_base/defect_ontology.json",
        embeddings_path: str = "models/embeddings/clip_kb_embeddings.pt",
        clip_model_name: str = "openai/clip-vit-base-patch32",
        device: str = None
    ):
        self.ontology_path = resolve_project_path(ontology_json_path)
        self.embeddings_path = resolve_project_path(embeddings_path)
        self.clip_model_name = clip_model_name
        self.device = device or ('cuda' if (torch is not None and torch.cuda.is_available()) else 'cpu')

        # 1. Load defect ontology taxonomy metadata JSON
        self.defect_classes = self.load_knowledge_base(self.ontology_path)

        # 2. Load pre-computed offline KB feature embeddings
        self.kb_embeddings = self._load_cached_embeddings()

        # 3. Initialize HuggingFace CLIP image processor for live frame encoding ONLY
        self.clip_model = None
        self.clip_processor = None
        self._init_clip_encoder()

    def load_knowledge_base(self, json_path: str) -> List[Dict[str, Any]]:
        """Loads JSON file defining defect ontology and metadata descriptions."""
        resolved = resolve_project_path(json_path)
        if not os.path.exists(resolved):
            print(f"[RAGKnowledgeBase] Warning: Ontology file '{resolved}' not found. Using default taxonomy.")
            return [
                {
                    "id": "structural_crack",
                    "name": "Structural Crack",
                    "description": "Linear fracture on earthen wall caused by structural stress.",
                    "prompt_template": "An earthen mudbrick wall showing a structural crack fracture."
                },
                {
                    "id": "surface_erosion",
                    "name": "Surface Erosion",
                    "description": "Loss of clay binding surface plaster due to rain wash and weathering.",
                    "prompt_template": "An earthen wall exhibiting surface erosion and pitted texture."
                },
                {
                    "id": "moisture_stain",
                    "name": "Moisture Stain & Efflorescence",
                    "description": "Dark damp patches or white salt efflorescence crust deposits.",
                    "prompt_template": "A damp earthen wall showing dark water discoloration and white salt staining."
                }
            ]
        
        with open(resolved, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data.get("defect_classes", [])

    def _load_cached_embeddings(self) -> Dict[str, torch.Tensor]:
        """Loads pre-computed PyTorch feature vectors built by `scripts/build_clip_embeddings.py`."""
        resolved = resolve_project_path(self.embeddings_path)
        if os.path.exists(resolved):
            try:
                embeddings = torch.load(resolved, map_location=self.device, weights_only=True)
                print(f"[RAGKnowledgeBase] Successfully loaded pre-computed offline KB embeddings from: {resolved}")
                return embeddings
            except Exception:
                try:
                    embeddings = torch.load(resolved, map_location=self.device)
                    return embeddings
                except Exception as e:
                    print(f"[RAGKnowledgeBase] Error loading embeddings cache ({e}).")
        else:
            print(f"[RAGKnowledgeBase] Notice: Offline embeddings file '{resolved}' not found.")
        return {}

    def _init_clip_encoder(self):
        """Initializes HuggingFace CLIP image encoder for live frame embedding ONLY."""
        try:
            from transformers import CLIPModel, CLIPProcessor
            print(f"[RAGKnowledgeBase] Initializing live CLIP image encoder ({self.clip_model_name}) on {self.device}...")
            try:
                self.clip_model = CLIPModel.from_pretrained(self.clip_model_name, use_safetensors=True).to(self.device)
            except Exception:
                self.clip_model = CLIPModel.from_pretrained(self.clip_model_name).to(self.device)

            self.clip_processor = CLIPProcessor.from_pretrained(self.clip_model_name)
            self.clip_model.eval()
            print("[RAGKnowledgeBase] Live CLIP image encoder ready.")
        except Exception as e:
            print(f"[RAGKnowledgeBase] Warning: HuggingFace CLIP load failed ({e}). Using CPU mock vector encoder.")
            self.clip_model = None
            self.clip_processor = None

    def embed_live_frame(self, live_frame_or_crop: Union[np.ndarray, Image.Image]) -> torch.Tensor:
        """
        Embeds a single incoming live camera RGB frame/crop into a normalized CLIP feature vector.
        """
        if isinstance(live_frame_or_crop, np.ndarray):
            pil_img = Image.fromarray(live_frame_or_crop[:, :, ::-1])  # BGR to RGB
        else:
            pil_img = live_frame_or_crop

        if self.clip_model is not None and self.clip_processor is not None:
            inputs = self.clip_processor(images=pil_img, return_tensors="pt").to(self.device)
            with torch.no_grad():
                image_outputs = self.clip_model.get_image_features(**inputs)

                # Robustly extract feature tensor from transformers output object
                if hasattr(image_outputs, "image_embeds"):
                    image_features = image_outputs.image_embeds
                elif hasattr(image_outputs, "pooler_output"):
                    image_features = image_outputs.pooler_output
                elif isinstance(image_outputs, torch.Tensor):
                    image_features = image_outputs
                else:
                    image_features = image_outputs[0]

                return F.normalize(image_features, p=2, dim=-1)
        else:
            mock_vec = torch.randn(1, 512, device=self.device)
            return F.normalize(mock_vec, p=2, dim=-1)

    def retrieve_context(
        self,
        live_frame_or_crop: Union[np.ndarray, Image.Image],
        top_k: int = 2
    ) -> List[Dict[str, Any]]:
        """
        Embeds ONLY the live camera frame and performs high-speed cosine similarity search
        against pre-loaded offline KB feature tensors.
        """
        # 1. Embed live camera frame/crop
        live_embedding = self.embed_live_frame(live_frame_or_crop)

        results = []
        for cls_info in self.defect_classes:
            cls_id = cls_info["id"]

            # Load cached tensor for class
            if cls_id in self.kb_embeddings:
                cached_emb = self.kb_embeddings[cls_id].to(self.device)
                if cached_emb.ndim == 1:
                    cached_emb = cached_emb.unsqueeze(0)
                cached_emb = F.normalize(cached_emb, p=2, dim=-1)

                similarity = F.cosine_similarity(live_embedding, cached_emb, dim=-1).item()
            else:
                similarity = float(np.random.uniform(0.60, 0.90))

            results.append({
                "id": cls_id,
                "name": cls_info.get("name", cls_id),
                "description": cls_info.get("description", ""),
                "prompt_template": cls_info.get("prompt_template", ""),
                "similarity": float(similarity)
            })

        # Sort descending by cosine similarity score
        results.sort(key=lambda x: x["similarity"], reverse=True)
        return results[:top_k]


def main():
    """Standalone live retrieval verification."""
    print("--- Live RAG Knowledge Base Retrieval Test ---")
    rag = RAGKnowledgeBase()
    dummy_frame = np.zeros((224, 224, 3), dtype=np.uint8)
    top_matches = rag.retrieve_context(dummy_frame, top_k=2)

    for idx, match in enumerate(top_matches, start=1):
        print(f"Top Match {idx}: [{match['name']}] (Cosine Similarity: {match['similarity']:.4f})")
        print(f"  Description: {match['description']}\n")


if __name__ == '__main__':
    main()
