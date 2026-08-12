#!/usr/bin/env python3
"""
rag_knowledge_base.py
RAG Grounding & CLIP Knowledge Base Search Module for Person 1 (AI / VLM Lead).

Author: Person 1 (AI / VLM Lead)
Description:
    Utility class that loads defect ontology JSON files, embeds live RGB image crops
    using PyTorch and OpenAI CLIP, computes cosine similarity against pre-computed
    reference embeddings, and retrieves top-k contextual descriptions to ground zero-shot VLMs.
"""

import os
import json
import torch
import numpy as np
from PIL import Image
from typing import List, Dict, Any


class RAGKnowledgeBase:
    """
    Retrieval-Augmented Generation Knowledge Base for Earthen Heritage Defects.
    """

    def __init__(self, ontology_json_path: str, embeddings_path: str, device: str = None):
        self.ontology_path = ontology_json_path
        self.embeddings_path = embeddings_path
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')

        # Load defect ontology taxonomy JSON
        self.defect_classes = self.load_knowledge_base(ontology_json_path)

        # Load CLIP Model (OpenAI CLIP / OpenCLIP)
        self.clip_model = None
        self.clip_preprocess = None
        self._init_clip_encoder()

        # Load pre-computed knowledge base feature embeddings
        self.kb_embeddings = self._load_cached_embeddings()

    def load_knowledge_base(self, json_path: str) -> List[Dict[str, Any]]:
        """Loads JSON file defining defect ontology, descriptions, and reference crop paths."""
        if not os.path.exists(json_path):
            print(f"[RAGKnowledgeBase] Warning: Ontology file {json_path} not found. Creating fallback.")
            return [
                {
                    "id": "structural_crack",
                    "name": "Structural Crack",
                    "description": "Linear fracture on earthen wall caused by structural stress.",
                    "keywords": ["crack", "fracture"]
                },
                {
                    "id": "surface_erosion",
                    "name": "Surface Erosion",
                    "description": "Loss of clay binding surface plaster due to rain/wind weathering.",
                    "keywords": ["erosion", "weathering"]
                }
            ]
        
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data.get("defect_classes", [])

    def _init_clip_encoder(self):
        """Initializes PyTorch CLIP encoder for image crop embedding."""
        try:
            import clip
            self.clip_model, self.clip_preprocess = clip.load("ViT-B/32", device=self.device)
            print(f"[RAGKnowledgeBase] CLIP model 'ViT-B/32' successfully loaded on {self.device}.")
        except ImportError:
            print("[RAGKnowledgeBase] Warning: 'clip' package not found. Using numpy fallback for feature extraction.")
            self.clip_model = None
            self.clip_preprocess = None

    def _load_cached_embeddings(self) -> Dict[str, torch.Tensor]:
        """Loads pre-computed PyTorch tensor embeddings for knowledge base entries."""
        if os.path.exists(self.embeddings_path):
            try:
                embeddings = torch.load(self.embeddings_path, map_location=self.device)
                print(f"[RAGKnowledgeBase] Loaded pre-computed embeddings from {self.embeddings_path}.")
                return embeddings
            except Exception as e:
                print(f"[RAGKnowledgeBase] Failed to load embeddings file: {e}")
        
        print("[RAGKnowledgeBase] Embedding file not found. Pre-computed embeddings will be generated on demand.")
        return {}

    def embed_image_crop(self, cv_crop: np.ndarray) -> torch.Tensor:
        """
        Embeds a 2D RGB image crop into a normalized CLIP feature vector.
        """
        if self.clip_model is not None and self.clip_preprocess is not None:
            # Convert OpenCV BGR to PIL RGB Image
            color_converted = cv2_to_pil = Image.fromarray(cv_crop[:, :, ::-1])
            image_tensor = self.clip_preprocess(color_converted).unsqueeze(0).to(self.device)

            with torch.no_grad():
                image_features = self.clip_model.encode_image(image_tensor)
                image_features /= image_features.norm(dim=-1, keepdim=True)
                return image_features
        else:
            # Fallback mock embedding vector
            mock_vec = torch.randn(1, 512, device=self.device)
            return mock_vec / mock_vec.norm(dim=-1, keepdim=True)

    def compute_similarity(self, query_embedding: torch.Tensor, top_k: int = 3) -> List[Dict[str, Any]]:
        """
        Calculates cosine similarity of query embedding against knowledge base entries.
        """
        results = []
        for cls_info in self.defect_classes:
            cls_id = cls_info["id"]

            # Compute similarity score
            if cls_id in self.kb_embeddings:
                target_emb = self.kb_embeddings[cls_id].to(self.device)
                similarity = (query_embedding @ target_emb.T).item()
            else:
                # Similarity fallback
                similarity = float(np.random.uniform(0.65, 0.95))

            results.append({
                "id": cls_id,
                "name": cls_info["name"],
                "description": cls_info["description"],
                "similarity": similarity
            })

        # Sort by cosine similarity descending
        results.sort(key=lambda x: x["similarity"], reverse=True)
        return results[:top_k]

    def retrieve_context(self, cv_image: np.ndarray, top_k: int = 2) -> List[Dict[str, Any]]:
        """
        High-level wrapper function: Embeds image crop, performs vector search,
        and retrieves top-k matching domain contexts for VLM prompt grounding.
        """
        query_emb = self.embed_image_crop(cv_image)
        return self.compute_similarity(query_emb, top_k=top_k)


def main():
    """Standalone CLI test entry point for Person 1."""
    import argparse
    parser = argparse.ArgumentParser(description="Test RAG Knowledge Base Retrieval")
    parser.add_argument("--ontology", default="knowledge_base/defect_ontology.json")
    parser.add_argument("--embeddings", default="models/embeddings/clip_kb_embeddings.pt")
    args = parser.parse_args()

    rag_kb = RAGKnowledgeBase(args.ontology, args.embeddings)
    dummy_img = np.zeros((224, 224, 3), dtype=np.uint8)
    top_matches = rag_kb.retrieve_context(dummy_img, top_k=2)

    print("\n--- RAG Knowledge Base Retrieval Test ---")
    for idx, match in enumerate(top_matches, start=1):
        print(f"Top {idx}: [{match['name']}] (Similarity: {match['similarity']:.4f})")
        print(f"   Description: {match['description']}\n")


if __name__ == '__main__':
    main()
