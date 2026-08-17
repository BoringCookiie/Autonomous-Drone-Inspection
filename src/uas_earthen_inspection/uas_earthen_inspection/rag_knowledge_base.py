#!/usr/bin/env python3
"""
rag_knowledge_base.py
RAG Grounding & CLIP Knowledge Base Search Module for Person 1 (AI / VLM Lead).

Author: Person 1 (AI / VLM Lead)
Description:
    Utility class that loads defect ontology JSON files, embeds live RGB image crops
    and text descriptions using HuggingFace transformers (`CLIPModel`, `CLIPProcessor`) and PyTorch,
    computes cosine similarity against pre-computed or live reference embeddings, and retrieves
    top-k contextual defect descriptions to ground zero-shot VLMs.
"""

import os
import json
import torch
import torch.nn.functional as F
import numpy as np
from PIL import Image
from typing import List, Dict, Any, Union


class RAGKnowledgeBase:
    """
    Retrieval-Augmented Generation Knowledge Base using HuggingFace Transformers CLIP.
    """

    def __init__(
        self,
        ontology_json_path: str = "knowledge_base/defect_ontology.json",
        embeddings_path: str = "models/embeddings/clip_kb_embeddings.pt",
        clip_model_name: str = "openai/clip-vit-base-patch32",
        device: str = None
    ):
        self.ontology_path = ontology_json_path
        self.embeddings_path = embeddings_path
        self.clip_model_name = clip_model_name
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')

        # 1. Load defect ontology taxonomy JSON
        self.defect_classes = self.load_knowledge_base(self.ontology_path)

        # 2. Initialize HuggingFace Transformers CLIP model & processor
        self.clip_model = None
        self.clip_processor = None
        self._init_clip_encoder()

        # 3. Load pre-computed knowledge base feature embeddings if available
        self.kb_embeddings = self._load_cached_embeddings()

    def load_knowledge_base(self, json_path: str) -> List[Dict[str, Any]]:
        """Loads JSON file defining defect ontology, descriptions, and reference crop paths."""
        if not os.path.exists(json_path):
            print(f"[RAGKnowledgeBase] Warning: Ontology file {json_path} not found. Using default taxonomy.")
            return [
                {
                    "id": "structural_crack",
                    "name": "Structural Crack",
                    "description": "Linear fracture on earthen wall caused by structural stress.",
                    "prompt_template": "An earthen mudbrick wall showing a structural crack fracture.",
                    "keywords": ["crack", "fracture"]
                },
                {
                    "id": "surface_erosion",
                    "name": "Surface Erosion",
                    "description": "Loss of clay binding surface plaster due to rain and wind weathering.",
                    "prompt_template": "An earthen wall exhibiting surface erosion and pitted texture.",
                    "keywords": ["erosion", "weathering"]
                },
                {
                    "id": "moisture_stain",
                    "name": "Moisture Stain & Efflorescence",
                    "description": "Dark damp patches or white salt efflorescence crust deposits.",
                    "prompt_template": "A damp earthen wall showing dark water discoloration and white salt staining.",
                    "keywords": ["moisture", "efflorescence", "damp"]
                }
            ]
        
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data.get("defect_classes", [])

    def _init_clip_encoder(self):
        """Initializes HuggingFace Transformers CLIP model and processor."""
        try:
            from transformers import CLIPModel, CLIPProcessor
            print(f"[RAGKnowledgeBase] Loading HuggingFace CLIP ({self.clip_model_name}) on {self.device}...")
            self.clip_model = CLIPModel.from_pretrained(self.clip_model_name).to(self.device)
            self.clip_processor = CLIPProcessor.from_pretrained(self.clip_model_name)
            self.clip_model.eval()
            print("[RAGKnowledgeBase] HuggingFace CLIP encoder successfully loaded.")
        except Exception as e:
            print(f"[RAGKnowledgeBase] Warning: Failed to load HuggingFace CLIP transformers model ({e}). Using CPU/numpy fallback.")
            self.clip_model = None
            self.clip_processor = None

    def _load_cached_embeddings(self) -> Dict[str, torch.Tensor]:
        """Loads pre-computed PyTorch tensor embeddings for knowledge base entries."""
        if os.path.exists(self.embeddings_path):
            try:
                embeddings = torch.load(self.embeddings_path, map_location=self.device)
                print(f"[RAGKnowledgeBase] Loaded cached embeddings from {self.embeddings_path}.")
                return embeddings
            except Exception as e:
                print(f"[RAGKnowledgeBase] Failed to load cached embeddings: {e}")
        return {}

    def embed_image_crop(self, cv_crop: Union[np.ndarray, Image.Image]) -> torch.Tensor:
        """
        Embeds a live RGB camera frame or crop into a normalized CLIP feature vector using HuggingFace.
        """
        if isinstance(cv_crop, np.ndarray):
            # Convert BGR (OpenCV) to PIL Image RGB
            pil_img = Image.fromarray(cv_crop[:, :, ::-1])
        else:
            pil_img = cv_crop

        if self.clip_model is not None and self.clip_processor is not None:
            inputs = self.clip_processor(images=pil_img, return_tensors="pt").to(self.device)
            with torch.no_grad():
                image_features = self.clip_model.get_image_features(**inputs)
                # Normalize features to unit norm
                image_features = F.normalize(image_features, p=2, dim=-1)
                return image_features
        else:
            # Fallback normalized random feature vector
            mock_vec = torch.randn(1, 512, device=self.device)
            return F.normalize(mock_vec, p=2, dim=-1)

    def embed_text_prompt(self, text: str) -> torch.Tensor:
        """
        Embeds a textual description into a normalized CLIP feature vector using HuggingFace.
        """
        if self.clip_model is not None and self.clip_processor is not None:
            inputs = self.clip_processor(text=[text], return_tensors="pt", padding=True).to(self.device)
            with torch.no_grad():
                text_features = self.clip_model.get_text_features(**inputs)
                text_features = F.normalize(text_features, p=2, dim=-1)
                return text_features
        else:
            mock_vec = torch.randn(1, 512, device=self.device)
            return F.normalize(mock_vec, p=2, dim=-1)

    def compute_cosine_similarity(
        self,
        query_image_embedding: torch.Tensor,
        top_k: int = 2
    ) -> List[Dict[str, Any]]:
        """
        Computes cosine similarity between live drone camera frame/crop embedding and defect reference entries,
        returning top-k defect descriptions.
        """
        results = []

        for cls_info in self.defect_classes:
            cls_id = cls_info["id"]
            description = cls_info.get("description", "")
            prompt_template = cls_info.get("prompt_template", description)

            # Check if reference embedding exists in cache or compute on the fly
            if cls_id in self.kb_embeddings:
                ref_emb = self.kb_embeddings[cls_id].to(self.device)
                if ref_emb.ndim == 1:
                    ref_emb = ref_emb.unsqueeze(0)
                ref_emb = F.normalize(ref_emb, p=2, dim=-1)
            else:
                ref_emb = self.embed_text_prompt(prompt_template)

            # Compute Cosine Similarity
            similarity = F.cosine_similarity(query_image_embedding, ref_emb, dim=-1).item()

            results.append({
                "id": cls_id,
                "name": cls_info.get("name", cls_id),
                "description": description,
                "prompt_template": prompt_template,
                "similarity": float(similarity)
            })

        # Sort descending by similarity
        results.sort(key=lambda x: x["similarity"], reverse=True)
        return results[:top_k]

    def retrieve_context(
        self,
        live_frame_or_crop: Union[np.ndarray, Image.Image],
        top_k: int = 2
    ) -> List[Dict[str, Any]]:
        """
        Retrieves top-k knowledge-grounded defect descriptions for a live drone camera frame.
        """
        query_emb = self.embed_image_crop(live_frame_or_crop)
        return self.compute_cosine_similarity(query_emb, top_k=top_k)


def main():
    """Standalone verification entry point."""
    print("--- Testing RAGKnowledgeBase with Transformers CLIP ---")
    rag = RAGKnowledgeBase()
    dummy_frame = np.zeros((224, 224, 3), dtype=np.uint8)
    top_k_defects = rag.retrieve_context(dummy_frame, top_k=2)

    for i, res in enumerate(top_k_defects, 1):
        print(f"Match {i}: [{res['name']}] - Cosine Similarity: {res['similarity']:.4f}")
        print(f"  Description: {res['description']}\n")


if __name__ == '__main__':
    main()
