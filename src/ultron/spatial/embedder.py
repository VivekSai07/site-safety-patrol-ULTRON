"""Image and text embeddings from the CLIP TorchScript model ultralytics ships for YOLOE.

`clip_b32_datacomp.ts` exports both encoders in one file. The repo's
`mobileclip_blt.ts` (fetched for YOLOE text prompts) only scripts the text
path, so it cannot re-identify objects by appearance. `load_embedder`
returns None when torch, ultralytics or the weights are missing; callers
then fall back to geometry-only behaviour.
"""
import logging

import numpy as np

logger = logging.getLogger(__name__)

WEIGHT = "clip_b32_datacomp.ts"
DIM = 512


class ClipEmbedder:
    """CLIP B/32 (datacomp) image and text encoder; vectors are unit-norm float32[512]."""

    def __init__(self, weight=WEIGHT, device=None):
        import torch
        from ultralytics.nn.text_model import MobileCLIPTS

        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.model = MobileCLIPTS(device=self.device, weight=weight)
        if not hasattr(self.model, "image_preprocess"):
            raise ValueError(f"{weight} does not export an image encoder")

    def embed_image(self, image):
        from PIL import Image

        if not isinstance(image, Image.Image):
            image = Image.fromarray(np.asarray(image, dtype=np.uint8))
        tensor = self.model.image_preprocess(image.convert("RGB")).unsqueeze(0).to(self.device)
        return np.asarray(self.model.encode_image(tensor)[0].cpu(), dtype=np.float32)

    def embed_text(self, text):
        return np.asarray(self.model.encode_text(self.model.tokenize([text]))[0].cpu(), dtype=np.float32)


def load_embedder(weight=WEIGHT, device=None):
    """The embedder, or None (geometry-only degradation) when the stack is unavailable."""
    try:
        return ClipEmbedder(weight=weight, device=device)
    except Exception as error:  # any missing dependency, file or GPU trouble
        logger.warning("spatial memory runs without appearance matching: %s", error)
        return None
