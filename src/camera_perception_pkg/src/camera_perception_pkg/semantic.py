"""Thin inference adapter for the unmodified upstream PIDNet-S model."""
import numpy as np
import torch
from torch.nn import functional as F
from .pidnet_model.pidnet import PIDNet


class Segmenter:
    def __init__(self, weights, device='cuda'):
        self.device = torch.device(device)
        torch.set_num_threads(4)
        torch.backends.cudnn.benchmark = self.device.type == 'cuda'
        # Retain training heads so strict loading checks every checkpoint tensor.
        self.model = PIDNet(m=2, n=3, num_classes=6, planes=32,
                            ppm_planes=96, head_planes=128, augment=True)
        state = torch.load(weights, map_location='cpu', weights_only=True)
        state = state.get('state_dict', state)
        state = {key[6:] if key.startswith('model.') else key: value for key, value in state.items()}
        self.model.load_state_dict(state, strict=True)
        self.model.to(self.device).eval()
        self.mean = [0.485, 0.456, 0.406]
        self.std = [0.229, 0.224, 0.225]

    @torch.inference_mode()
    def __call__(self, rgb):
        # Same RGB normalization and full-resolution interpolation as Morai6.
        x = rgb.astype(np.float32) / 255.0
        x -= self.mean
        x /= self.std
        x = torch.from_numpy(x.transpose(2, 0, 1).copy()).unsqueeze(0).to(self.device)
        logits = self.model(x)[1]
        logits = F.interpolate(logits, rgb.shape[:2], mode='bilinear', align_corners=True)
        return logits.softmax(1)[0].permute(1, 2, 0).cpu().numpy()
