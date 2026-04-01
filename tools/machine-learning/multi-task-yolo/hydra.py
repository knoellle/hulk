import torch.nn as nn
import torch.nn.functional
from torch._prims_common import Tensor

from head import YOLOHead


class Hydra(nn.Module):
    def __init__(
        self, backbone: nn.ModuleList, heads: list[nn.Module], backbone_len: int
    ):
        super().__init__()
        self.backbone = backbone
        self.heads = nn.ModuleList(
            YOLOHead(head, backbone_len) for head in heads
        )

    def forward(self, x):
        y = []
        # 1. Run backbone and save EVERY intermediate output
        for m in self.backbone:
            x = m(x)
            y.append(x)

        # 2. Pass all outputs to the heads
        return {
            "detect": self.heads[0](y),
            "pose": self.heads[1](y),
        }
