import torch
import torch.nn as nn


class YOLOHead(nn.Module):
    def __init__(self, layers: nn.ModuleList, offset: int):
        super().__init__()
        self.layers = layers
        self.offset = (
            offset  # The index where this head starts in the original model
        )

    def forward(self, backbone_outputs: list):
        # 'y' stores the outputs of layers within THIS head
        # We start by including all backbone outputs for skip connections
        y = list(backbone_outputs)

        x = backbone_outputs[-1]  # Start with the last backbone output

        for module in self.layers:
            # YOLO layers have an 'f' attribute (index of the source layer)
            if module.f != -1:
                if isinstance(module.f, int):
                    # Pull a single previous output
                    x = y[module.f]
                else:
                    # Concat layer: pull a list of outputs (e.g., [-1, 6])
                    x = [x if j == -1 else y[j] for j in module.f]

            x = module(x)
            y.append(x)  # Save this layer's output for future skip connections

        return x
