from __future__ import annotations

import torch
from torch import nn


class CellClassifier(nn.Module):
    def __init__(self, input_dim: int = 4):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(input_dim, 16),
            nn.ReLU(),
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Linear(8, 2),
        )

    def forward(self, x):
        return self.layers(x)

