from __future__ import annotations

import json
import random
from pathlib import Path

try:
    import torch
    from torch import nn
except ImportError as exc:
    raise SystemExit(
        "PyTorch is not installed. Install it first, then rerun train.py."
    ) from exc

from model_def import CellClassifier


def generate_sample(rng: random.Random):
    label = rng.randint(0, 1)

    if label == 1:
        intensity = rng.uniform(0.58, 1.0)
        local_density = rng.uniform(0.35, 1.0)
        normalized_distance = rng.uniform(0.0, 1.0)
        boundary_flag = 1.0 if rng.random() < 0.35 else 0.0
    else:
        intensity = rng.uniform(0.0, 0.55)
        local_density = rng.uniform(0.0, 0.55)
        normalized_distance = rng.uniform(0.0, 1.0)
        boundary_flag = 1.0 if rng.random() < 0.12 else 0.0

    return [intensity, local_density, normalized_distance, boundary_flag], label


def build_dataset(sample_count: int = 6000, seed: int = 7):
    rng = random.Random(seed)
    features = []
    labels = []
    for _ in range(sample_count):
        feature_row, label = generate_sample(rng)
        features.append(feature_row)
        labels.append(label)
    return torch.tensor(features, dtype=torch.float32), torch.tensor(labels, dtype=torch.long)


def normalize_features(features: torch.Tensor):
    mean = features.mean(dim=0)
    std = features.std(dim=0)
    std = torch.where(std < 1e-6, torch.ones_like(std), std)
    return (features - mean) / std, mean, std


def main():
    torch.manual_seed(7)

    features, labels = build_dataset()
    normalized_features, mean, std = normalize_features(features)

    model = CellClassifier(input_dim=normalized_features.shape[1])
    optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
    loss_fn = nn.CrossEntropyLoss()

    for epoch in range(250):
        logits = model(normalized_features)
        loss = loss_fn(logits, labels)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if epoch % 50 == 0:
            predictions = logits.argmax(dim=1)
            accuracy = (predictions == labels).float().mean().item()
            print(f"epoch={epoch} loss={loss.item():.4f} acc={accuracy:.4f}")

    ml_dir = Path(__file__).resolve().parent
    model_path = ml_dir / "model.pt"
    stats_path = ml_dir / "feature_stats.json"

    torch.save(model.state_dict(), model_path)
    stats_path.write_text(json.dumps({
        "feature_order": [
            "intensity",
            "local_density",
            "normalized_distance",
            "boundary_flag",
        ],
        "mean": mean.tolist(),
        "std": std.tolist(),
    }, indent=2))

    print(f"saved torch model to {model_path}")
    print(f"saved feature stats to {stats_path}")


if __name__ == "__main__":
    main()

