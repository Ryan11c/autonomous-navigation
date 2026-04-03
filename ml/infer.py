from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="CSV file with observation features")
    parser.add_argument("--output", required=True, help="CSV file to write predictions")
    return parser.parse_args()


def load_rows(input_path: Path):
    with input_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader)


def torch_backend_available(ml_dir: Path):
    if importlib.util.find_spec("numpy") is None:
        return False, "numpy_missing"
    if importlib.util.find_spec("torch") is None:
        return False, "torch_missing"
    if not (ml_dir / "feature_stats.json").exists():
        return False, "feature_stats_missing"
    if not (ml_dir / "model.pt").exists():
        return False, "model_missing"
    return True, "torch_mlp"


def heuristic_predict(row):
    intensity = float(row["intensity"])
    local_density = float(row["local_density"])
    boundary_flag = float(row["boundary_flag"])
    obstacle_score = 0.65 * intensity + 0.25 * local_density + 0.10 * boundary_flag
    label = "obstacle" if obstacle_score >= 0.5 else "free"
    confidence = min(0.95, max(0.55, abs(obstacle_score - 0.5) + 0.5))
    return label, confidence, "heuristic_fallback"


def torch_predict(rows, ml_dir: Path):
    import torch

    from model_def import CellClassifier

    stats_path = ml_dir / "feature_stats.json"
    model_path = ml_dir / "model.pt"
    if not stats_path.exists() or not model_path.exists():
        raise FileNotFoundError("Torch model artifacts not found. Run train.py first.")

    stats = json.loads(stats_path.read_text())
    feature_order = stats["feature_order"]
    mean = torch.tensor(stats["mean"], dtype=torch.float32)
    std = torch.tensor(stats["std"], dtype=torch.float32)

    features = []
    for row in rows:
        features.append([float(row[name]) for name in feature_order])

    inputs = torch.tensor(features, dtype=torch.float32)
    normalized = (inputs - mean) / std

    model = CellClassifier(input_dim=len(feature_order))
    model.load_state_dict(torch.load(model_path, map_location="cpu"))
    model.eval()

    with torch.no_grad():
        logits = model(normalized)
        probabilities = torch.softmax(logits, dim=1)
        confidences, classes = torch.max(probabilities, dim=1)

    outputs = []
    for row, predicted_class, confidence in zip(rows, classes.tolist(), confidences.tolist()):
        outputs.append({
            "x": row["x"],
            "y": row["y"],
            "label": "obstacle" if predicted_class == 1 else "free",
            "confidence": f"{confidence:.6f}",
            "backend": "torch_mlp",
        })
    return outputs


def main():
    args = parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)
    rows = load_rows(input_path)
    ml_dir = Path(__file__).resolve().parent

    predictions = None
    torch_ready, backend_reason = torch_backend_available(ml_dir)
    if torch_ready:
        try:
            predictions = torch_predict(rows, ml_dir)
        except Exception:
            predictions = []
            for row in rows:
                label, confidence, _ = heuristic_predict(row)
                predictions.append({
                    "x": row["x"],
                    "y": row["y"],
                    "label": label,
                    "confidence": f"{confidence:.6f}",
                    "backend": "heuristic_fallback_after_torch_error",
                })
    else:
        predictions = []
        for row in rows:
            label, confidence, _ = heuristic_predict(row)
            predictions.append({
                "x": row["x"],
                "y": row["y"],
                "label": label,
                "confidence": f"{confidence:.6f}",
                "backend": backend_reason,
            })

    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["x", "y", "label", "confidence", "backend"])
        writer.writeheader()
        writer.writerows(predictions)


if __name__ == "__main__":
    main()
