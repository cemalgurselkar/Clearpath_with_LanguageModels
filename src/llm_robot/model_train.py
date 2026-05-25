#!/usr/bin/env python3

import json
import torch
from pathlib import Path
from ultralytics import YOLO


CONFIG_PATH = "train_config.json"

DEFAULT_CONFIG = {
    "model":        "yolov8n.pt",
    "data":         "/home/cemal/ros2_ws/src/llm_robot/data/YOLODataset/dataset.yaml",
    "epochs":       50,
    "imgsz":        640,
    "batch":        4,
    "lr0":          0.01,
    "lrf":          0.01,
    "momentum":     0.937,
    "weight_decay": 0.0005,
    "patience":     10,
    "project":      "runs/train",
    "name":         "exp",
    "pretrained":   True,
    "optimizer":    "SGD",
    "val":          True,
    "save":         True,
    "exist_ok":     False,
}


def check_cuda():
    if torch.cuda.is_available():
        device = "cuda"
        print(f"CUDA: {torch.cuda.get_device_name(0)}")
        print(f"VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
    else:
        device = "cpu"
        print("CUDA bulunamadı, CPU kullanılıyor.")
    return device


def load_config():
    path = Path(CONFIG_PATH)
    if path.exists():
        with open(path) as f:
            cfg = json.load(f)
        print(f"Config yüklendi: {CONFIG_PATH}")
        merged = DEFAULT_CONFIG.copy()
        merged.update(cfg)
        return merged
    with open(path, "w") as f:
        json.dump(DEFAULT_CONFIG, f, indent=4)
    print(f"Yeni config oluşturuldu: {CONFIG_PATH}")
    return DEFAULT_CONFIG.copy()


def train(cfg, device):
    model = YOLO(cfg["model"])

    model.train(
        data         = cfg["data"],
        epochs       = cfg["epochs"],
        imgsz        = cfg["imgsz"],
        batch        = cfg["batch"],
        lr0          = cfg["lr0"],
        lrf          = cfg["lrf"],
        momentum     = cfg["momentum"],
        weight_decay = cfg["weight_decay"],
        patience     = cfg["patience"],
        project      = cfg["project"],
        name         = cfg["name"],
        pretrained   = cfg["pretrained"],
        optimizer    = cfg["optimizer"],
        val          = cfg["val"],
        save         = cfg["save"],
        exist_ok     = cfg["exist_ok"],
        device       = device,
    )

    best = Path(cfg["project"]) / cfg["name"] / "weights" / "best.pt"
    cfg["model"] = str(best)
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=4)
    print(f"Eğitim tamamlandı. En iyi model: {best}")


if __name__ == "__main__":
    device = check_cuda()
    cfg    = load_config()
    train(cfg, device)