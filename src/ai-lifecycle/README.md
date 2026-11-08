# ai-lifecycle

> AI model lifecycle management for ROS-based drone workspaces and beyond.

`ai-lifecycle` provides a clean, extensible framework for managing the full lifecycle of AI/ML models — from dependency provisioning through training, evaluation, inference, and reporting — using strict OOP principles with ABCs, dependency injection, and a registry pattern.

---

## Features

- **Layered YAML config** validated by Pydantic v2 with CLI dot-notation overrides.
- **Registry pattern** — add a new model family or dataset format with a single decorator.
- **Dependency injection** — `LifecyclePipeline` never instantiates concrete classes directly.
- **uv-powered environments** hashed from the dependency list for near-instant reuse.
- **Docker + NVIDIA GPU** support via parameterised Compose service.
- **Post-training reporter** — training curves, confusion matrix, PR curve, per-class AP, and a Markdown summary.
- **Rich TUI** + Click CLI share the same execution path.

---

## Quick Start

### Install (host-side, Python ≥ 3.11)

```bash
pip install -e ".[dev]"
# or with uv:
uv pip install -e .
```

### Run training (CLI)

```bash
ailc train -c configs/yolov11_detect.yaml --set training.epochs=50
```

### Launch TUI

```bash
ailc tui
```

### Docker workflow

```bash
# Start the container and provision the hashed uv env
ailc start -c configs/yolov11_detect.yaml

# Train inside the container
ailc train -c configs/yolov11_detect.yaml

# Stop
ailc stop -c configs/yolov11_detect.yaml
```

---

## Project Layout

```
ai-lifecycle/
├── ai_lifecycle/
│   ├── cli.py                  # Click CLI: start|stop|status|train|evaluate|infer|tui
│   ├── tui.py                  # Rich + questionary TUI
│   ├── core/
│   │   ├── base.py             # BaseManager ABC + AiLifecycleError
│   │   ├── config.py           # Pydantic schema + YAML loader
│   │   ├── registry.py         # @register_model / @register_dataset
│   │   ├── pipeline.py         # LifecyclePipeline orchestrator
│   │   ├── logging.py          # Console + file + CSV + W&B sinks
│   │   └── runs.py             # Run-dir creation + config snapshot
│   ├── dependencies/
│   │   ├── base.py             # DependencyManager ABC
│   │   └── uv_docker.py        # UvDockerDependencyManager
│   ├── datasets/
│   │   ├── base.py             # DatasetManager ABC
│   │   └── yolo_dataset.py     # YoloDatasetManager (@register_dataset("yolo"))
│   ├── models/
│   │   ├── base.py             # ModelManager ABC
│   │   └── yolov11.py          # YoloV11ModelManager (@register_model("yolov11"))
│   └── reporting/
│       └── reporter.py         # PostTrainingReporter
├── configs/
│   ├── base.yaml               # Shared defaults
│   └── yolov11_detect.yaml     # Example experiment config
├── docker/
│   ├── Dockerfile.base         # CUDA + Python + uv base image
│   ├── docker-compose.yml      # Parameterised service
│   └── entrypoint.sh           # Env bootstrap
├── outputs/                    # Experiment run directories
└── pyproject.toml
```

---

## Config Reference

Configs are layered: an experiment YAML can reference `base: configs/base.yaml` and only override what differs.

```yaml
base: configs/base.yaml          # optional layer

project: my_project
run_name: exp1

dataset:
  type: yolo                     # registered name
  path: /data/my_dataset
  train: images/train
  val: images/val
  test: images/test
  names: {0: cat, 1: dog}

model:
  type: yolov11                  # registered name
  size: s                        # n | s | m | l | x
  task: detect                   # detect | segment | classify
  weights: null                  # checkpoint path or null
  scratch: false
  loss:   {box: 7.5, cls: 0.5, dfl: 1.5}
  optimizer: {name: AdamW, lr0: 0.001, weight_decay: 0.0005}

training:
  epochs: 100
  imgsz: 640
  batch: 16
  freeze: null
  device: "0"

evaluation: {split: val, conf: 0.001, iou: 0.6}
inference:  {source: null, conf: 0.25, iou: 0.7, save: true}

logging:
  level: INFO
  to_file: true
  tensorboard: true
  csv: true
  wandb: false
  wandb_project: null

outputs:
  root: /workspace/outputs
```

CLI overrides (dot-notation):

```bash
ailc train -c configs/yolov11_detect.yaml \
    --set training.epochs=200 \
    --set model.size=m \
    --set logging.wandb=true
```

---

## Extending with a New Model

1. Create `ai_lifecycle/models/my_model.py`.
2. Subclass `ModelManager` and implement all abstract methods.
3. Decorate with `@register_model("my_model")`.
4. Set `model.type: my_model` in your config.

No other code changes required.

---

## Outputs Structure

Each run produces:

```
outputs/<project>/<run_name>_<timestamp>/
├── config.snapshot.yaml
├── summary.md
├── run.log
├── checkpoints/
├── eval/
├── metrics/
│   ├── metrics.csv
│   ├── eval_metrics.json
│   └── per_class_ap.json
└── plots/
    ├── training_curves.png
    ├── pr_curve.png
    └── confusion_matrix.png
```

---

## License

Apache-2.0
