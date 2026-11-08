# Plan: `ai_lifecycle` module (YOLOv11 first)

The existing `yolo11_tool` stays as is. `ai_lifecycle` wraps it behind clean abstractions, and the YOLOv11 manager reuses its model registry.

## 1. Layout

```
yolo11-project/
├── ai_lifecycle/
│   ├── cli.py                  # common entrypoint: ailc <command>
│   ├── tui.py                  # minimal terminal UI
│   ├── core/
│   │   ├── base.py             # BaseManager ABC (logger, config, lifecycle hooks)
│   │   ├── config.py           # pydantic schema + YAML loader (layered)
│   │   ├── registry.py         # @register_model / @register_dataset
│   │   ├── pipeline.py         # LifecyclePipeline: orchestrates the stages
│   │   ├── logging.py          # console + file + CSV/TensorBoard sinks
│   │   └── runs.py             # run-dir creation, config snapshot
│   ├── dependencies/
│   │   ├── base.py             # DependencyManager ABC
│   │   └── uv_docker.py        # UvDockerDependencyManager
│   ├── datasets/
│   │   ├── base.py             # DatasetManager ABC
│   │   └── yolo_dataset.py     # YoloDatasetManager
│   ├── models/
│   │   ├── base.py             # ModelManager ABC
│   │   └── yolov11.py          # YoloV11ModelManager
│   └── reporting/
│       └── reporter.py         # plots + metrics + eval summary
├── configs/ailc/yolov11_detect.yaml
├── docker/
│   ├── Dockerfile.base
│   ├── docker-compose.yml
│   └── entrypoint.sh
└── outputs/<project>/<run_id>/ # logs, checkpoints, plots, metrics, eval, config.snapshot.yaml
```

## 2. OOP design

| Class | Role | Key abstract methods |
|---|---|---|
| `BaseManager` (ABC) | Shared logger, config section, `setup()` / `validate()` / `teardown()` | `setup`, `validate` |
| `DependencyManager` | Resolve and provision the environment | `resolve()`, `provision()`, `is_ready()`, `run_in_env()` |
| `DatasetManager` | Data handling | `validate()`, `prepare()`, `stats()`, `get_spec()` |
| `ModelManager` | Model lifecycle | `build()`, `load_checkpoint()`, `configure_loss()`, `configure_optimizer()`, `train()`, `evaluate()`, `predict()`, `save()` |
| `Reporter` | Post-training artifacts | `plot_training_curves()`, `plot_eval()`, `write_summary()` |
| `LifecyclePipeline` | Runs the stages in order | `run(stage)` |

- **Concrete classes.** `UvDockerDependencyManager`, `YoloDatasetManager` and `YoloV11ModelManager` each implement one ABC.
- **Basic dataset methods in the ABC.** `DatasetManager` provides path checks, split-ratio validation, a file-count summary and class-distribution stats. Subclasses add format-specific logic: YOLO label parsing, `data.yaml` generation and image/label pairing.
- **Dependency injection.** The pipeline receives managers through constructors and never instantiates concrete classes directly.
- **Registry and factory.** `@register_model("yolov11")` and `@register_dataset("yolo")` let the config pick the implementation. Adding another model family means adding one file and one decorator.
- **Hooks.** `ModelManager` exposes `on_train_start`, `on_epoch_end` and `on_train_end` callbacks. The logger and reporter subscribe to them, so training code never calls logging directly.
- **YOLOv11 customisation.** Ultralytics has no clean API for swapping the loss, so `YoloV11ModelManager` uses a custom `DetectionTrainer` subclass. Overriding `build_optimizer` handles optimizer changes. Wrapping `model.init_criterion` handles the loss (for example, different box/cls/dfl weights or a custom loss class). All of this is selected from config.

## 3. Config (one YAML per experiment)

```yaml
project: yolov11_detect
run_name: exp1

dependencies:
  python: "3.11"
  packages: [ultralytics>=8.3.0, torch, matplotlib, pandas]
  extra_index_url: null          # e.g. CUDA wheels
  system_packages: [libgl1]      # optional apt installs

dataset:
  type: yolo
  path: /data/my_dataset
  train: images/train
  val: images/val
  test: images/test
  names: {0: class_a, 1: class_b}

model:
  type: yolov11
  size: s
  task: detect
  weights: null                  # or path to checkpoint
  scratch: false
  loss: {box: 7.5, cls: 0.5, dfl: 1.5}
  optimizer: {name: AdamW, lr0: 0.001, weight_decay: 0.0005}

training:    {epochs: 100, imgsz: 640, batch: 16, freeze: null, device: "0"}
evaluation:  {split: val, conf: 0.001, iou: 0.6}
inference:   {source: null, conf: 0.25, iou: 0.7, save: true}
logging:     {level: INFO, to_file: true, tensorboard: true, csv: true}
outputs:     {root: /outputs}
```

The config is validated by pydantic, so errors appear before any container starts. Configs layer as `base.yaml` plus an experiment override, plus CLI `--set training.epochs=50`.

## 4. Docker, uv and compose

Each config gets its own container and its own uv environment.

- **`Dockerfile.base`** is a CUDA runtime image with Python, `uv`, git and the `ai_lifecycle` core (pydantic, pyyaml, rich). It contains no model dependencies.
- **`docker-compose.yml`** defines one parameterised service, `runner`. The project name is `ailc-<config-name>` and the container name is `ailc-${AILC_NAME}`. It mounts the source code, the dataset, `outputs/`, a shared uv cache volume and an `envs/` volume. It reserves the GPU through the NVIDIA device block.
- **`entrypoint.sh`** is the environment bootstrap:
  1. Read `dependencies:` from the config.
  2. Hash the dependency set, then check for `/envs/<hash>`.
  3. If it is missing, run `uv venv` and `uv pip install` (plus the framework's core packages). If it exists, reuse it, so a repeat `start` is near-instant.
  4. Keep the container alive, ready for process commands.
- **Command flow:**
  - `ailc start -c cfg.yaml` builds the base image if needed, runs `docker compose -p ailc-<name> up -d` and provisions the env.
  - `ailc train|evaluate|infer -c cfg.yaml` runs `docker exec` inside that container with the env's Python. If the container is not up, it runs `start` first.
  - `ailc stop` and `ailc status` handle the rest.
- **Host side.** The host CLI is a thin Python script that only needs the standard library plus pyyaml and rich. Heavy dependencies exist only inside the containers.

## 5. Post-training

After `train`, the pipeline automatically runs `evaluate` on the configured split, then the `Reporter` writes:

- `plots/`: loss curves (box/cls/dfl), mAP50 and mAP50-95 over epochs, precision/recall, learning rate, confusion matrix, and PR curve.
- `metrics/`: `train_metrics.csv`, `eval_metrics.json` and per-class AP.
- `eval/`: sample predictions on a few validation images.
- `summary.md`: the config snapshot, best epoch, final metrics and paths.

## 6. CLI and TUI

**CLI commands:** `start | stop | status | train | evaluate | infer | tui`. Running `ailc` with no arguments or `ailc tui` opens the TUI. It uses `rich` for output and `questionary` for prompts, which keeps it minimal.

**TUI flow:**
1. Choose a config, or create one from a template.
2. Choose a process: Train / Evaluate / Inference.
3. Follow-up prompts depend on the process:
   - Train: dataset path, model size and task, resume checkpoint, epochs and batch.
   - Evaluate: checkpoint path, dataset path and split.
   - Infer: checkpoint, source (file, folder, webcam, video), confidence and save options.
4. Show a summary and confirm. The TUI then writes an override config and calls the same code path as the CLI, so there is only one execution route.

## 7. Phases

1. **Core:** `BaseManager`, config schema, registry, logging and run directories.
2. **Docker and uv layer:** `Dockerfile.base`, compose file, `entrypoint.sh` and `UvDockerDependencyManager`. Verify with a dummy config.
3. **YOLOv11 managers:** `YoloDatasetManager` and `YoloV11ModelManager`, including the custom trainer, loss and optimizer hooks, and logging callbacks.
4. **Reporter** and the automatic train-then-evaluate chain.
5. **CLI and TUI.**
6. **Tests and docs:** unit tests per manager using fakes for Docker, an end-to-end smoke test on a tiny dataset, and a README.

## Assumptions to confirm

- Docker with the NVIDIA Container Toolkit is installed on the host.
- The host CLI runs on plain Python, so the heavy dependencies stay in containers.
- Dependency changes in the config trigger a new hashed env rather than modifying the old one.