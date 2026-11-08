"""Pydantic v2 configuration schema and YAML loader for ai-lifecycle.

Supports layered configs: a ``base:`` key in an experiment YAML causes the
base file to be loaded first, then deep-merged with the experiment overrides,
and finally patched with any dot-notation CLI overrides.

Usage::

    from ai_lifecycle.core.config import load_config

    cfg = load_config("configs/yolov11_detect.yaml", {"training.epochs": "50"})
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Optional

import yaml
from pydantic import BaseModel, ConfigDict, Field


# ---------------------------------------------------------------------------
# Sub-models
# ---------------------------------------------------------------------------


class DependenciesConfig(BaseModel):
    """Python and system dependency specification."""

    model_config = ConfigDict(frozen=False)

    python: str = "3.11"
    packages: list[str] = Field(default_factory=list)
    extra_index_url: Optional[str] = None
    system_packages: list[str] = Field(default_factory=list)


class DatasetConfig(BaseModel):
    """Dataset layout and class-name mapping."""

    model_config = ConfigDict(frozen=False)

    type: str
    path: str
    train: str = "images/train"
    val: str = "images/val"
    test: str = "images/test"
    names: dict[int, str] = Field(default_factory=dict)


class LossConfig(BaseModel):
    """Loss function hyper-parameters."""

    model_config = ConfigDict(frozen=False)

    box: float = 7.5
    cls: float = 0.5
    dfl: float = 1.5


class OptimizerConfig(BaseModel):
    """Optimizer hyper-parameters."""

    model_config = ConfigDict(frozen=False)

    name: str = "AdamW"
    lr0: float = 0.001
    weight_decay: float = 0.0005


class ModelConfig(BaseModel):
    """Model architecture and initialisation options."""

    model_config = ConfigDict(frozen=False)

    type: str
    size: str = "s"
    task: str = "detect"
    weights: Optional[str] = None
    scratch: bool = False
    loss: LossConfig = Field(default_factory=LossConfig)
    optimizer: OptimizerConfig = Field(default_factory=OptimizerConfig)


class TrainingConfig(BaseModel):
    """Training hyper-parameters."""

    model_config = ConfigDict(frozen=False)

    epochs: int = 100
    imgsz: int = 640
    batch: int = 16
    freeze: Optional[int] = None
    device: str = "0"


class EvaluationConfig(BaseModel):
    """Evaluation / validation settings."""

    model_config = ConfigDict(frozen=False)

    split: str = "val"
    conf: float = 0.001
    iou: float = 0.6


class InferenceConfig(BaseModel):
    """Inference settings."""

    model_config = ConfigDict(frozen=False)

    source: Optional[str] = None
    conf: float = 0.25
    iou: float = 0.7
    save: bool = True


class LoggingConfig(BaseModel):
    """Logging sinks and verbosity."""

    model_config = ConfigDict(frozen=False)

    level: str = "INFO"
    to_file: bool = True
    tensorboard: bool = True
    csv: bool = True
    wandb: bool = False
    wandb_project: Optional[str] = None


class OutputsConfig(BaseModel):
    """Output root directory."""

    model_config = ConfigDict(frozen=False)

    root: str = "/workspace/outputs"


# ---------------------------------------------------------------------------
# Root config
# ---------------------------------------------------------------------------


class AiLifecycleConfig(BaseModel):
    """Root configuration model for an ai-lifecycle experiment.

    All fields are validated by pydantic so configuration errors surface
    before any computation begins.
    """

    model_config = ConfigDict(frozen=False)

    project: str = "default"
    run_name: str = "exp"
    dependencies: DependenciesConfig = Field(default_factory=DependenciesConfig)
    dataset: Optional[DatasetConfig] = None
    model: ModelConfig
    training: TrainingConfig = Field(default_factory=TrainingConfig)
    evaluation: EvaluationConfig = Field(default_factory=EvaluationConfig)
    inference: InferenceConfig = Field(default_factory=InferenceConfig)
    logging: LoggingConfig = Field(default_factory=LoggingConfig)
    outputs: OutputsConfig = Field(default_factory=OutputsConfig)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge *override* into *base*, returning a new dict.

    Nested dicts are merged recursively; all other types are replaced.
    """
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _apply_dot_overrides(data: dict[str, Any], overrides: dict[str, str]) -> dict[str, Any]:
    """Apply dot-notation overrides to *data* in-place (returns *data*).

    Each key in *overrides* is a dotted path such as ``"training.epochs"``
    and the value is a string that is coerced to int/float/bool/str as needed.

    Args:
        data: The mutable config dict.
        overrides: A mapping of ``"section.key"`` -> ``"value"`` strings.

    Returns:
        The mutated *data* dict.
    """

    def _coerce(v: str) -> Any:
        if v.lower() in ("true", "yes"):
            return True
        if v.lower() in ("false", "no"):
            return False
        if v.lower() in ("null", "none", "~"):
            return None
        try:
            return int(v)
        except ValueError:
            pass
        try:
            return float(v)
        except ValueError:
            pass
        return v

    for dotted_key, raw_value in overrides.items():
        parts = dotted_key.split(".")
        node = data
        for part in parts[:-1]:
            if not isinstance(node, dict):
                raise ValueError(
                    f"Cannot traverse path '{dotted_key}': '{part}' is not a dict."
                )
            node = node.setdefault(part, {})
        node[parts[-1]] = _coerce(raw_value)

    return data


def _load_yaml(path: str | Path) -> dict[str, Any]:
    """Load and parse a YAML file, returning a plain dict."""
    with open(path, "r") as fh:
        return yaml.safe_load(fh) or {}


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def load_config(
    path: str,
    overrides: dict[str, str] | None = None,
) -> AiLifecycleConfig:
    """Load, layer, and validate an ai-lifecycle experiment config.

    Steps:

    1. Load the YAML at *path*.
    2. If it contains a ``base:`` key, load that file first and deep-merge
       the experiment on top (the experiment wins on conflicts).
    3. Apply any dot-notation CLI overrides from *overrides*.
    4. Validate the result with pydantic and return an
       :class:`AiLifecycleConfig`.

    Args:
        path: Path to the experiment YAML file.
        overrides: Optional dict of dot-notation overrides, e.g.
                   ``{"training.epochs": "50"}``.

    Returns:
        A validated :class:`AiLifecycleConfig` instance.

    Raises:
        FileNotFoundError: If *path* or the referenced ``base:`` file does not
                           exist.
        pydantic.ValidationError: If the merged config fails schema validation.
    """
    overrides = overrides or {}
    config_path = Path(path).expanduser().resolve()

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    experiment_raw = _load_yaml(config_path)

    # Remove internal keys before merging
    base_ref: str | None = experiment_raw.pop("base", None)

    if base_ref:
        # Resolve base path relative to the experiment config's directory
        base_path = (config_path.parent / base_ref).resolve()
        if not base_path.exists():
            # Also try relative to CWD
            base_path = Path(base_ref).expanduser().resolve()
        if not base_path.exists():
            raise FileNotFoundError(
                f"Base config referenced by '{path}' not found: {base_ref}"
            )
        base_raw = _load_yaml(base_path)
        merged = _deep_merge(base_raw, experiment_raw)
    else:
        merged = experiment_raw

    if overrides:
        _apply_dot_overrides(merged, overrides)

    return AiLifecycleConfig.model_validate(merged)
