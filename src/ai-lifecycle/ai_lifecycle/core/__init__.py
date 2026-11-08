"""Public API for the ai_lifecycle.core sub-package."""

from __future__ import annotations

from ai_lifecycle.core.base import AiLifecycleError, BaseManager
from ai_lifecycle.core.config import (
    AiLifecycleConfig,
    DatasetConfig,
    DependenciesConfig,
    EvaluationConfig,
    InferenceConfig,
    LoggingConfig,
    LossConfig,
    ModelConfig,
    OptimizerConfig,
    OutputsConfig,
    TrainingConfig,
    load_config,
)
from ai_lifecycle.core.logging import LogManager
from ai_lifecycle.core.pipeline import LifecyclePipeline
from ai_lifecycle.core.registry import (
    get_dataset_class,
    get_model_class,
    list_datasets,
    list_models,
    register_dataset,
    register_model,
)
from ai_lifecycle.core.runs import RunManager

__all__ = [
    # Exceptions
    "AiLifecycleError",
    # Base
    "BaseManager",
    # Config
    "AiLifecycleConfig",
    "DatasetConfig",
    "DependenciesConfig",
    "EvaluationConfig",
    "InferenceConfig",
    "LoggingConfig",
    "LossConfig",
    "ModelConfig",
    "OptimizerConfig",
    "OutputsConfig",
    "TrainingConfig",
    "load_config",
    # Logging
    "LogManager",
    # Pipeline
    "LifecyclePipeline",
    # Registry
    "get_dataset_class",
    "get_model_class",
    "list_datasets",
    "list_models",
    "register_dataset",
    "register_model",
    # Runs
    "RunManager",
]
