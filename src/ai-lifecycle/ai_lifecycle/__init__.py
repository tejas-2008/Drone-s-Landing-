"""Public API for the ai_lifecycle package."""

from __future__ import annotations

from ai_lifecycle.core.base import AiLifecycleError, BaseManager
from ai_lifecycle.core.config import AiLifecycleConfig, load_config
from ai_lifecycle.core.pipeline import LifecyclePipeline
from ai_lifecycle.core.registry import (
    get_dataset_class,
    get_model_class,
    register_dataset,
    register_model,
)
from ai_lifecycle.core.runs import RunManager

# Force registration of built-in implementations.
import ai_lifecycle.datasets.yolo_dataset  # noqa: F401
import ai_lifecycle.models.yolov11  # noqa: F401

__version__ = "0.1.0"

__all__ = [
    "__version__",
    # Exceptions
    "AiLifecycleError",
    # Core
    "BaseManager",
    "AiLifecycleConfig",
    "load_config",
    "LifecyclePipeline",
    "RunManager",
    # Registry
    "register_model",
    "register_dataset",
    "get_model_class",
    "get_dataset_class",
]
