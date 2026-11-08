"""Model and dataset registry using decorator-based registration.

Provides ``@register_model`` and ``@register_dataset`` decorators that
populate module-level dictionaries, plus factory helpers to retrieve the
registered classes by name.

Usage::

    from ai_lifecycle.core.registry import register_model, get_model_class

    @register_model("yolov11")
    class YoloV11ModelManager(ModelManager):
        ...

    cls = get_model_class("yolov11")
"""

from __future__ import annotations

from typing import Callable, TypeVar

T = TypeVar("T", bound=type)

_MODEL_REGISTRY: dict[str, type] = {}
_DATASET_REGISTRY: dict[str, type] = {}


# ---------------------------------------------------------------------------
# Decorators
# ---------------------------------------------------------------------------


def register_model(name: str) -> Callable[[T], T]:
    """Register a model manager class under *name*.

    Args:
        name: Unique identifier used in experiment configs (e.g. ``"yolov11"``).

    Returns:
        A class decorator that stores the class in :data:`_MODEL_REGISTRY`
        and returns it unchanged.

    Raises:
        ValueError: If *name* is already registered (guards against typos).
    """

    def decorator(cls: T) -> T:
        if name in _MODEL_REGISTRY:
            raise ValueError(
                f"Model '{name}' is already registered "
                f"(existing: {_MODEL_REGISTRY[name].__qualname__}, "
                f"new: {cls.__qualname__}). Use a unique name."
            )
        _MODEL_REGISTRY[name] = cls
        return cls

    return decorator


def register_dataset(name: str) -> Callable[[T], T]:
    """Register a dataset manager class under *name*.

    Args:
        name: Unique identifier used in experiment configs (e.g. ``"yolo"``).

    Returns:
        A class decorator that stores the class in :data:`_DATASET_REGISTRY`
        and returns it unchanged.

    Raises:
        ValueError: If *name* is already registered.
    """

    def decorator(cls: T) -> T:
        if name in _DATASET_REGISTRY:
            raise ValueError(
                f"Dataset '{name}' is already registered "
                f"(existing: {_DATASET_REGISTRY[name].__qualname__}, "
                f"new: {cls.__qualname__}). Use a unique name."
            )
        _DATASET_REGISTRY[name] = cls
        return cls

    return decorator


# ---------------------------------------------------------------------------
# Factory helpers
# ---------------------------------------------------------------------------


def get_model_class(name: str) -> type:
    """Return the model manager class registered under *name*.

    Args:
        name: The name passed to :func:`register_model`.

    Returns:
        The registered class.

    Raises:
        KeyError: With a helpful message listing available names if *name* is
                  not found.
    """
    if name not in _MODEL_REGISTRY:
        available = ", ".join(sorted(_MODEL_REGISTRY)) or "<none>"
        raise KeyError(
            f"No model registered under '{name}'. Available: {available}."
        )
    return _MODEL_REGISTRY[name]


def get_dataset_class(name: str) -> type:
    """Return the dataset manager class registered under *name*.

    Args:
        name: The name passed to :func:`register_dataset`.

    Returns:
        The registered class.

    Raises:
        KeyError: With a helpful message listing available names if *name* is
                  not found.
    """
    if name not in _DATASET_REGISTRY:
        available = ", ".join(sorted(_DATASET_REGISTRY)) or "<none>"
        raise KeyError(
            f"No dataset registered under '{name}'. Available: {available}."
        )
    return _DATASET_REGISTRY[name]


def list_models() -> list[str]:
    """Return a sorted list of registered model names."""
    return sorted(_MODEL_REGISTRY)


def list_datasets() -> list[str]:
    """Return a sorted list of registered dataset names."""
    return sorted(_DATASET_REGISTRY)
