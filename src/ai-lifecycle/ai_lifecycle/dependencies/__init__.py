"""Public API for the ai_lifecycle.dependencies sub-package."""

from __future__ import annotations

from ai_lifecycle.dependencies.base import DependencyManager
from ai_lifecycle.dependencies.uv_docker import UvDockerDependencyManager

__all__ = [
    "DependencyManager",
    "UvDockerDependencyManager",
]
