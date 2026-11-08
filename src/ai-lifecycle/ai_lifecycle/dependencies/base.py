"""Abstract base class for dependency managers.

A :class:`DependencyManager` is responsible for resolving, provisioning, and
validating the Python (and optionally system) environment required to run an
experiment.

Concrete implementations should extend this class and decorate with the
appropriate registry decorator when a registry-based lookup is desired.
"""

from __future__ import annotations

import logging
from abc import abstractmethod
from typing import Any

from ai_lifecycle.core.base import BaseManager


class DependencyManager(BaseManager):
    """Abstract manager for Python environment provisioning.

    Extends :class:`~ai_lifecycle.core.base.BaseManager` with environment-
    specific abstract methods:

    - :meth:`resolve` — compute the final ordered list of packages.
    - :meth:`provision` — install packages into the environment.
    - :meth:`is_ready` — check whether the environment is usable.
    - :meth:`run_in_env` — execute a command inside the provisioned env.

    Args:
        config: The ``dependencies`` section of
                :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                serialised to a plain dict.
        logger: Optional pre-configured logger.
    """

    def __init__(
        self,
        config: dict[str, Any],
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(config, logger)

    # ------------------------------------------------------------------
    # DependencyManager abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def resolve(self) -> list[str]:
        """Compute the final, ordered list of packages to install.

        Returns:
            A list of PEP 508 package specifiers, e.g.
            ``["ultralytics>=8.3.0", "torch>=2.0"]``.
        """

    @abstractmethod
    def provision(self) -> None:
        """Install the resolved packages into the managed environment.

        Should be idempotent — calling it on an already-provisioned
        environment is a no-op or a fast path.

        Raises:
            AiLifecycleError: If provisioning fails.
        """

    @abstractmethod
    def is_ready(self) -> bool:
        """Return True if the environment exists and is usable.

        This check must be cheap (no network calls) so it can be used as a
        guard before every pipeline stage.
        """

    @abstractmethod
    def run_in_env(self, cmd: list[str]) -> int:
        """Run *cmd* inside the provisioned environment.

        Args:
            cmd: The command and arguments to execute, e.g.
                 ``["python", "-c", "import torch; print(torch.__version__)"]``.

        Returns:
            The process exit code.

        Raises:
            AiLifecycleError: If the command cannot be launched.
        """
