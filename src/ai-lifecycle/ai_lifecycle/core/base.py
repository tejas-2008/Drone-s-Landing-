"""Base manager ABC and custom exception for the ai-lifecycle module."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any


class AiLifecycleError(Exception):
    """Custom exception raised by ai-lifecycle managers on unrecoverable errors."""

    def __init__(self, message: str, cause: BaseException | None = None) -> None:
        super().__init__(message)
        self.cause = cause

    def __str__(self) -> str:
        base = super().__str__()
        if self.cause:
            return f"{base} (caused by: {self.cause!r})"
        return base


class BaseManager(ABC):
    """Abstract base class for all lifecycle managers.

    Provides:
    - A shared logger (auto-named after the concrete class).
    - A config dict scoped to the relevant section.
    - A context-manager protocol that calls ``setup()`` on entry and
      ``teardown()`` on exit.

    Subclasses *must* implement :meth:`setup` and :meth:`validate`.
    :meth:`teardown` is optional and defaults to a no-op.
    """

    def __init__(
        self,
        config: dict[str, Any],
        logger: logging.Logger | None = None,
    ) -> None:
        """Initialise the manager with a config dict and an optional logger.

        Args:
            config: A dictionary containing the configuration for this manager.
                    Typically a section sliced from the validated
                    :class:`~ai_lifecycle.core.config.AiLifecycleConfig`.
            logger: An optional pre-configured :class:`logging.Logger`.
                    If *None* a logger named after the concrete class is used.
        """
        self._config: dict[str, Any] = config
        self._logger: logging.Logger = logger or logging.getLogger(
            self.__class__.__name__
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def config(self) -> dict[str, Any]:
        """The configuration dict this manager was initialised with."""
        return self._config

    @property
    def logger(self) -> logging.Logger:
        """The logger bound to this manager instance."""
        return self._logger

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def setup(self) -> None:
        """Initialise resources required by this manager.

        Called automatically when used as a context manager, or explicitly
        before the first call to :meth:`validate` / the main operation.
        """

    @abstractmethod
    def validate(self) -> bool:
        """Validate configuration and internal state.

        Returns:
            ``True`` when all invariants hold, ``False`` otherwise.

        Raises:
            AiLifecycleError: On any fatal validation failure.
        """

    # ------------------------------------------------------------------
    # Optional override
    # ------------------------------------------------------------------

    def teardown(self) -> None:
        """Release resources held by this manager.

        Safe to call multiple times. Subclasses may override to flush
        buffers, close file handles, etc.
        """

    # ------------------------------------------------------------------
    # Context-manager protocol
    # ------------------------------------------------------------------

    def __enter__(self) -> BaseManager:
        self.setup()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: Any,
    ) -> bool:
        self.teardown()
        return False  # do not suppress exceptions
