"""Structured logging utilities for ai-lifecycle.

Sets up a root logger (or a named child logger) with:

- A **console handler** using Rich for human-friendly output.
- An optional **rotating file handler** writing plain text.
- An optional **CSV metrics logger** (a separate helper, not a log handler).
- Optional **Weights & Biases** initialisation.

Usage::

    from ai_lifecycle.core.logging import LogManager
    from ai_lifecycle.core.config import LoggingConfig

    cfg = LoggingConfig(level="DEBUG", to_file=True, csv=True)
    mgr = LogManager(config=cfg.model_dump(), run_dir="/tmp/run_001")
    mgr.setup()
    mgr.logger.info("Training started")
    mgr.log_metrics({"epoch": 1, "loss": 0.42})
"""

from __future__ import annotations

import csv
import logging
import logging.handlers
import os
from pathlib import Path
from typing import Any, Optional

from ai_lifecycle.core.base import AiLifecycleError, BaseManager

# Rich is a required dependency; import lazily to give a clear error if missing.
try:
    from rich.logging import RichHandler
except ImportError as _rich_err:  # pragma: no cover
    raise ImportError(
        "The 'rich' package is required. Install it with: pip install rich"
    ) from _rich_err


class LogManager(BaseManager):
    """Manages all logging sinks for an experiment run.

    Configures a named Python logger with:

    - A ``RichHandler`` for coloured console output.
    - An optional ``RotatingFileHandler`` writing to ``<run_dir>/run.log``.
    - A CSV metrics file at ``<run_dir>/metrics/metrics.csv``.
    - Optional W&B session initialisation.

    Args:
        config: The ``logging`` section of :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                serialised to a plain dict.
        run_dir: Absolute path to the run output directory created by
                 :class:`~ai_lifecycle.core.runs.RunManager`.
        logger: Optional pre-configured logger.  If *None* a new logger named
                ``"ailc"`` is created.
    """

    _CSV_FIELDNAMES_SENTINEL = "_ailc_csv_init"

    def __init__(
        self,
        config: dict[str, Any],
        run_dir: str,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(config, logger)
        self._run_dir = Path(run_dir)
        self._ailc_logger: logging.Logger | None = None
        self._csv_path: Path | None = None
        self._csv_writer: csv.DictWriter | None = None
        self._csv_file: Any = None  # open file handle

    # ------------------------------------------------------------------
    # BaseManager interface
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Configure all requested logging sinks."""
        level_name: str = self._config.get("level", "INFO").upper()
        level = getattr(logging, level_name, logging.INFO)

        # Create a dedicated logger so we don't pollute the root logger.
        ailc_logger = logging.getLogger("ailc")
        ailc_logger.setLevel(level)
        ailc_logger.propagate = False

        # Remove existing handlers to avoid duplicate output on re-setup.
        for h in list(ailc_logger.handlers):
            ailc_logger.removeHandler(h)

        # -- Console handler (Rich) --
        console_handler = RichHandler(
            level=level,
            show_time=True,
            show_level=True,
            show_path=False,
            markup=True,
        )
        console_handler.setLevel(level)
        ailc_logger.addHandler(console_handler)

        # -- File handler --
        if self._config.get("to_file", False):
            log_file = self._run_dir / "run.log"
            log_file.parent.mkdir(parents=True, exist_ok=True)
            file_handler = logging.handlers.RotatingFileHandler(
                str(log_file),
                maxBytes=10 * 1024 * 1024,  # 10 MB
                backupCount=5,
                encoding="utf-8",
            )
            file_handler.setLevel(level)
            fmt = logging.Formatter(
                "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
                datefmt="%Y-%m-%dT%H:%M:%S",
            )
            file_handler.setFormatter(fmt)
            ailc_logger.addHandler(file_handler)

        self._ailc_logger = ailc_logger
        self._logger = ailc_logger  # expose via BaseManager.logger

        # -- CSV metrics sink --
        if self._config.get("csv", False):
            metrics_dir = self._run_dir / "metrics"
            metrics_dir.mkdir(parents=True, exist_ok=True)
            self._csv_path = metrics_dir / "metrics.csv"
            # File opened lazily on first log_metrics call.

        # -- W&B --
        if self._config.get("wandb", False):
            self._init_wandb()

        ailc_logger.debug("LogManager setup complete (run_dir=%s)", self._run_dir)

    def validate(self) -> bool:
        """Return True if the logger has been configured."""
        return self._ailc_logger is not None

    def teardown(self) -> None:
        """Flush and close all handlers."""
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
            self._csv_writer = None

        if self._ailc_logger:
            for handler in list(self._ailc_logger.handlers):
                try:
                    handler.flush()
                    handler.close()
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def log_metrics(self, metrics: dict[str, Any]) -> None:
        """Append a row of metrics to the CSV sink (if enabled).

        Args:
            metrics: A flat dict of metric name -> scalar value.  The first
                     call determines the column headers.
        """
        if self._csv_path is None:
            return

        if self._csv_writer is None:
            # First call — open the file and write the header.
            self._csv_file = open(self._csv_path, "w", newline="", encoding="utf-8")
            fieldnames = list(metrics.keys())
            self._csv_writer = csv.DictWriter(
                self._csv_file, fieldnames=fieldnames, extrasaction="ignore"
            )
            self._csv_writer.writeheader()

        self._csv_writer.writerow(metrics)
        self._csv_file.flush()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _init_wandb(self) -> None:
        """Initialise a Weights & Biases run if the package is available."""
        try:
            import wandb  # type: ignore[import]
        except ImportError:
            self._logger.warning(
                "W&B requested but 'wandb' is not installed. Skipping."
            )
            return

        project = self._config.get("wandb_project") or "ai-lifecycle"
        try:
            wandb.init(project=project, resume="allow")
            self._logger.info("W&B initialised (project=%s)", project)
        except Exception as exc:
            raise AiLifecycleError("Failed to initialise W&B", cause=exc) from exc
