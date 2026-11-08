"""Run-directory creation and config snapshot utilities.

Each experiment run gets an isolated output directory whose name encodes the
project, run name and a wall-clock timestamp so that parallel runs never
collide.

Directory layout created by :class:`RunManager`::

    <outputs.root>/
    └── <project>/
        └── <run_name>_<YYYYMMDD_HHMMSS>/
            ├── checkpoints/
            ├── eval/
            ├── metrics/
            ├── plots/
            └── config.snapshot.yaml

Usage::

    from ai_lifecycle.core.runs import RunManager
    from ai_lifecycle.core.config import AiLifecycleConfig

    cfg = load_config("configs/yolov11_detect.yaml")
    rm = RunManager(config=cfg.model_dump(), full_config=cfg)
    rm.setup()
    run_dir = rm.run_dir
"""

from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from ai_lifecycle.core.base import AiLifecycleError, BaseManager


class RunManager(BaseManager):
    """Creates and manages the output directory for a single experiment run.

    Args:
        config: The ``outputs`` section of :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                serialised to a plain dict.  Must contain at least ``root``.
        full_config: The complete :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                     instance used to write the config snapshot.
        project: Project name (overrides ``full_config.project`` when given).
        run_name: Run name prefix (overrides ``full_config.run_name`` when given).
        logger: Optional pre-configured logger.
    """

    _SUBDIRS = ("checkpoints", "eval", "metrics", "plots")

    def __init__(
        self,
        config: dict[str, Any],
        full_config: Any,  # AiLifecycleConfig — avoid circular import typing
        project: str | None = None,
        run_name: str | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(config, logger)
        self._full_config = full_config
        self._project = project or getattr(full_config, "project", "default")
        self._run_name = run_name or getattr(full_config, "run_name", "exp")
        self._run_dir: Path | None = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def run_dir(self) -> Path:
        """The resolved run directory path.

        Raises:
            AiLifecycleError: If :meth:`setup` has not been called yet.
        """
        if self._run_dir is None:
            raise AiLifecycleError(
                "RunManager.run_dir accessed before setup() was called."
            )
        return self._run_dir

    # ------------------------------------------------------------------
    # BaseManager interface
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Create the run directory tree and write the config snapshot."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        root = Path(self._config.get("root", "/workspace/outputs")).expanduser()
        run_dir = root / self._project / f"{self._run_name}_{timestamp}"

        try:
            run_dir.mkdir(parents=True, exist_ok=True)
            for subdir in self._SUBDIRS:
                (run_dir / subdir).mkdir(exist_ok=True)
        except OSError as exc:
            raise AiLifecycleError(
                f"Failed to create run directory at '{run_dir}'", cause=exc
            ) from exc

        self._run_dir = run_dir
        self._write_snapshot()
        self._logger.info("Run directory created: %s", self._run_dir)

    def validate(self) -> bool:
        """Return True if the run directory exists on disk."""
        if self._run_dir is None:
            return False
        return self._run_dir.is_dir()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _write_snapshot(self) -> None:
        """Serialise the full config to ``config.snapshot.yaml`` in the run dir."""
        snapshot_path = self._run_dir / "config.snapshot.yaml"  # type: ignore[operator]
        try:
            # Pydantic v2 — model_dump() returns a plain dict.
            if hasattr(self._full_config, "model_dump"):
                data = self._full_config.model_dump()
            else:
                data = dict(self._full_config)

            with open(snapshot_path, "w", encoding="utf-8") as fh:
                yaml.dump(data, fh, default_flow_style=False, allow_unicode=True)

            self._logger.debug("Config snapshot written: %s", snapshot_path)
        except Exception as exc:
            # Non-fatal — log a warning and continue.
            self._logger.warning(
                "Failed to write config snapshot: %s", exc, exc_info=True
            )
