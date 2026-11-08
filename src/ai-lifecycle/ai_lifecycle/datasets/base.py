"""Abstract base class for dataset managers.

A :class:`DatasetManager` handles all dataset-related operations:
path validation, split-ratio checks, format-specific preparation
(e.g. generating a ``data.yaml``), per-class statistics, and returning
a model-ready dataset specification.

Concrete methods provided here (not abstract) cover generic filesystem
operations so subclasses only need to implement format-specific logic.
"""

from __future__ import annotations

import logging
from abc import abstractmethod
from pathlib import Path
from typing import Any

from ai_lifecycle.core.base import AiLifecycleError, BaseManager


class DatasetManager(BaseManager):
    """Abstract manager for dataset preparation and validation.

    Extends :class:`~ai_lifecycle.core.base.BaseManager` with:

    **Concrete helpers** (no need to override):

    - :meth:`check_paths` — verify the dataset root exists.
    - :meth:`count_files` — count image files in a split directory.
    - :meth:`validate_split_ratios` — verify that each split has files.
    - :meth:`summary` — return per-split file counts.

    **Abstract methods** (must be implemented by each format):

    - :meth:`prepare` — format-specific dataset preparation.
    - :meth:`stats` — per-class label statistics.
    - :meth:`get_spec` — model-ready dataset specification (e.g. path to
      ``data.yaml``).

    Args:
        config: The ``dataset`` section of
                :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                serialised to a plain dict.
        logger: Optional pre-configured logger.
    """

    # Image extensions recognised across all formats.
    _IMAGE_EXTENSIONS: frozenset[str] = frozenset(
        {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"}
    )

    def __init__(
        self,
        config: dict[str, Any],
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(config, logger)
        self._dataset_root: Path | None = None

    # ------------------------------------------------------------------
    # BaseManager interface
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Resolve and store the dataset root path."""
        raw_path = self._config.get("path", "")
        if not raw_path:
            raise AiLifecycleError("Dataset config must include a 'path' field.")
        self._dataset_root = Path(raw_path).expanduser().resolve()
        self._logger.debug("Dataset root resolved: %s", self._dataset_root)

    def validate(self) -> bool:
        """Validate that the dataset root exists.

        Returns:
            ``True`` when the root directory is found on disk.

        Raises:
            AiLifecycleError: If ``setup()`` has not been called yet.
        """
        return self.check_paths()

    # ------------------------------------------------------------------
    # Concrete helper methods
    # ------------------------------------------------------------------

    def check_paths(self) -> bool:
        """Return True if the dataset root directory exists.

        Raises:
            AiLifecycleError: If :meth:`setup` has not been called.
        """
        if self._dataset_root is None:
            raise AiLifecycleError(
                "DatasetManager.check_paths() called before setup()."
            )
        exists = self._dataset_root.is_dir()
        if not exists:
            self._logger.warning("Dataset root not found: %s", self._dataset_root)
        return exists

    def count_files(self, split: str) -> int:
        """Count image files in the given split sub-directory.

        Args:
            split: A key from the config, e.g. ``"train"``, ``"val"``,
                   or ``"test"``.  Its value must be a relative path like
                   ``"images/train"``.

        Returns:
            Number of image files found (0 if the path does not exist).
        """
        if self._dataset_root is None:
            raise AiLifecycleError(
                "DatasetManager.count_files() called before setup()."
            )
        split_rel = self._config.get(split, "")
        split_path = self._dataset_root / split_rel
        if not split_path.is_dir():
            return 0
        return sum(
            1
            for f in split_path.iterdir()
            if f.suffix.lower() in self._IMAGE_EXTENSIONS
        )

    def validate_split_ratios(self) -> bool:
        """Check that train and val splits each contain at least one file.

        The ``test`` split is optional — many public YOLO datasets only ship
        train and val.  A warning is logged if test is empty but ``True`` is
        still returned as long as train and val are non-empty.

        Returns:
            ``True`` when train and val are non-empty.
        """
        required = ("train", "val")
        optional = ("test",)
        results = {}
        for split in (*required, *optional):
            count = self.count_files(split)
            results[split] = count
            if count == 0:
                level = "warning" if split in required else "info"
                getattr(self._logger, level)(
                    "Split '%s' has no image files.", split
                )

        all_required_ok = all(results[s] > 0 for s in required)
        if not all_required_ok:
            self._logger.warning("Split-ratio validation failed: %s", results)
        return all_required_ok

    def summary(self) -> dict[str, int]:
        """Return a dict of split name → image file count.

        Returns:
            e.g. ``{"train": 4200, "val": 600, "test": 200}``
        """
        return {
            split: self.count_files(split)
            for split in ("train", "val", "test")
        }

    # ------------------------------------------------------------------
    # Abstract interface (format-specific)
    # ------------------------------------------------------------------

    @abstractmethod
    def prepare(self) -> None:
        """Perform format-specific dataset preparation.

        For YOLO this means generating ``data.yaml``.  Other formats
        may need to convert annotations, build index files, etc.

        Raises:
            AiLifecycleError: On any preparation failure.
        """

    @abstractmethod
    def stats(self) -> dict[str, Any]:
        """Return per-class label statistics.

        Returns:
            A dict whose exact shape is format-specific, e.g.::

                {
                    "total_instances": 12400,
                    "per_class": {0: 4200, 1: 8200},
                }
        """

    @abstractmethod
    def get_spec(self) -> dict[str, Any]:
        """Return the model-ready dataset specification.

        For YOLO this is ``{"data_yaml": "/path/to/data.yaml"}``.
        Other formats may return raw tensors, HuggingFace dataset objects,
        etc.

        Returns:
            A format-specific spec dict consumed by the model manager.
        """
