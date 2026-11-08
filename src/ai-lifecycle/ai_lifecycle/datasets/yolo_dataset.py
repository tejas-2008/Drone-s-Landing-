"""YOLO dataset manager.

Registered under the name ``"yolo"`` so experiment configs can select it
with ``dataset.type: yolo``.

Responsibilities:

- Generate a ``data.yaml`` file that ultralytics expects.
- Parse YOLO ``.txt`` label files to compute per-class instance counts.
- Return the path to the generated ``data.yaml`` as the dataset spec.
- Validate that each split has matching image and label directories.
"""

from __future__ import annotations

import logging
from collections import defaultdict
from pathlib import Path
from typing import Any

import yaml

from ai_lifecycle.core.base import AiLifecycleError
from ai_lifecycle.core.registry import register_dataset
from ai_lifecycle.datasets.base import DatasetManager


@register_dataset("yolo")
class YoloDatasetManager(DatasetManager):
    """YOLO-format dataset manager.

    Generates a ``data.yaml`` compatible with ultralytics and parses
    YOLO ``.txt`` label files for class-distribution statistics.

    Args:
        config: The ``dataset`` section of
                :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                serialised to a plain dict.  Expected keys:

                - ``path`` — absolute dataset root.
                - ``train``, ``val``, ``test`` — relative paths to image dirs.
                - ``names`` — ``{int: str}`` class-name map.

        logger: Optional pre-configured logger.
    """

    def __init__(
        self,
        config: dict[str, Any],
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(config, logger)
        self._data_yaml_path: Path | None = None

    # ------------------------------------------------------------------
    # BaseManager overrides
    # ------------------------------------------------------------------

    def validate(self) -> bool:
        """Extended validation: check images *and* labels dirs are paired.

        Returns:
            ``True`` when all split image dirs exist and each has a
            corresponding labels directory at the same level.
        """
        if not self.check_paths():
            return False

        ok = True
        for split in ("train", "val", "test"):
            split_rel: str = self._config.get(split, "")
            if not split_rel:
                continue
            images_path = self._dataset_root / split_rel  # type: ignore[operator]
            labels_path = Path(str(images_path).replace("images", "labels", 1))

            if not images_path.is_dir():
                self._logger.warning(
                    "Images directory not found for split '%s': %s",
                    split,
                    images_path,
                )
                ok = False
            elif not labels_path.is_dir():
                self._logger.warning(
                    "Labels directory not found for split '%s' "
                    "(expected alongside images): %s",
                    split,
                    labels_path,
                )
                ok = False

        return ok

    # ------------------------------------------------------------------
    # DatasetManager abstract interface
    # ------------------------------------------------------------------

    def prepare(self) -> None:
        """Generate ``data.yaml`` in the dataset root.

        The generated file follows the ultralytics data format::

            path: /data/my_dataset
            train: images/train
            val: images/val
            test: images/test
            nc: 2
            names: {0: class_a, 1: class_b}

        Raises:
            AiLifecycleError: If :meth:`setup` has not been called or the
                              file cannot be written.
        """
        if self._dataset_root is None:
            raise AiLifecycleError(
                "YoloDatasetManager.prepare() called before setup()."
            )

        names: dict[int, str] = self._config.get("names", {})
        # Ensure integer keys (YAML may load them as ints already).
        names = {int(k): str(v) for k, v in names.items()}

        data_yaml: dict[str, Any] = {
            "path": str(self._dataset_root),
            "train": self._config.get("train", "images/train"),
            "val": self._config.get("val", "images/val"),
            "test": self._config.get("test", "images/test"),
            "nc": len(names),
            "names": names,
        }

        data_yaml_path = self._dataset_root / "data.yaml"
        try:
            with open(data_yaml_path, "w", encoding="utf-8") as fh:
                yaml.dump(data_yaml, fh, default_flow_style=False, allow_unicode=True)
        except OSError as exc:
            raise AiLifecycleError(
                f"Failed to write data.yaml to {data_yaml_path}", cause=exc
            ) from exc

        self._data_yaml_path = data_yaml_path
        self._logger.info("data.yaml written to %s", data_yaml_path)

    def stats(self) -> dict[str, Any]:
        """Parse YOLO ``.txt`` label files and compute per-class instance counts.

        YOLO label files have one bounding-box per line; the first column is
        the class index.

        Returns:
            A dict with:

            - ``total_instances`` — total annotation count across all splits.
            - ``per_class`` — ``{class_index: count}`` mapping.
            - ``per_split`` — ``{split: {class_index: count}}``.

        Raises:
            AiLifecycleError: If :meth:`setup` has not been called.
        """
        if self._dataset_root is None:
            raise AiLifecycleError(
                "YoloDatasetManager.stats() called before setup()."
            )

        per_class: dict[int, int] = defaultdict(int)
        per_split: dict[str, dict[int, int]] = {}
        total = 0

        for split in ("train", "val", "test"):
            split_rel: str = self._config.get(split, "")
            images_dir = self._dataset_root / split_rel
            labels_dir = Path(str(images_dir).replace("images", "labels", 1))

            split_counts: dict[int, int] = defaultdict(int)
            if labels_dir.is_dir():
                for label_file in labels_dir.glob("*.txt"):
                    try:
                        lines = label_file.read_text(encoding="utf-8").splitlines()
                        for line in lines:
                            parts = line.strip().split()
                            if parts:
                                cls_idx = int(parts[0])
                                split_counts[cls_idx] += 1
                                per_class[cls_idx] += 1
                                total += 1
                    except (ValueError, OSError) as exc:
                        self._logger.warning(
                            "Skipping malformed label file %s: %s", label_file, exc
                        )

            per_split[split] = dict(split_counts)

        return {
            "total_instances": total,
            "per_class": dict(per_class),
            "per_split": per_split,
        }

    def get_spec(self) -> dict[str, Any]:
        """Return the ultralytics-compatible dataset specification.

        Calls :meth:`prepare` automatically if ``data.yaml`` has not been
        generated yet.

        Returns:
            ``{"data_yaml": "/abs/path/to/data.yaml"}``

        Raises:
            AiLifecycleError: If preparation fails.
        """
        if self._data_yaml_path is None:
            self.prepare()
        return {"data_yaml": str(self._data_yaml_path)}
