"""Abstract base class for model managers.

A :class:`ModelManager` owns the full model lifecycle:
build → load checkpoint → configure loss/optimizer → train → evaluate → predict → save.

Callback hook lists (``on_train_start``, ``on_epoch_end``, ``on_train_end``)
allow external observers (e.g. the pipeline, the logger, W&B) to subscribe
without coupling to this class.
"""

from __future__ import annotations

import logging
from abc import abstractmethod
from typing import Any, Callable

from ai_lifecycle.core.base import BaseManager


class ModelManager(BaseManager):
    """Abstract manager for the model lifecycle.

    Extends :class:`~ai_lifecycle.core.base.BaseManager` with model-specific
    abstract methods and observable callback hooks.

    Callback hooks
    --------------
    ``on_train_start : list[Callable[[], None]]``
        Fired once before training begins.
    ``on_epoch_end : list[Callable[[int, dict], None]]``
        Fired after each epoch with ``(epoch_index, metrics_dict)``.
    ``on_train_end : list[Callable[[dict], None]]``
        Fired after training completes with the final metrics dict.

    Args:
        config: The ``model`` section (and any other relevant sections) of
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

        # Callback hook lists — populated by LifecyclePipeline.
        self.on_train_start: list[Callable[[], None]] = []
        self.on_epoch_end: list[Callable[[int, dict[str, Any]], None]] = []
        self.on_train_end: list[Callable[[dict[str, Any]], None]] = []

    # ------------------------------------------------------------------
    # Hook firing helpers (not abstract — concrete classes call these)
    # ------------------------------------------------------------------

    def _fire_train_start(self) -> None:
        """Invoke all ``on_train_start`` callbacks."""
        for cb in self.on_train_start:
            try:
                cb()
            except Exception as exc:
                self._logger.warning("on_train_start callback raised: %s", exc)

    def _fire_epoch_end(self, epoch: int, metrics: dict[str, Any]) -> None:
        """Invoke all ``on_epoch_end`` callbacks.

        Args:
            epoch: 1-based epoch index.
            metrics: Metrics dict for this epoch.
        """
        for cb in self.on_epoch_end:
            try:
                cb(epoch, metrics)
            except Exception as exc:
                self._logger.warning("on_epoch_end callback raised: %s", exc)

    def _fire_train_end(self, metrics: dict[str, Any]) -> None:
        """Invoke all ``on_train_end`` callbacks.

        Args:
            metrics: Final training metrics.
        """
        for cb in self.on_train_end:
            try:
                cb(metrics)
            except Exception as exc:
                self._logger.warning("on_train_end callback raised: %s", exc)

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def build(self) -> None:
        """Construct the model architecture.

        Called once after :meth:`setup`.  For pretrained models this may
        also download weights.

        Raises:
            AiLifecycleError: On build failure.
        """

    @abstractmethod
    def load_checkpoint(self, path: str) -> None:
        """Load model weights from *path*.

        Args:
            path: Absolute or relative path to the checkpoint file.

        Raises:
            AiLifecycleError: If the file is missing or incompatible.
        """

    @abstractmethod
    def configure_loss(self, **kwargs: Any) -> None:
        """Customise loss function hyper-parameters.

        Common kwargs: ``box``, ``cls``, ``dfl`` (YOLO loss weights).

        Args:
            **kwargs: Loss hyper-parameters passed from the config.
        """

    @abstractmethod
    def configure_optimizer(self, **kwargs: Any) -> None:
        """Customise optimizer hyper-parameters.

        Common kwargs: ``name``, ``lr0``, ``weight_decay``.

        Args:
            **kwargs: Optimizer hyper-parameters passed from the config.
        """

    @abstractmethod
    def train(
        self,
        dataset_spec: dict[str, Any],
        run_dir: str,
    ) -> dict[str, Any]:
        """Run model training.

        Args:
            dataset_spec: The spec returned by the dataset manager's
                          :meth:`~ai_lifecycle.datasets.base.DatasetManager.get_spec`.
            run_dir: Absolute path to the run output directory.

        Returns:
            A dict of final training metrics (e.g. ``{"mAP50": 0.82}``).

        Raises:
            AiLifecycleError: On training failure.
        """

    @abstractmethod
    def evaluate(
        self,
        dataset_spec: dict[str, Any],
        run_dir: str,
    ) -> dict[str, Any]:
        """Run model evaluation / validation.

        Args:
            dataset_spec: The spec returned by the dataset manager.
            run_dir: Absolute path to the run output directory.

        Returns:
            A dict of evaluation metrics.

        Raises:
            AiLifecycleError: On evaluation failure.
        """

    @abstractmethod
    def predict(self, source: str, run_dir: str) -> None:
        """Run model inference on *source*.

        Args:
            source: A file path, directory, URL, webcam index, or video path.
            run_dir: Absolute path to the run output directory where
                     prediction results are saved.

        Raises:
            AiLifecycleError: On inference failure.
        """

    @abstractmethod
    def save(self, path: str) -> None:
        """Save the current model / checkpoint to *path*.

        Args:
            path: Destination file path (e.g. ``"best.pt"``).

        Raises:
            AiLifecycleError: If the model cannot be serialised.
        """
