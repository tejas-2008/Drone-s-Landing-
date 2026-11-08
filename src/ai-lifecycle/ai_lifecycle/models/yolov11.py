"""YOLOv11 model manager.

Registered under the name ``"yolov11"`` so experiment configs can select it
with ``model.type: yolov11``.

This manager wraps the `ultralytics <https://github.com/ultralytics/ultralytics>`_
``YOLO`` class and exposes it through the :class:`~ai_lifecycle.models.base.ModelManager`
interface.  Loss and optimizer hyper-parameters are forwarded directly to
``model.train(...)`` as ultralytics keyword arguments.

W&B integration
---------------
When ``logging.wandb`` is ``true`` in the config the ultralytics W&B callback
is registered before training starts.
"""

from __future__ import annotations

import logging
import shutil
from pathlib import Path
from typing import Any

from ai_lifecycle.core.base import AiLifecycleError
from ai_lifecycle.core.registry import register_model
from ai_lifecycle.models.base import ModelManager


@register_model("yolov11")
class YoloV11ModelManager(ModelManager):
    """YOLOv11 model lifecycle manager.

    Wraps the ultralytics ``YOLO`` API and exposes all standard
    :class:`~ai_lifecycle.models.base.ModelManager` methods.

    Args:
        config: The ``model`` section of
                :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                serialised to a plain dict.  Expected keys:

                - ``size`` — model variant letter (``n``, ``s``, ``m``, ``l``, ``x``).
                - ``task`` — ultralytics task string (``detect``, ``segment``, etc.).
                - ``weights`` — optional checkpoint path.
                - ``scratch`` — if ``True``, train from random weights.

        training_config: The ``training`` section serialised to a plain dict.
        logging_config: The ``logging`` section serialised to a plain dict
                        (used for W&B gating).
        logger: Optional pre-configured logger.
    """

    def __init__(
        self,
        config: dict[str, Any],
        training_config: dict[str, Any] | None = None,
        evaluation_config: dict[str, Any] | None = None,
        logging_config: dict[str, Any] | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(config, logger)
        self._training_cfg: dict[str, Any] = training_config or {}
        self._evaluation_cfg: dict[str, Any] = evaluation_config or {}
        self._logging_cfg: dict[str, Any] = logging_config or {}

        self._model: Any = None  # ultralytics.YOLO instance
        self._loss_kwargs: dict[str, Any] = {}
        self._opt_kwargs: dict[str, Any] = {}

    # ------------------------------------------------------------------
    # BaseManager interface
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Verify ultralytics is importable; actual model construction in build()."""
        try:
            import ultralytics  # noqa: F401  # type: ignore[import]
        except ImportError as exc:
            raise AiLifecycleError(
                "ultralytics is not installed in this environment. "
                "Ensure it is listed in dependencies.packages and the env is provisioned.",
                cause=exc,
            ) from exc
        self._logger.debug("ultralytics import OK (version: %s)", _ult_version())

    def validate(self) -> bool:
        """Return True if the model object has been built."""
        return self._model is not None

    # ------------------------------------------------------------------
    # ModelManager abstract interface
    # ------------------------------------------------------------------

    def build(self) -> None:
        """Construct the YOLO model.

        Model name is derived from size and task, e.g. ``yolo11s.pt`` for
        detection or ``yolo11s-seg.pt`` for segmentation.

        Raises:
            AiLifecycleError: If the model cannot be instantiated.
        """
        try:
            from ultralytics import YOLO  # type: ignore[import]
        except ImportError as exc:
            raise AiLifecycleError("ultralytics not available.", cause=exc) from exc

        size: str = self._config.get("size", "s")
        task: str = self._config.get("task", "detect")

        if task == "detect":
            model_name = f"yolo11{size}.pt"
        else:
            model_name = f"yolo11{size}-{task}.pt"

        try:
            self._model = YOLO(model_name)
            self._logger.info("Built YOLO model: %s", model_name)
        except Exception as exc:
            raise AiLifecycleError(
                f"Failed to build YOLO model '{model_name}'", cause=exc
            ) from exc

    def load_checkpoint(self, path: str) -> None:
        """Load weights from *path* into the current model.

        Args:
            path: Checkpoint file path (``*.pt``).

        Raises:
            AiLifecycleError: If the file does not exist or loading fails.
        """
        from ultralytics import YOLO  # type: ignore[import]

        ckpt = Path(path)
        if not ckpt.exists():
            raise AiLifecycleError(f"Checkpoint not found: {ckpt}")

        try:
            self._model = YOLO(str(ckpt))
            self._logger.info("Loaded checkpoint: %s", ckpt)
        except Exception as exc:
            raise AiLifecycleError(
                f"Failed to load checkpoint '{ckpt}'", cause=exc
            ) from exc

    def configure_loss(self, **kwargs: Any) -> None:
        """Store loss hyper-parameters for forwarding to model.train().

        Args:
            **kwargs: Typically ``box``, ``cls``, ``dfl``.
        """
        self._loss_kwargs.update(kwargs)
        self._logger.debug("Loss config: %s", self._loss_kwargs)

    def configure_optimizer(self, **kwargs: Any) -> None:
        """Store optimizer hyper-parameters for forwarding to model.train().

        Args:
            **kwargs: Typically ``name``, ``lr0``, ``weight_decay``.
        """
        self._opt_kwargs.update(kwargs)
        self._logger.debug("Optimizer config: %s", self._opt_kwargs)

    def train(
        self,
        dataset_spec: dict[str, Any],
        run_dir: str,
    ) -> dict[str, Any]:
        """Train the YOLO model.

        Fires ``on_train_start`` and ``on_train_end`` callbacks; ultralytics
        handles per-epoch progress internally.

        Args:
            dataset_spec: Must contain ``data_yaml`` key.
            run_dir: Absolute path to run output directory.

        Returns:
            A flat dict of training metrics extracted from ultralytics results.

        Raises:
            AiLifecycleError: If training fails.
        """
        if self._model is None:
            raise AiLifecycleError("Model not built. Call build() first.")

        data_yaml = dataset_spec.get("data_yaml", "")
        if not data_yaml:
            raise AiLifecycleError("dataset_spec must contain 'data_yaml'.")

        self._register_wandb_callback()
        self._fire_train_start()

        train_kwargs: dict[str, Any] = {
            "data": data_yaml,
            "epochs": self._training_cfg.get("epochs", 100),
            "imgsz": self._training_cfg.get("imgsz", 640),
            "batch": self._training_cfg.get("batch", 16),
            "device": self._training_cfg.get("device", "0"),
            "project": str(Path(run_dir).parent),
            "name": Path(run_dir).name,
            "exist_ok": True,
        }

        freeze = self._training_cfg.get("freeze")
        if freeze is not None:
            train_kwargs["freeze"] = freeze

        optimizer_name = self._opt_kwargs.get("name", "AdamW")
        train_kwargs["optimizer"] = optimizer_name
        train_kwargs["lr0"] = self._opt_kwargs.get("lr0", 0.001)
        train_kwargs["weight_decay"] = self._opt_kwargs.get("weight_decay", 0.0005)

        # YOLO loss weights (box_loss_gain, cls_loss_gain, dfl_loss_gain)
        if "box" in self._loss_kwargs:
            train_kwargs["box"] = self._loss_kwargs["box"]
        if "cls" in self._loss_kwargs:
            train_kwargs["cls"] = self._loss_kwargs["cls"]
        if "dfl" in self._loss_kwargs:
            train_kwargs["dfl"] = self._loss_kwargs["dfl"]

        if self._logging_cfg.get("tensorboard", False):
            train_kwargs["plots"] = True

        try:
            results = self._model.train(**train_kwargs)
            metrics = self._extract_train_metrics(results)
        except Exception as exc:
            raise AiLifecycleError("Training failed", cause=exc) from exc

        self._fire_train_end(metrics)
        return metrics

    def evaluate(
        self,
        dataset_spec: dict[str, Any],
        run_dir: str,
    ) -> dict[str, Any]:
        """Evaluate the model on the validation split.

        Args:
            dataset_spec: Must contain ``data_yaml`` key.
            run_dir: Absolute path to run output directory.

        Returns:
            A flat dict of evaluation metrics.

        Raises:
            AiLifecycleError: If evaluation fails.
        """
        if self._model is None:
            raise AiLifecycleError("Model not built. Call build() first.")

        data_yaml = dataset_spec.get("data_yaml", "")
        conf: float = self._evaluation_cfg.get("conf", 0.001)
        iou: float = self._evaluation_cfg.get("iou", 0.6)
        split: str = self._evaluation_cfg.get("split", "val")

        try:
            results = self._model.val(
                data=data_yaml,
                conf=conf,
                iou=iou,
                split=split,
                project=str(Path(run_dir).parent),
                name=Path(run_dir).name + "_eval",
                exist_ok=True,
            )
            metrics = self._extract_eval_metrics(results)
        except Exception as exc:
            raise AiLifecycleError("Evaluation failed", cause=exc) from exc

        self._logger.info("Evaluation metrics: %s", metrics)
        return metrics

    def predict(self, source: str, run_dir: str) -> None:
        """Run inference on *source* and save results under *run_dir*.

        Args:
            source: File path, directory, URL, webcam index, or video path.
            run_dir: Output directory where prediction images are saved.

        Raises:
            AiLifecycleError: If inference fails.
        """
        if self._model is None:
            raise AiLifecycleError("Model not built. Call build() first.")

        save_dir = Path(run_dir) / "predictions"
        save_dir.mkdir(parents=True, exist_ok=True)

        try:
            self._model.predict(
                source=source,
                project=str(save_dir.parent),
                name=save_dir.name,
                exist_ok=True,
                save=True,
            )
            self._logger.info("Predictions saved to %s", save_dir)
        except Exception as exc:
            raise AiLifecycleError("Inference failed", cause=exc) from exc

    def save(self, path: str) -> None:
        """Export the model to *path*.

        Args:
            path: Destination file (``*.pt``).

        Raises:
            AiLifecycleError: If the model has not been built or save fails.
        """
        if self._model is None:
            raise AiLifecycleError("Model not built. Nothing to save.")

        dest = Path(path)
        dest.parent.mkdir(parents=True, exist_ok=True)

        try:
            # ultralytics stores the best checkpoint at model.trainer.best
            if hasattr(self._model, "trainer") and self._model.trainer is not None:
                best = Path(self._model.trainer.best)
                if best.exists():
                    shutil.copy2(best, dest)
                    self._logger.info("Model saved: %s → %s", best, dest)
                    return
            # Fallback: export to ONNX or similar — here we just warn.
            self._logger.warning(
                "Trainer not available; cannot determine best checkpoint path. "
                "Pass an explicit checkpoint path to load_checkpoint() to save."
            )
        except Exception as exc:
            raise AiLifecycleError(
                f"Failed to save model to '{path}'", cause=exc
            ) from exc

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _register_wandb_callback(self) -> None:
        """Register the ultralytics W&B callback if W&B is configured."""
        if not self._logging_cfg.get("wandb", False):
            return
        try:
            from ultralytics.utils.callbacks import wb as wb_callbacks  # type: ignore[import]
            if hasattr(self._model, "add_callback"):
                for event, fn in wb_callbacks.callbacks.items():
                    self._model.add_callback(event, fn)
                self._logger.info("W&B ultralytics callbacks registered.")
        except ImportError:
            self._logger.warning(
                "ultralytics W&B callbacks not available. "
                "Ensure 'wandb' is installed."
            )
        except Exception as exc:
            self._logger.warning("Failed to register W&B callbacks: %s", exc)

    @staticmethod
    def _extract_train_metrics(results: Any) -> dict[str, Any]:
        """Convert ultralytics training results to a flat dict.

        Args:
            results: The object returned by ``model.train()``.

        Returns:
            A flat metrics dict (best-effort extraction).
        """
        if results is None:
            return {}
        metrics: dict[str, Any] = {}
        # ultralytics Results object attributes vary by version.
        for attr in ("results_dict", "maps", "box"):
            val = getattr(results, attr, None)
            if val is not None:
                if isinstance(val, dict):
                    metrics.update(val)
                else:
                    metrics[attr] = val
        return metrics

    @staticmethod
    def _extract_eval_metrics(results: Any) -> dict[str, Any]:
        """Convert ultralytics validation results to a flat dict.

        Args:
            results: The object returned by ``model.val()``.

        Returns:
            A flat metrics dict.
        """
        if results is None:
            return {}
        metrics: dict[str, Any] = {}
        for attr in ("results_dict", "maps", "map", "map50", "map75"):
            val = getattr(results, attr, None)
            if val is not None:
                if isinstance(val, dict):
                    metrics.update(val)
                else:
                    metrics[attr] = float(val) if hasattr(val, "__float__") else val
        # per-class AP
        if hasattr(results, "ap_class_index") and hasattr(results, "ap"):
            per_class = {}
            for i, cls_idx in enumerate(results.ap_class_index):
                per_class[int(cls_idx)] = float(results.ap[i])
            metrics["per_class_ap"] = per_class
        return metrics


def _ult_version() -> str:
    """Return the installed ultralytics version string, or 'unknown'."""
    try:
        import ultralytics  # type: ignore[import]
        return getattr(ultralytics, "__version__", "unknown")
    except ImportError:
        return "not installed"
