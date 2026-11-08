"""LifecyclePipeline — orchestrates the full model lifecycle.

The pipeline receives all managers through its constructor (dependency
injection) so it is never coupled to concrete implementations.  New model
families or dataset formats can be added without changing this class.

Usage::

    pipeline = LifecyclePipeline(
        config=cfg,
        dependency_manager=dep_mgr,
        dataset_manager=ds_mgr,
        model_manager=model_mgr,
        run_manager=run_mgr,
        log_manager=log_mgr,
        reporter=reporter,
    )
    pipeline.run("train")
"""

from __future__ import annotations

import logging
from typing import Any, Callable

from ai_lifecycle.core.base import AiLifecycleError


class LifecyclePipeline:
    """Orchestrates training, evaluation, and inference stages.

    The pipeline uses strict dependency injection — concrete manager
    classes are never instantiated here.  All managers are expected to
    follow the :class:`~ai_lifecycle.core.base.BaseManager` protocol.

    Callback hooks on the ``model_manager`` (``on_train_start``,
    ``on_epoch_end``, ``on_train_end``) are populated during construction so
    that logging and reporting code does not need to call those managers
    directly.

    Args:
        config: The validated :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                instance.
        dependency_manager: Manages the Python/system environment.
        dataset_manager: Handles dataset preparation and spec generation.
        model_manager: Handles the model lifecycle (build / train / eval / infer).
        run_manager: Creates and owns the run output directory.
        log_manager: Configures logging sinks.
        reporter: Generates post-training plots and summaries.
    """

    def __init__(
        self,
        config: Any,  # AiLifecycleConfig
        dependency_manager: Any,
        dataset_manager: Any,
        model_manager: Any,
        run_manager: Any,
        log_manager: Any | None = None,
        reporter: Any | None = None,
    ) -> None:
        self._config = config
        self._dep_mgr = dependency_manager
        self._ds_mgr = dataset_manager
        self._model_mgr = model_manager
        self._run_mgr = run_manager
        self._log_mgr = log_manager
        self._reporter = reporter
        self._logger = logging.getLogger("ailc.pipeline")

        self._train_metrics: dict[str, Any] = {}
        self._eval_metrics: dict[str, Any] = {}

        self._register_callbacks()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, stage: str) -> None:
        """Run a named lifecycle stage.

        Args:
            stage: One of ``"train"``, ``"evaluate"``, or ``"infer"``.

        Raises:
            AiLifecycleError: On any stage failure.
            ValueError: If *stage* is unrecognised.
        """
        stages = {
            "train": self._run_train,
            "evaluate": self._run_evaluate,
            "infer": self._run_infer,
        }
        if stage not in stages:
            raise ValueError(
                f"Unknown stage '{stage}'. Valid stages: {sorted(stages)}"
            )

        self._logger.info("=== Starting stage: %s ===", stage.upper())
        try:
            stages[stage]()
        except AiLifecycleError:
            raise
        except Exception as exc:
            raise AiLifecycleError(
                f"Stage '{stage}' failed unexpectedly", cause=exc
            ) from exc

    # ------------------------------------------------------------------
    # Callback hooks
    # ------------------------------------------------------------------

    def on_train_start(self) -> None:
        """Called immediately before the first training iteration."""
        self._logger.info("on_train_start fired")

    def on_epoch_end(self, epoch: int, metrics: dict[str, Any]) -> None:
        """Called after every training epoch.

        Args:
            epoch: 1-based epoch index.
            metrics: Dict of metric name -> scalar value for this epoch.
        """
        self._logger.debug("Epoch %d metrics: %s", epoch, metrics)
        if self._log_mgr is not None:
            try:
                self._log_mgr.log_metrics({"epoch": epoch, **metrics})
            except Exception as exc:
                self._logger.warning("log_metrics failed: %s", exc)

    def on_train_end(self, metrics: dict[str, Any]) -> None:
        """Called after training completes.

        Args:
            metrics: Final training metrics dict.
        """
        self._logger.info("on_train_end fired — final metrics: %s", metrics)
        self._train_metrics = metrics

    # ------------------------------------------------------------------
    # Stage implementations
    # ------------------------------------------------------------------

    def _run_train(self) -> None:
        """Full train → evaluate → report chain."""
        self._setup_all()

        dataset_spec = self._ds_mgr.get_spec()
        run_dir = str(self._run_mgr.run_dir)

        self._logger.info("Starting training …")
        self._train_metrics = self._model_mgr.train(
            dataset_spec=dataset_spec, run_dir=run_dir
        )

        # Automatically chain evaluate and report after training.
        self._logger.info("Training complete. Running post-training evaluation …")
        self._eval_metrics = self._model_mgr.evaluate(
            dataset_spec=dataset_spec, run_dir=run_dir
        )

        if self._reporter is not None:
            self._logger.info("Generating post-training report …")
            try:
                self._reporter.run(
                    train_metrics=self._train_metrics,
                    eval_metrics=self._eval_metrics,
                )
            except Exception as exc:
                self._logger.warning(
                    "Reporter failed (non-fatal): %s", exc, exc_info=True
                )

    def _run_evaluate(self) -> None:
        """Standalone evaluation stage."""
        self._setup_all()
        dataset_spec = self._ds_mgr.get_spec()
        run_dir = str(self._run_mgr.run_dir)
        self._eval_metrics = self._model_mgr.evaluate(
            dataset_spec=dataset_spec, run_dir=run_dir
        )
        self._logger.info("Evaluation complete: %s", self._eval_metrics)

    def _run_infer(self) -> None:
        """Standalone inference stage."""
        self._setup_all()
        run_dir = str(self._run_mgr.run_dir)
        source = self._config.inference.source
        if not source:
            raise AiLifecycleError(
                "Inference stage requires 'inference.source' to be set in the config."
            )
        self._model_mgr.predict(source=source, run_dir=run_dir)
        self._logger.info("Inference complete. Results saved to %s", run_dir)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _setup_all(self) -> None:
        """Setup all managers in dependency order.

        Note: RunManager.setup() is intentionally called in _build_pipeline()
        before the pipeline is constructed so the actual timestamped run_dir
        is available to LogManager and Reporter at build time.  Calling
        run_mgr.setup() again here would create a *second* timestamped directory.
        We guard against that with the validate() check.
        """
        if self._log_mgr is not None:
            self._log_mgr.setup()

        # Only call setup() if it hasn't been run yet (run_dir not yet created).
        if not self._run_mgr.validate():
            self._run_mgr.setup()

        if not self._dep_mgr.is_ready():
            self._logger.info("Provisioning environment …")
            self._dep_mgr.setup()
            self._dep_mgr.provision()

        self._ds_mgr.setup()
        if not self._ds_mgr.validate():
            raise AiLifecycleError("Dataset validation failed. Check paths and labels.")

        self._model_mgr.setup()
        self._model_mgr.build()

        weights = self._config.model.weights
        if weights:
            self._model_mgr.load_checkpoint(weights)

        self._model_mgr.configure_loss(
            box=self._config.model.loss.box,
            cls=self._config.model.loss.cls,
            dfl=self._config.model.loss.dfl,
        )
        self._model_mgr.configure_optimizer(
            name=self._config.model.optimizer.name,
            lr0=self._config.model.optimizer.lr0,
            weight_decay=self._config.model.optimizer.weight_decay,
        )

    def _register_callbacks(self) -> None:
        """Subscribe pipeline hooks to model_manager callback lists."""
        if not hasattr(self._model_mgr, "on_train_start"):
            return

        self._model_mgr.on_train_start.append(self.on_train_start)
        self._model_mgr.on_epoch_end.append(self.on_epoch_end)
        self._model_mgr.on_train_end.append(self.on_train_end)
