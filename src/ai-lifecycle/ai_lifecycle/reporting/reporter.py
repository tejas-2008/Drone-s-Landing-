"""Post-training reporter: plots, metrics, and summary markdown.

:class:`PostTrainingReporter` is intentionally **not** a
:class:`~ai_lifecycle.core.base.BaseManager` subclass — it is a pure utility
class that only writes artefacts; it never manages resources or lifecycle state.

All plots are saved to ``<run_dir>/plots/`` and all structured metrics to
``<run_dir>/metrics/``.

Usage::

    reporter = PostTrainingReporter(run_dir="/workspace/outputs/proj/exp_001", config=cfg)
    reporter.run(train_metrics=train_results, eval_metrics=eval_results)
"""

from __future__ import annotations

import json
import logging
import shutil
import textwrap
from datetime import datetime
from pathlib import Path
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from ai_lifecycle.core.config import AiLifecycleConfig

logger = logging.getLogger("ailc.reporter")


class PostTrainingReporter:
    """Generates post-training artefacts for a single experiment run.

    Args:
        run_dir: Absolute path to the run output directory created by
                 :class:`~ai_lifecycle.core.runs.RunManager`.
        config: The validated :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                instance for the experiment.
    """

    def __init__(self, run_dir: str, config: "AiLifecycleConfig") -> None:
        self._run_dir = Path(run_dir)
        self._config = config
        self._plots_dir = self._run_dir / "plots"
        self._metrics_dir = self._run_dir / "metrics"

        self._plots_dir.mkdir(parents=True, exist_ok=True)
        self._metrics_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(
        self,
        train_metrics: dict[str, Any],
        eval_metrics: dict[str, Any],
    ) -> None:
        """Execute the full reporting pipeline.

        Steps (non-fatal on individual failure):

        1. Plot training curves from ``metrics/metrics.csv`` if it exists.
        2. Copy confusion matrix if ultralytics generated one.
        3. Write ``eval_metrics.json``.
        4. Write ``per_class_ap.json`` (extracted from *eval_metrics*).
        5. Write ``summary.md``.

        Args:
            train_metrics: Dict returned by the model manager's ``train()``.
            eval_metrics: Dict returned by the model manager's ``evaluate()``.
        """
        self._safe_call(
            self.plot_training_curves,
            str(self._metrics_dir / "metrics.csv"),
            label="plot_training_curves",
        )
        self._safe_call(
            self._copy_confusion_matrix,
            label="copy_confusion_matrix",
        )
        self._safe_call(
            self.write_eval_results,
            eval_metrics,
            label="write_eval_results",
        )
        per_class = eval_metrics.get("per_class_ap", {})
        if per_class:
            self._safe_call(
                self.write_per_class_ap,
                per_class,
                label="write_per_class_ap",
            )
        self._safe_call(
            self.write_summary,
            train_metrics,
            eval_metrics,
            label="write_summary",
        )

    def plot_training_curves(self, metrics_csv: str) -> None:
        """Plot loss, mAP and learning-rate curves from *metrics_csv*.

        Ultralytics saves ``results.csv`` during training; this method reads
        that file and produces a ``training_curves.png`` in the plots dir.

        Args:
            metrics_csv: Path to the CSV file containing per-epoch metrics.
                         Silently skipped if the file does not exist.
        """
        csv_path = Path(metrics_csv)
        if not csv_path.exists():
            # Try to find ultralytics results.csv in run dir
            candidate = self._run_dir / "results.csv"
            if candidate.exists():
                csv_path = candidate
            else:
                logger.info("No metrics CSV found — skipping training curves plot.")
                return

        try:
            import matplotlib  # type: ignore[import]
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt  # type: ignore[import]
            import pandas as pd  # type: ignore[import]
        except ImportError:
            logger.warning("matplotlib/pandas not installed — skipping plots.")
            return

        try:
            df = pd.read_csv(csv_path)
            df.columns = df.columns.str.strip()
        except Exception as exc:
            logger.warning("Failed to read metrics CSV: %s", exc)
            return

        # Detect common ultralytics column groups
        loss_cols = [c for c in df.columns if "loss" in c.lower()]
        map_cols = [c for c in df.columns if "map" in c.lower()]
        lr_cols = [c for c in df.columns if "lr" in c.lower()]

        fig, axes = plt.subplots(1, 3, figsize=(18, 5))
        fig.suptitle(
            f"{self._config.project} / {self._config.run_name} — Training Curves",
            fontsize=13,
        )

        x = df.index + 1  # epoch numbers (1-based)

        for ax, cols, title in zip(
            axes,
            [loss_cols, map_cols, lr_cols],
            ["Loss", "mAP", "Learning Rate"],
        ):
            if cols:
                for col in cols:
                    ax.plot(x, df[col], label=col)
                ax.legend(fontsize=8)
            else:
                ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)
            ax.set_title(title)
            ax.set_xlabel("Epoch")
            ax.grid(True, alpha=0.3)

        out_path = self._plots_dir / "training_curves.png"
        fig.tight_layout()
        fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        logger.info("Training curves saved: %s", out_path)

    def plot_confusion_matrix(self, matrix_path: str) -> None:
        """Copy or render a confusion matrix image to the plots directory.

        Args:
            matrix_path: Path to an existing confusion matrix image or numpy
                         array file (``.npy``).  If it does not exist the
                         method returns silently.
        """
        src = Path(matrix_path)
        if not src.exists():
            logger.info("Confusion matrix not found at %s — skipping.", matrix_path)
            return

        dest = self._plots_dir / src.name
        try:
            shutil.copy2(src, dest)
            logger.info("Confusion matrix copied: %s → %s", src, dest)
        except OSError as exc:
            logger.warning("Failed to copy confusion matrix: %s", exc)

    def plot_pr_curve(self, pr_data: dict[str, Any]) -> None:
        """Plot a Precision-Recall curve from *pr_data*.

        Args:
            pr_data: Dict with keys ``"precision"`` and ``"recall"`` (both
                     lists of floats).  Returns silently if either is missing.
        """
        precision = pr_data.get("precision", [])
        recall = pr_data.get("recall", [])
        if not precision or not recall:
            logger.info("PR data empty — skipping PR curve plot.")
            return

        try:
            import matplotlib  # type: ignore[import]
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt  # type: ignore[import]
        except ImportError:
            logger.warning("matplotlib not installed — skipping PR curve plot.")
            return

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(recall, precision, lw=2)
        ax.set_xlabel("Recall")
        ax.set_ylabel("Precision")
        ax.set_title("Precision-Recall Curve")
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.grid(True, alpha=0.3)

        out_path = self._plots_dir / "pr_curve.png"
        fig.tight_layout()
        fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        logger.info("PR curve saved: %s", out_path)

    def write_eval_results(self, eval_metrics: dict[str, Any]) -> None:
        """Serialise evaluation metrics to ``<metrics_dir>/eval_metrics.json``.

        Args:
            eval_metrics: Flat metrics dict from the model manager.
        """
        out_path = self._metrics_dir / "eval_metrics.json"
        self._write_json(out_path, eval_metrics)
        logger.info("Eval metrics written: %s", out_path)

    def write_per_class_ap(self, per_class: dict[str, Any]) -> None:
        """Serialise per-class AP to ``<metrics_dir>/per_class_ap.json``.

        Args:
            per_class: Mapping of class index (int or str) to AP value.
        """
        out_path = self._metrics_dir / "per_class_ap.json"
        # JSON requires string keys.
        self._write_json(out_path, {str(k): v for k, v in per_class.items()})
        logger.info("Per-class AP written: %s", out_path)

    def write_summary(
        self,
        train_metrics: dict[str, Any],
        eval_metrics: dict[str, Any],
    ) -> None:
        """Write a Markdown summary of the experiment to ``<run_dir>/summary.md``.

        The summary includes: config snapshot excerpt, best epoch (if
        available), key training and evaluation metrics, and paths to output
        artefacts.

        Args:
            train_metrics: Final training metrics dict.
            eval_metrics: Evaluation metrics dict.
        """
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        map50 = eval_metrics.get("map50", eval_metrics.get("metrics/mAP50(B)", "N/A"))
        map50_95 = eval_metrics.get(
            "map50_95",
            eval_metrics.get("metrics/mAP50-95(B)", "N/A"),
        )
        best_epoch = train_metrics.get("epoch", "N/A")

        cfg = self._config
        lines = [
            f"# Experiment Summary",
            f"",
            f"**Generated:** {ts}  ",
            f"**Project:** `{cfg.project}`  ",
            f"**Run:** `{cfg.run_name}`  ",
            f"**Run directory:** `{self._run_dir}`",
            f"",
            f"---",
            f"",
            f"## Config Snapshot",
            f"",
            f"| Key | Value |",
            f"|-----|-------|",
            f"| Model type | `{cfg.model.type}` |",
            f"| Model size | `{cfg.model.size}` |",
            f"| Task | `{cfg.model.task}` |",
            f"| Epochs | `{cfg.training.epochs}` |",
            f"| Image size | `{cfg.training.imgsz}` |",
            f"| Batch size | `{cfg.training.batch}` |",
            f"| Device | `{cfg.training.device}` |",
            f"| Optimizer | `{cfg.model.optimizer.name}` |",
            f"| LR₀ | `{cfg.model.optimizer.lr0}` |",
            f"",
            f"---",
            f"",
            f"## Training Results",
            f"",
            f"| Metric | Value |",
            f"|--------|-------|",
            f"| Best epoch | `{best_epoch}` |",
        ]
        for k, v in train_metrics.items():
            if k != "epoch":
                lines.append(f"| {k} | `{v}` |")

        lines += [
            f"",
            f"---",
            f"",
            f"## Evaluation Results",
            f"",
            f"| Metric | Value |",
            f"|--------|-------|",
            f"| mAP@50 | `{map50}` |",
            f"| mAP@50-95 | `{map50_95}` |",
        ]
        for k, v in eval_metrics.items():
            if k not in ("map50", "map50_95", "per_class_ap"):
                lines.append(f"| {k} | `{v}` |")

        lines += [
            f"",
            f"---",
            f"",
            f"## Output Paths",
            f"",
            f"| Artefact | Path |",
            f"|----------|------|",
            f"| Config snapshot | `config.snapshot.yaml` |",
            f"| Training curves | `plots/training_curves.png` |",
            f"| PR curve | `plots/pr_curve.png` |",
            f"| Eval metrics | `metrics/eval_metrics.json` |",
            f"| Per-class AP | `metrics/per_class_ap.json` |",
            f"| Checkpoints | `checkpoints/` |",
            f"",
        ]

        out_path = self._run_dir / "summary.md"
        out_path.write_text("\n".join(lines), encoding="utf-8")
        logger.info("Summary written: %s", out_path)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _copy_confusion_matrix(self) -> None:
        """Look for a confusion matrix generated by ultralytics and copy it."""
        # ultralytics typically saves confusion_matrix.png in the run dir
        for name in ("confusion_matrix.png", "confusion_matrix_normalized.png"):
            candidate = self._run_dir / name
            if candidate.exists():
                self.plot_confusion_matrix(str(candidate))
                return

    @staticmethod
    def _write_json(path: Path, data: dict[str, Any]) -> None:
        """Write *data* as pretty-printed JSON to *path*."""
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2, default=str)

    @staticmethod
    def _safe_call(fn: Any, *args: Any, label: str = "") -> None:
        """Call *fn(*args)* and log any exception without re-raising.

        Args:
            fn: Callable to invoke.
            *args: Positional arguments forwarded to *fn*.
            label: Human-readable label used in the warning message.
        """
        try:
            fn(*args)
        except Exception as exc:
            logger.warning(
                "Reporter step '%s' failed (non-fatal): %s", label, exc, exc_info=True
            )
