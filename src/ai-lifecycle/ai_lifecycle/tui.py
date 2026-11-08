"""Interactive terminal UI for ai-lifecycle.

Uses ``rich`` for display and ``questionary`` for prompts.

Flow
----
1. **Main Menu** — Train / Evaluate / Inference / Manage Dependencies / Quit.
2. **Config selection** — list ``.yaml`` files in ``configs/`` or enter custom path.
3. **Process-specific prompts** — collect parameters interactively.
4. **Summary panel** — confirm before running.
5. **Execution** — write temporary overrides and run the pipeline.

The TUI shares the exact same pipeline code path as the CLI — there is only
one execution route.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path
from typing import Any

import yaml
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

console = Console()


# ---------------------------------------------------------------------------
# Questionary import (optional at import time; hard-fail at runtime)
# ---------------------------------------------------------------------------


def _q() -> Any:
    """Return the questionary module, raising a clear error if not installed."""
    try:
        import questionary  # type: ignore[import]
        return questionary
    except ImportError:
        console.print(
            "[red]Error:[/] 'questionary' is required for the TUI. "
            "Install it with: pip install questionary"
        )
        sys.exit(1)


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def launch_tui() -> None:
    """Launch the interactive TUI.  Called by the ``tui`` CLI command."""
    q = _q()

    console.print(
        Panel(
            "[bold cyan]ai-lifecycle[/] — Interactive Terminal UI",
            subtitle="Use arrow keys to navigate",
            expand=False,
        )
    )

    while True:
        action = q.select(
            "What would you like to do?",
            choices=[
                "Train",
                "Evaluate",
                "Inference",
                "Manage Dependencies",
                "Quit",
            ],
        ).ask()

        if action is None or action == "Quit":
            console.print("[dim]Bye![/]")
            break
        elif action == "Train":
            _flow_train(q)
        elif action == "Evaluate":
            _flow_evaluate(q)
        elif action == "Inference":
            _flow_infer(q)
        elif action == "Manage Dependencies":
            _flow_deps(q)


# ---------------------------------------------------------------------------
# Config selection
# ---------------------------------------------------------------------------


def _select_config(q: Any) -> str:
    """Prompt the user to choose or enter a config YAML path."""
    configs_dir = Path("configs")
    yamls: list[str] = []
    if configs_dir.is_dir():
        yamls = sorted(str(p) for p in configs_dir.glob("*.yaml"))

    choices = yamls + ["Enter custom path …"]
    selected = q.select("Select a config:", choices=choices).ask()

    if selected == "Enter custom path …":
        selected = q.text("Config path:").ask()

    return selected or ""


# ---------------------------------------------------------------------------
# Train flow
# ---------------------------------------------------------------------------


def _flow_train(q: Any) -> None:
    """Interactive prompts for the training flow."""
    config_path = _select_config(q)
    if not config_path:
        return

    dataset_path = q.text(
        "Dataset path:", default="/data/my_dataset"
    ).ask()
    model_size = q.select(
        "Model size:", choices=["n", "s", "m", "l", "x"], default="s"
    ).ask()
    task = q.select(
        "Task:", choices=["detect", "segment", "classify"], default="detect"
    ).ask()
    checkpoint = q.text(
        "Resume checkpoint (leave blank for none):", default=""
    ).ask()
    epochs = q.text("Epochs:", default="100").ask()
    batch = q.text("Batch size:", default="16").ask()
    device = q.text("Device (e.g. 0, cpu, 0,1):", default="0").ask()

    params: dict[str, Any] = {
        "dataset.path": dataset_path,
        "model.size": model_size,
        "model.task": task,
        "training.epochs": epochs,
        "training.batch": batch,
        "training.device": device,
    }
    if checkpoint:
        params["model.weights"] = checkpoint

    if not _confirm_and_run(q, "train", config_path, params):
        return

    _execute("train", config_path, params)


# ---------------------------------------------------------------------------
# Evaluate flow
# ---------------------------------------------------------------------------


def _flow_evaluate(q: Any) -> None:
    """Interactive prompts for the evaluation flow."""
    config_path = _select_config(q)
    if not config_path:
        return

    checkpoint = q.text("Checkpoint path:").ask()
    dataset_path = q.text("Dataset path:", default="/data/my_dataset").ask()
    split = q.select("Split:", choices=["val", "test", "train"], default="val").ask()
    conf = q.text("Confidence threshold:", default="0.001").ask()
    iou = q.text("IoU threshold:", default="0.6").ask()

    params: dict[str, Any] = {
        "dataset.path": dataset_path,
        "evaluation.split": split,
        "evaluation.conf": conf,
        "evaluation.iou": iou,
    }
    if checkpoint:
        params["model.weights"] = checkpoint

    if not _confirm_and_run(q, "evaluate", config_path, params):
        return

    _execute("evaluate", config_path, params)


# ---------------------------------------------------------------------------
# Inference flow
# ---------------------------------------------------------------------------


def _flow_infer(q: Any) -> None:
    """Interactive prompts for the inference flow."""
    config_path = _select_config(q)
    if not config_path:
        return

    checkpoint = q.text("Checkpoint path:").ask()
    source = q.text(
        "Source (file / folder / webcam index / video path):", default="0"
    ).ask()
    conf = q.text("Confidence threshold:", default="0.25").ask()
    iou = q.text("IoU threshold:", default="0.7").ask()
    save = q.confirm("Save results?", default=True).ask()

    params: dict[str, Any] = {
        "inference.source": source,
        "inference.conf": conf,
        "inference.iou": iou,
        "inference.save": "true" if save else "false",
    }
    if checkpoint:
        params["model.weights"] = checkpoint

    if not _confirm_and_run(q, "infer", config_path, params):
        return

    _execute("infer", config_path, params)


# ---------------------------------------------------------------------------
# Dependencies flow
# ---------------------------------------------------------------------------


def _flow_deps(q: Any) -> None:
    """Show dependency info and optionally provision the environment."""
    config_path = _select_config(q)
    if not config_path:
        return

    from ai_lifecycle.core.config import load_config

    try:
        cfg = load_config(config_path)
    except Exception as exc:
        console.print(f"[red]Failed to load config:[/] {exc}")
        return

    table = Table(title="Packages to install", show_header=True)
    table.add_column("Package", style="cyan")
    for pkg in cfg.dependencies.packages:
        table.add_row(pkg)
    console.print(table)

    if q.confirm("Provision environment now?", default=False).ask():
        from ai_lifecycle.dependencies.uv_docker import UvDockerDependencyManager

        mgr = UvDockerDependencyManager(config=cfg.dependencies.model_dump())
        mgr.setup()
        if mgr.is_ready():
            console.print("[yellow]Environment already provisioned.[/]")
        else:
            with console.status("Provisioning …"):
                mgr.provision()
            console.print("[green]✓ Environment provisioned.[/]")


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _confirm_and_run(
    q: Any,
    stage: str,
    config_path: str,
    params: dict[str, Any],
) -> bool:
    """Show a summary table and ask for confirmation.

    Args:
        q: questionary module.
        stage: Pipeline stage name.
        config_path: Path to the experiment config.
        params: Dot-notation override dict to display.

    Returns:
        ``True`` if the user confirmed, ``False`` otherwise.
    """
    table = Table(
        title=f"[bold]{stage.upper()}[/] — Parameters",
        show_header=True,
        header_style="bold magenta",
    )
    table.add_column("Parameter", style="cyan")
    table.add_column("Value")
    table.add_row("config", config_path)
    table.add_row("stage", stage)
    for k, v in params.items():
        table.add_row(k, str(v))

    console.print(table)
    return q.confirm("Confirm and run?", default=True).ask() or False


def _execute(stage: str, config_path: str, params: dict[str, Any]) -> None:
    """Load config, apply overrides, and run the pipeline.

    This function uses the *same* pipeline code path as the CLI so there is
    only one execution route.

    Args:
        stage: One of ``"train"``, ``"evaluate"``, or ``"infer"``.
        config_path: Path to the experiment config YAML.
        params: Dot-notation overrides to apply.
    """
    from ai_lifecycle.core.config import load_config
    from pydantic import ValidationError

    str_overrides = {k: str(v) for k, v in params.items()}

    try:
        cfg = load_config(config_path, str_overrides)
    except (FileNotFoundError, ValidationError) as exc:
        console.print(f"[red]Config error:[/] {exc}")
        return

    # Re-use CLI helper to build the pipeline
    from ai_lifecycle.cli import _build_pipeline

    pipeline = _build_pipeline(cfg)
    with console.status(f"Running [bold]{stage}[/] …"):
        try:
            pipeline.run(stage)
        except Exception as exc:
            console.print(f"[red]{stage} failed:[/] {exc}")
            return

    console.print(f"[green]✓ {stage.capitalize()} complete.[/]")
