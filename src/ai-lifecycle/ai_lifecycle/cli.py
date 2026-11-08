"""Click-based CLI for ai-lifecycle.

Entry point: ``ailc`` (configured in pyproject.toml).

Commands
--------
start     Build Docker image and provision the environment.
stop      Stop and remove the running container.
status    Show container and environment status.
train     Run the training pipeline (starts container first if needed).
evaluate  Run the evaluation pipeline.
infer     Run the inference pipeline.
tui       Launch the interactive TUI.

All commands accept ``-c / --config`` and ``--set key=value`` overrides.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Optional

import click
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

console = Console()


# ---------------------------------------------------------------------------
# Shared options / helpers
# ---------------------------------------------------------------------------


def _config_option() -> click.Option:
    return click.option(
        "-c",
        "--config",
        "config_path",
        type=click.Path(exists=False),
        default=None,
        show_default=True,
        help="Path to experiment config YAML.",
    )


def _set_option() -> click.Option:
    return click.option(
        "--set",
        "overrides",
        multiple=True,
        metavar="KEY=VALUE",
        help="Dot-notation config overrides, e.g. --set training.epochs=50.",
    )


def _parse_overrides(overrides: tuple[str, ...]) -> dict[str, str]:
    """Parse ``KEY=VALUE`` tuples into a dict."""
    result: dict[str, str] = {}
    for item in overrides:
        if "=" not in item:
            console.print(
                f"[yellow]Warning:[/] Ignoring malformed override (no '='): {item!r}"
            )
            continue
        k, _, v = item.partition("=")
        result[k.strip()] = v.strip()
    return result


def _load_cfg(config_path: str | None, overrides: tuple[str, ...]) -> object:
    """Load and return an AiLifecycleConfig, printing validation errors nicely."""
    from ai_lifecycle.core.config import load_config
    from pydantic import ValidationError

    path = config_path or _find_default_config()
    if path is None:
        console.print(
            "[red]Error:[/] No config specified and no default found in configs/. "
            "Use -c to specify a config file."
        )
        sys.exit(1)

    parsed_overrides = _parse_overrides(overrides)
    try:
        cfg = load_config(path, parsed_overrides)
    except FileNotFoundError as exc:
        console.print(f"[red]Error:[/] {exc}")
        sys.exit(1)
    except ValidationError as exc:
        console.print(f"[red]Config validation error:[/]\n{exc}")
        sys.exit(1)

    return cfg


def _find_default_config() -> str | None:
    """Return the first YAML in ``configs/``, or None."""
    configs_dir = Path("configs")
    if configs_dir.is_dir():
        yamls = sorted(configs_dir.glob("*.yaml"))
        if yamls:
            return str(yamls[0])
    return None


def _build_pipeline(cfg: object) -> object:
    """Construct and return a LifecyclePipeline from *cfg*."""
    from ai_lifecycle.core.base import AiLifecycleError
    from ai_lifecycle.core.config import AiLifecycleConfig
    from ai_lifecycle.core.logging import LogManager
    from ai_lifecycle.core.pipeline import LifecyclePipeline
    from ai_lifecycle.core.registry import get_dataset_class, get_model_class
    from ai_lifecycle.core.runs import RunManager
    from ai_lifecycle.datasets import YoloDatasetManager
    from ai_lifecycle.dependencies.uv_docker import UvDockerDependencyManager
    from ai_lifecycle.models.yolov11 import YoloV11ModelManager
    from ai_lifecycle.reporting.reporter import PostTrainingReporter

    assert isinstance(cfg, AiLifecycleConfig)

    dep_mgr = UvDockerDependencyManager(config=cfg.dependencies.model_dump())

    # Dataset manager — require dataset for all stages; fail early with a clear message.
    if cfg.dataset is not None:
        ds_cls = get_dataset_class(cfg.dataset.type)
        ds_mgr = ds_cls(config=cfg.dataset.model_dump())
    else:
        raise AiLifecycleError(
            "No 'dataset' section found in config. "
            "Add a dataset block with type, path, train/val/test splits and class names."
        )

    # Model manager
    model_cls = get_model_class(cfg.model.type)
    model_mgr = model_cls(
        config=cfg.model.model_dump(),
        training_config=cfg.training.model_dump(),
        evaluation_config=cfg.evaluation.model_dump(),
        logging_config=cfg.logging.model_dump(),
    )

    # RunManager is set up first so the real timestamped run_dir is available.
    run_mgr = RunManager(
        config=cfg.outputs.model_dump(),
        full_config=cfg,
    )
    run_mgr.setup()
    actual_run_dir = str(run_mgr.run_dir)

    log_mgr = LogManager(
        config=cfg.logging.model_dump(),
        run_dir=actual_run_dir,
    )

    reporter = PostTrainingReporter(
        run_dir=actual_run_dir,
        config=cfg,
    )

    return LifecyclePipeline(
        config=cfg,
        dependency_manager=dep_mgr,
        dataset_manager=ds_mgr,
        model_manager=model_mgr,
        run_manager=run_mgr,
        log_manager=log_mgr,
        reporter=reporter,
    )


# ---------------------------------------------------------------------------
# Docker helpers
# ---------------------------------------------------------------------------


def _compose_cmd(cfg: object, *args: str) -> list[str]:
    """Build a docker-compose command list.

    The compose file path is resolved relative to this file so the CLI works
    regardless of the process working directory.
    """
    compose_file = Path(__file__).parent.parent / "docker" / "docker-compose.yml"
    return [
        "docker",
        "compose",
        "-f",
        str(compose_file),
        *args,
    ]


def _container_running(cfg: object) -> bool:
    """Return True if the ailc container is running."""
    from ai_lifecycle.core.config import AiLifecycleConfig
    assert isinstance(cfg, AiLifecycleConfig)
    name = f"ailc-{cfg.run_name}"
    result = subprocess.run(
        ["docker", "inspect", "--format", "{{.State.Running}}", name],
        capture_output=True,
        text=True,
    )
    return result.returncode == 0 and result.stdout.strip() == "true"


# ---------------------------------------------------------------------------
# CLI group
# ---------------------------------------------------------------------------


@click.group()
def main() -> None:
    """ai-lifecycle — manage the full ML model lifecycle."""


# ---------------------------------------------------------------------------
# start
# ---------------------------------------------------------------------------


@main.command()
@_config_option()
@_set_option()
def start(config_path: str | None, overrides: tuple[str, ...]) -> None:
    """Build Docker image and provision the Python environment."""
    cfg = _load_cfg(config_path, overrides)

    console.print(Panel("[bold green]Starting ai-lifecycle environment[/]", expand=False))

    console.print("[cyan]Building Docker image …[/]")
    result = subprocess.run(
        _compose_cmd(cfg, "build"),
        check=False,
    )
    if result.returncode != 0:
        console.print("[red]Docker build failed.[/]")
        sys.exit(result.returncode)

    console.print("[cyan]Starting container …[/]")
    result = subprocess.run(
        _compose_cmd(cfg, "up", "-d"),
        check=False,
    )
    if result.returncode != 0:
        console.print("[red]docker compose up failed.[/]")
        sys.exit(result.returncode)

    console.print("[green]✓ Container started.[/]")


# ---------------------------------------------------------------------------
# stop
# ---------------------------------------------------------------------------


@main.command()
@_config_option()
@_set_option()
def stop(config_path: str | None, overrides: tuple[str, ...]) -> None:
    """Stop and remove the running container."""
    cfg = _load_cfg(config_path, overrides)
    console.print("[cyan]Stopping container …[/]")
    subprocess.run(_compose_cmd(cfg, "down"), check=False)
    console.print("[green]✓ Container stopped.[/]")


# ---------------------------------------------------------------------------
# status
# ---------------------------------------------------------------------------


@main.command()
@_config_option()
@_set_option()
def status(config_path: str | None, overrides: tuple[str, ...]) -> None:
    """Show container and environment status."""
    cfg = _load_cfg(config_path, overrides)

    table = Table(title="ai-lifecycle Status", show_header=True, header_style="bold magenta")
    table.add_column("Component", style="cyan")
    table.add_column("Status", style="green")

    running = _container_running(cfg)
    table.add_row("Docker container", "✓ running" if running else "✗ stopped")
    console.print(table)


# ---------------------------------------------------------------------------
# train
# ---------------------------------------------------------------------------


@main.command()
@_config_option()
@_set_option()
def train(config_path: str | None, overrides: tuple[str, ...]) -> None:
    """Run the training pipeline."""
    cfg = _load_cfg(config_path, overrides)
    console.print(Panel("[bold green]Training[/]", expand=False))

    pipeline = _build_pipeline(cfg)
    try:
        pipeline.run("train")
    except Exception as exc:
        console.print(f"[red]Training failed:[/] {exc}")
        sys.exit(1)

    console.print("[green]✓ Training complete.[/]")


# ---------------------------------------------------------------------------
# evaluate
# ---------------------------------------------------------------------------


@main.command()
@_config_option()
@_set_option()
def evaluate(config_path: str | None, overrides: tuple[str, ...]) -> None:
    """Run the evaluation pipeline."""
    cfg = _load_cfg(config_path, overrides)
    console.print(Panel("[bold blue]Evaluation[/]", expand=False))

    pipeline = _build_pipeline(cfg)
    try:
        pipeline.run("evaluate")
    except Exception as exc:
        console.print(f"[red]Evaluation failed:[/] {exc}")
        sys.exit(1)

    console.print("[green]✓ Evaluation complete.[/]")


# ---------------------------------------------------------------------------
# infer
# ---------------------------------------------------------------------------


@main.command()
@_config_option()
@_set_option()
@click.option("--source", default=None, help="Inference source path / URL / webcam index.")
def infer(config_path: str | None, overrides: tuple[str, ...], source: str | None) -> None:
    """Run the inference pipeline."""
    # Allow --source to override config by injecting it as a dot-notation override.
    if source:
        overrides = overrides + (f"inference.source={source}",)

    cfg = _load_cfg(config_path, overrides)
    console.print(Panel("[bold yellow]Inference[/]", expand=False))

    pipeline = _build_pipeline(cfg)
    try:
        pipeline.run("infer")
    except Exception as exc:
        console.print(f"[red]Inference failed:[/] {exc}")
        sys.exit(1)

    console.print("[green]✓ Inference complete.[/]")


# ---------------------------------------------------------------------------
# tui
# ---------------------------------------------------------------------------


@main.command()
def tui() -> None:
    """Launch the interactive terminal UI."""
    from ai_lifecycle.tui import launch_tui

    launch_tui()


if __name__ == "__main__":
    main()
