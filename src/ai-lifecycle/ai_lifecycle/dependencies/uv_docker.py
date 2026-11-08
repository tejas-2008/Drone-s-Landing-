"""uv-based dependency manager for Docker environments.

:class:`UvDockerDependencyManager` provisions a hashed virtual environment
under ``/envs/<sha256>`` using `uv <https://github.com/astral-sh/uv>`_.

The hash is derived from the *sorted* package specifier list, so changing any
package triggers a fresh environment while repeated starts reuse the existing
one — making restarts near-instant.

Usage::

    mgr = UvDockerDependencyManager(
        config=cfg.dependencies.model_dump(),
    )
    mgr.setup()
    if not mgr.is_ready():
        mgr.provision()
    mgr.run_in_env(["python", "-c", "import ultralytics"])
"""

from __future__ import annotations

import hashlib
import logging
import subprocess
from pathlib import Path
from typing import Any

from ai_lifecycle.core.base import AiLifecycleError
from ai_lifecycle.dependencies.base import DependencyManager


class UvDockerDependencyManager(DependencyManager):
    """Provisions a hashed ``uv`` virtual environment inside a Docker container.

    Environment path: ``<envs_root>/<sha256_of_sorted_packages>/``

    Args:
        config: The ``dependencies`` section of
                :class:`~ai_lifecycle.core.config.AiLifecycleConfig`
                serialised to a plain dict.  Expected keys:

                - ``packages`` (list[str]) — PEP 508 specifiers.
                - ``extra_index_url`` (str | None) — optional extra PyPI index.
                - ``python`` (str) — Python version hint, e.g. ``"3.11"``.

        envs_root: Root directory where hashed envs are stored.
                   Defaults to ``/envs``.
        logger: Optional pre-configured logger.
    """

    def __init__(
        self,
        config: dict[str, Any],
        envs_root: str = "/envs",
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__(config, logger)
        self._envs_root = Path(envs_root)
        self._env_path: Path | None = None

    # ------------------------------------------------------------------
    # BaseManager interface
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Compute the hashed env path; create the envs root if needed."""
        packages = self.resolve()
        env_hash = self._hash_packages(packages)
        self._env_path = self._envs_root / env_hash
        self._envs_root.mkdir(parents=True, exist_ok=True)
        self._logger.debug(
            "Dependency hash: %s → env at %s", env_hash, self._env_path
        )

    def validate(self) -> bool:
        """Return True if the hashed env directory exists and has a Python binary."""
        if self._env_path is None:
            return False
        python_bin = self._env_path / "bin" / "python"
        return python_bin.is_file()

    # ------------------------------------------------------------------
    # DependencyManager interface
    # ------------------------------------------------------------------

    def resolve(self) -> list[str]:
        """Return the sorted package list from config.

        Sorting ensures the hash is deterministic regardless of the order
        the packages are listed in the config.

        Returns:
            Sorted list of PEP 508 package specifiers.
        """
        packages: list[str] = self._config.get("packages", [])
        return sorted(packages)

    def provision(self) -> None:
        """Create the hashed venv and install all resolved packages.

        Skips provisioning if the environment already exists (fast reuse).

        Raises:
            AiLifecycleError: If ``uv venv`` or ``uv pip install`` fails.
        """
        if self._env_path is None:
            self.setup()

        if self.is_ready():
            self._logger.info(
                "Environment already provisioned at %s. Skipping.", self._env_path
            )
            return

        packages = self.resolve()
        python_version = self._config.get("python", "3.11")

        self._logger.info(
            "Provisioning new environment at %s (python=%s, %d packages) …",
            self._env_path,
            python_version,
            len(packages),
        )

        # Create venv
        venv_cmd = [
            "uv",
            "venv",
            "--python",
            python_version,
            str(self._env_path),
        ]
        self._run_subprocess(venv_cmd, "uv venv")

        # Install packages
        if packages:
            install_cmd = [
                "uv",
                "pip",
                "install",
                "--python",
                str(self._env_path / "bin" / "python"),
            ]
            extra_index = self._config.get("extra_index_url")
            if extra_index:
                install_cmd += ["--extra-index-url", extra_index]
            install_cmd.extend(packages)
            self._run_subprocess(install_cmd, "uv pip install")

        self._logger.info("Environment provisioned successfully at %s", self._env_path)

    def is_ready(self) -> bool:
        """Return True if the hashed venv Python binary exists."""
        return self.validate()

    def run_in_env(self, cmd: list[str]) -> int:
        """Run *cmd* using the venv's Python executable as the interpreter.

        The first element of *cmd* should be ``"python"`` — it is replaced
        with the venv's absolute Python binary path.  Any other binary is
        passed through unchanged.

        Args:
            cmd: Command and arguments to execute inside the environment.

        Returns:
            Process exit code.

        Raises:
            AiLifecycleError: If the environment is not ready or the
                              subprocess cannot be launched.
        """
        if not self.is_ready():
            raise AiLifecycleError(
                "Environment is not ready. Call provision() first."
            )

        python_bin = str(self._env_path / "bin" / "python")  # type: ignore[operator]
        resolved_cmd = [python_bin if c == "python" else c for c in cmd]

        self._logger.debug("run_in_env: %s", " ".join(resolved_cmd))
        try:
            result = subprocess.run(resolved_cmd, check=False)
            return result.returncode
        except FileNotFoundError as exc:
            raise AiLifecycleError(
                f"Command not found: {resolved_cmd[0]}", cause=exc
            ) from exc
        except OSError as exc:
            raise AiLifecycleError(
                f"Failed to run command in env: {exc}", cause=exc
            ) from exc

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _hash_packages(packages: list[str]) -> str:
        """Return a 16-character hex digest of the *sorted* package list.

        Args:
            packages: Already-sorted list of PEP 508 specifiers.

        Returns:
            A short hex string suitable for use as a directory name.
        """
        canonical = "\n".join(packages).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()[:16]

    def _run_subprocess(self, cmd: list[str], label: str) -> None:
        """Run *cmd* via subprocess, raising :class:`AiLifecycleError` on failure.

        Args:
            cmd: Command to execute.
            label: Human-readable label for error messages.

        Raises:
            AiLifecycleError: If the command exits with a non-zero code or
                              cannot be found.
        """
        self._logger.debug("$ %s", " ".join(cmd))
        try:
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError as exc:
            raise AiLifecycleError(
                f"'{cmd[0]}' not found. Is uv installed?", cause=exc
            ) from exc

        if result.returncode != 0:
            self._logger.error(
                "%s failed (exit %d):\n%s", label, result.returncode, result.stderr
            )
            raise AiLifecycleError(
                f"{label} failed with exit code {result.returncode}: {result.stderr}"
            )

        if result.stdout:
            self._logger.debug("%s stdout:\n%s", label, result.stdout)
