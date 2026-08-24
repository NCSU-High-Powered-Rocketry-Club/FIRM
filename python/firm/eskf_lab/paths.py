"""Repository paths used by the ESKF lab."""

from __future__ import annotations

from pathlib import Path


def find_repo_root() -> Path:
    """Find the FIRM repository from the installed package or current directory."""
    candidates = [Path.cwd(), *Path.cwd().parents, Path(__file__).resolve().parents[3]]
    for candidate in candidates:
        if (candidate / "STM32" / "Core" / "Src" / "data_processing").is_dir():
            return candidate.resolve()
    raise RuntimeError("could not locate the FIRM repository root")


REPO_ROOT = find_repo_root()
LAB_ROOT = REPO_ROOT / "eskf_lab"
DEFAULT_FLIGHT_DATA_DIR = REPO_ROOT / "flight_data"
DEFAULT_CACHE_DIR = LAB_ROOT / "cache"
DEFAULT_RESULTS_DIR = LAB_ROOT / "results"
DEFAULT_BUILD_DIR = LAB_ROOT / "build" / "native"
DEFAULT_PROFILE = LAB_ROOT / "config" / "default.toml"
