"""Build host-side ESKF shared library for Python-based desktop testing."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent.parent

DATA_PROCESSING_SRC = REPO_ROOT / "STM32" / "Core" / "Src" / "data_processing"
DATA_PROCESSING_INC = REPO_ROOT / "STM32" / "Core" / "Inc" / "data_processing"
BUILD_DIR = THIS_DIR / "build"
OBJ_DIR = BUILD_DIR / "obj"

SOURCES = [
    THIS_DIR / "setup.c",
    DATA_PROCESSING_SRC / "matrix_helper.c",
    DATA_PROCESSING_SRC / "eskf_functions.c",
    DATA_PROCESSING_SRC / "eskf_config.c",
    DATA_PROCESSING_SRC / "error_state_kalman_filter.c",
]


class BuildError(RuntimeError):
    """Raised when compiling or linking the host filter library fails."""


def _shared_library_name() -> str:
    if sys.platform.startswith("win"):
        return "firm_filter.dll"
    if sys.platform == "darwin":
        return "libfirm_filter.dylib"
    return "libfirm_filter.so"


def _shared_library_path() -> Path:
    return BUILD_DIR / _shared_library_name()


def _run(cmd: list[str], *, verbose: bool) -> None:
    if verbose:
        sys.stdout.write(f"[build] {' '.join(cmd)}\n")

    completed = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise BuildError(
            "\n".join(
                [
                    f"Command failed: {' '.join(cmd)}",
                    "--- stdout ---",
                    completed.stdout.strip(),
                    "--- stderr ---",
                    completed.stderr.strip(),
                ]
            )
        )


def _needs_rebuild(output: Path, sources: list[Path], force: bool) -> bool:
    if force or not output.exists():
        return True

    out_mtime = output.stat().st_mtime
    return any(src.stat().st_mtime > out_mtime for src in sources)


def build_filter_shared_library(force: bool = False, verbose: bool = False) -> Path:
    """Compile and link the host ESKF shared library and return its path."""
    gcc = shutil.which("gcc")
    if gcc is None:
        raise BuildError(
            "gcc was not found in PATH. Install MinGW/MSYS2 gcc and ensure it is available."
        )

    for src in SOURCES:
        if not src.exists():
            raise BuildError(f"Missing source file: {src}")

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    OBJ_DIR.mkdir(parents=True, exist_ok=True)

    output = _shared_library_path()
    if not _needs_rebuild(output, SOURCES, force):
        if verbose:
            sys.stdout.write(f"[build] Up to date: {output}\n")
        return output

    objects: list[Path] = []
    common_flags = [
        "-Ofast",
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-DFILTER_TESTING_BUILD_DLL",
    ]

    for src in SOURCES:
        obj = OBJ_DIR / f"{src.stem}.o"
        cmd = [
            gcc,
            "-c",
            str(src),
            "-I",
            str(DATA_PROCESSING_INC),
            "-I",
            str(THIS_DIR),
            "-o",
            str(obj),
            *common_flags,
        ]
        if not sys.platform.startswith("win"):
            cmd.append("-fPIC")
        _run(cmd, verbose=verbose)
        objects.append(obj)

    link_cmd = [
        gcc,
        "-shared",
        "-o",
        str(output),
        *[str(obj) for obj in objects],
        "-lm",
    ]
    _run(link_cmd, verbose=verbose)

    if verbose:
        sys.stdout.write(f"[build] Built: {output}\n")
    return output


def main() -> int:
    """CLI entry point for building the host-side shared library."""
    parser = argparse.ArgumentParser(description="Build host-side ESKF shared library.")
    parser.add_argument("--force", action="store_true", help="Force rebuild even if up to date")
    parser.add_argument("--verbose", action="store_true", help="Print build commands")
    args = parser.parse_args()

    try:
        output = build_filter_shared_library(force=args.force, verbose=args.verbose)
    except BuildError as exc:
        sys.stderr.write(f"{exc}\n")
        return 1

    sys.stdout.write(f"{output}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
