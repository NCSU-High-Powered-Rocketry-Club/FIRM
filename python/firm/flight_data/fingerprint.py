"""Stable build-input fingerprint shared by the manager and its consumers."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import TYPE_CHECKING

from .formats import LATEST_VERSION

if TYPE_CHECKING:
    from .archive import Recording

BUILD_SCHEMA_VERSION = 2
DECODER_SCHEMA_VERSION = 1
FORMAT_SCHEMA_VERSION = 2


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_fingerprint(recording: Recording, target_version: str = LATEST_VERSION) -> str:
    """Hash every authoritative input and processing-schema version."""
    manifest = recording.manifest
    override_manifest = manifest.get("calibration_override")
    override_actual = (
        _sha256_file(recording.calibration_override)
        if recording.calibration_override.is_file()
        else None
    )
    inputs = {
        "build_schema_version": BUILD_SCHEMA_VERSION,
        "decoder_schema_version": DECODER_SCHEMA_VERSION,
        "format_schema_version": FORMAT_SCHEMA_VERSION,
        "target_version": target_version,
        "source_hardware": manifest.get("source_hardware"),
        "sources": manifest.get("sources"),
        "trim_window": manifest.get("trim_window"),
        "calibration_override": {
            "manifest": override_manifest,
            "actual_sha256": override_actual,
        },
    }
    return hashlib.sha256(
        json.dumps(inputs, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
