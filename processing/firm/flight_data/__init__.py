"""Versioned, immutable flight-log archive management."""

from .archive import Archive, Launch, Recording
from .build import build_recording
from .formats import LogReader, LogWriter
from .models import (
    BuildRequest,
    BuildResult,
    CalibrationOverride,
    EffectiveCalibration,
    LogHeader,
    LogPacket,
    TrimProposal,
    TrimWindow,
    ValidationReport,
)

__all__ = [
    "Archive",
    "BuildRequest",
    "BuildResult",
    "CalibrationOverride",
    "EffectiveCalibration",
    "Launch",
    "LogHeader",
    "LogPacket",
    "LogReader",
    "LogWriter",
    "Recording",
    "TrimProposal",
    "TrimWindow",
    "ValidationReport",
    "build_recording",
]

__version__ = "0.1.0"
