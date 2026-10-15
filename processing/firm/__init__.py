"""FIRM utilities for flight-log archiving, ESKF replay, and trace conversion.

Tools:
    firm-log: Archive and manage flight logs.
    firm-eskf: Replay the firmware ESKF against archived recordings.
    firm-trace: Convert trace formats.
    firm-reconstruct: Recover a .frm log from legacy decoder CSVs.

The live USB client moved to the separate ``firm-client`` package
(``from firm_client import FIRMClient``); ``firm.FIRM`` was removed in 0.4.0.
"""

__version__ = "0.4.0"

__all__ = ("__version__",)
