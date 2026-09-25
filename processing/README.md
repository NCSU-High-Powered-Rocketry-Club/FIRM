# Processing (`firm-hprc`)

Offline FIRM tools: log archive management, ESKF replay, and FreeRTOS trace conversion.

Install from this repository with `just sync` (or `uv sync --all-packages --extra usb --group dev` at the repo root). The published package name is `firm-hprc`; imports are `firm` and `eskf_lab`.

Live USB is **not** this package. Use `from firm_client import FIRMClient` from `firm-client` (`usb` extra).

## Commands

| Command | Purpose |
|---------|---------|
| `firm-log` | Ingest, trim, calibrate, build, and export versioned flight logs |
| `firm-eskf` | Replay production ESKF C sources against built recordings |
| `firm-trace` | Convert a GDB `trace.bin` dump to JSON for Spall or Perfetto |
| `firm-reconstruct` | Recover a v1.4 `.frm` from legacy decoder CSVs |

```bash
uv run firm-log ingest --launch my-launch --recording primary --flight LOG1.FRM --hardware new
uv run firm-log build my-launch/primary
uv run firm-log trim preview my-launch/primary --phases
uv run firm-log calibration show my-launch/primary
uv run firm-eskf run my-launch/primary
uv run firm-trace -i STM32/trace.bin -o trace.json
```

See [eskf_lab/README.md](eskf_lab/README.md) for the filter lab workflow.

## Layout

```
processing/
  firm/           log archive (`firm.flight_data`) and CLI entry points
  eskf_lab/       Python lab + native CMake replay of STM32 ESKF sources
```
