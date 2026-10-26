import pytest

from firm_emu.cli import main
from firm_emu.log import Record, compare_logs, write_log
from firm_emu.replay import FileOutput, RealClock, replay
from tests.test_log import imu, make_records


class FakeClock:
    """Jumps straight to each deadline (plus an optional delay), so tests run instantly."""

    def __init__(self, delay_s: float = 0.0):
        self.t = 100.0
        self.delay_s = delay_s

    def now(self) -> float:
        """Fake time in seconds."""
        return self.t

    def wait_until(self, deadline: float) -> None:
        """Jump to the deadline, then add the delay."""
        self.t = max(self.t, deadline) + self.delay_s


class ListOutput:
    """Keeps sent records in memory."""

    def __init__(self):
        self.records: list[Record] = []
        self.closed = False

    def send(self, record: Record) -> None:
        """Keep the record."""
        self.records.append(record)

    def close(self) -> None:
        """Remember that close was called."""
        self.closed = True


def times(records: list[Record]) -> list[float]:
    return [r.time_s for r in records]


def test_replay_keeps_the_original_timing():
    records, output = make_records(), ListOutput()

    stats = replay(records, output, clock=FakeClock())

    assert output.closed
    assert stats.sent == len(records)
    assert stats.max_late_ms == 0
    assert times(output.records) == pytest.approx(times(records))


def test_speed_shortens_real_time_but_not_log_time():
    records, output = make_records(), ListOutput()

    stats = replay(records, output, speed=4, clock=FakeClock())

    assert stats.duration_s == pytest.approx(records[-1].time_s / 4)
    assert times(output.records) == pytest.approx(times(records))


def test_late_sends_are_measured():
    records, output = imu(make_records()), ListOutput()

    stats = replay(records, output, clock=FakeClock(delay_s=0.0005))

    assert stats.max_late_ms == pytest.approx(0.5)
    assert times(output.records) == pytest.approx([t + 0.0005 for t in times(records)])


def test_replayed_file_matches_the_original(tmp_path):
    original = tmp_path / "flight.frm"
    write_log(original, make_records())

    replay(make_records(), FileOutput(tmp_path / "out.frm"), clock=FakeClock())

    assert all(d.passed for d in compare_logs(original, tmp_path / "out.frm").values())


def test_real_clock_keeps_time():
    records = [r for r in imu(make_records()) if r.time_s < 0.2]

    stats = replay(records, ListOutput(), clock=RealClock())

    assert stats.duration_s >= records[-1].time_s
    assert stats.max_late_ms < 5  # loose, so a busy machine does not fail the test


def test_cli_run_then_compare(tmp_path, capsys):
    log, out = str(tmp_path / "flight.frm"), str(tmp_path / "out.frm")
    write_log(log, make_records())

    assert main(["run", log, "--out", out, "--seconds", "0.2"]) == 0
    assert main(["compare", log, out, "--seconds", "0.2", "--tolerance-ms", "5"]) == 0
    assert main(["compare", log, out]) == 1  # the rest of the log was never replayed
    assert "worst delay" in capsys.readouterr().out
