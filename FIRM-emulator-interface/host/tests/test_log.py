from dataclasses import replace

import pytest

from firm_emu.cli import main
from firm_emu.log import (
    BLOCK_SIZE,
    Record,
    by_sensor,
    compare_logs,
    compare_sensor,
    read_log,
    sensor_stats,
    write_log,
)

# rate (Hz) and payload size for each sensor, matching LOG90.FRM
SENSORS = {"barometer": (250, 6), "imu": (400, 15), "magnetometer": (220, 7), "high_g": (290, 6)}


def make_records(seconds: float = 1.0) -> list[Record]:
    """A fake flight: every sensor at its real rate, with a different payload each sample."""
    records = [
        Record(name, n / hz, bytes((n + k) % 256 for k in range(size)))
        for name, (hz, size) in SENSORS.items()
        for n in range(int(seconds * hz))
    ]
    return sorted(records, key=lambda r: r.time_s)


def imu(records: list[Record]) -> list[Record]:
    return by_sensor(records)["imu"]


@pytest.fixture
def log_file(tmp_path):
    path = tmp_path / "flight.frm"
    write_log(path, make_records())
    return path


def test_write_then_read_gives_the_same_records(log_file):
    header, records = read_log(log_file)
    original = make_records()

    assert header.version == "1.4"
    assert log_file.stat().st_size % BLOCK_SIZE == 0
    assert [(r.sensor, r.payload) for r in records] == [(r.sensor, r.payload) for r in original]
    assert all(abs(a.time_s - b.time_s) < 1e-6 for a, b in zip(records, original, strict=True))


def test_sensor_stats_finds_the_rate():
    stats = sensor_stats(imu(make_records()))

    assert stats.count == 400
    assert stats.rate_hz == pytest.approx(400)
    assert stats.max_gap_ms == pytest.approx(2.5)


def test_identical_logs_match():
    diff = compare_sensor(imu(make_records()), imu(make_records()))

    assert diff.passed
    assert diff.matched == 400
    assert diff.missing == diff.extra == diff.wrong == 0


def test_a_few_dropped_records_pass():
    expected = imu(make_records())
    actual = [r for n, r in enumerate(expected) if n not in {10, 11, 200}]

    diff = compare_sensor(expected, actual)

    assert diff.passed
    assert diff.missing == 3


def test_many_dropped_records_fail():
    expected = imu(make_records())

    assert not compare_sensor(expected, expected[::2]).passed
    assert not compare_sensor(expected, []).passed


def test_changed_bytes_fail():
    expected = imu(make_records())
    actual = list(expected)
    actual[50] = replace(actual[50], payload=bytes(15))

    diff = compare_sensor(expected, actual)

    assert not diff.passed
    assert diff.wrong == 1
    assert diff.first_problem_s == pytest.approx(expected[50].time_s)


def test_extra_record_fails():
    expected = imu(make_records())
    actual = sorted([*expected, Record("imu", 0.1012, bytes(15))], key=lambda r: r.time_s)

    diff = compare_sensor(expected, actual)

    assert not diff.passed
    assert diff.extra == 1


def test_double_read_is_missing_not_wrong():
    # high-g is sometimes read twice within 1 ms; dropping the first read is not a mismatch
    expected = [Record("high_g", t, bytes([n])) for n, t in enumerate((0.0, 0.0002, 0.0035))]

    diff = compare_sensor(expected, expected[1:])

    assert diff.missing == 1
    assert diff.wrong == diff.extra == 0


def test_slow_clock_drift_passes():
    expected = imu(make_records(seconds=2))
    actual = [replace(r, time_s=r.time_s * 1.001) for r in expected]  # 2 ms behind by the end

    diff = compare_sensor(expected, actual)

    assert diff.passed
    assert diff.drift_ms == pytest.approx(2.0, abs=0.3)  # the offset follows a little behind


def test_late_record_fails_without_moving_the_rest():
    expected = imu(make_records())
    actual = list(expected)
    actual[100] = replace(actual[100], time_s=actual[100].time_s + 0.003)

    diff = compare_sensor(expected, actual)

    assert not diff.passed
    assert diff.late == 1
    assert diff.matched == 399
    assert diff.worst_ms == pytest.approx(3.0)


def test_compare_logs_checks_every_sensor(tmp_path, log_file):
    broken = make_records()
    broken[-1] = replace(broken[-1], payload=bytes(len(broken[-1].payload)))
    write_log(tmp_path / "broken.frm", broken)

    diffs = compare_logs(log_file, tmp_path / "broken.frm")

    assert [name for name, d in diffs.items() if not d.passed] == [broken[-1].sensor]


def test_cli_inspect(log_file, capsys):
    assert main(["inspect", str(log_file)]) == 0
    output = capsys.readouterr().out
    assert "log v1.4" in output
    assert "imu" in output


def test_cli_compare_pass_and_fail(tmp_path, log_file, capsys):
    assert main(["compare", str(log_file), str(log_file)]) == 0
    assert capsys.readouterr().out.strip().endswith("PASS")

    write_log(tmp_path / "empty.frm", [])
    assert main(["compare", str(log_file), str(tmp_path / "empty.frm")]) == 1
    assert capsys.readouterr().out.strip().endswith("FAIL")
