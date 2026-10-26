import pytest

from firm_emu.cli import main
from firm_emu.fake_teensy import FakeTeensy
from firm_emu.log import Record, write_log
from firm_emu.protocol import (
    DONE,
    LED_EVENT,
    LOAD,
    MAX_PAYLOAD,
    FrameError,
    FrameReader,
    Teensy,
    TeensyError,
    cobs_decode,
    cobs_encode,
    decode_frame,
    encode_frame,
    pack_sample,
    unpack_samples,
)
from tests.test_log import make_records
from tests.test_replay import FakeClock


def connect(buffer_bytes: int = 256 * 1024) -> tuple[Teensy, FakeTeensy]:
    clock = FakeClock()
    fake = FakeTeensy(clock, buffer_bytes)
    return Teensy(fake, clock), fake


@pytest.mark.parametrize(
    "data",
    [b"", b"\0", b"\0\0", b"\x01" * 254, b"\x01" * 255, b"\x01" * 254 + b"\0", bytes(range(256))],
)
def test_cobs_round_trip_has_no_zeros(data):
    encoded = cobs_encode(data)

    assert 0 not in encoded
    assert cobs_decode(encoded) == data


def test_frame_round_trip_and_crc_check():
    frame = encode_frame(LOAD, b"\x00\x01\x02")
    assert decode_frame(frame[:-1]) == (LOAD, b"\x00\x01\x02")

    corrupted = bytearray(frame[:-1])
    corrupted[2] ^= 0x10
    with pytest.raises(FrameError):
        decode_frame(bytes(corrupted))


def test_frame_reader_handles_frames_split_across_reads():
    data = encode_frame(1, b"a") + encode_frame(2, b"bc")
    reader = FrameReader()

    first = reader.feed(data[:5])
    rest = reader.feed(data[5:])

    assert [decode_frame(f) for f in first + rest] == [(1, b"a"), (2, b"bc")]


def test_samples_round_trip():
    records = make_records(seconds=0.1)

    unpacked = unpack_samples(b"".join(pack_sample(r) for r in records))

    assert [(r.sensor, r.payload) for r in unpacked] == [(r.sensor, r.payload) for r in records]
    assert [r.time_s for r in unpacked] == pytest.approx([r.time_s for r in records], abs=1e-6)


def test_bad_sample_is_rejected():
    with pytest.raises(ValueError, match="unknown sensor"):
        unpack_samples(b"X\0\0\0\0")
    with pytest.raises(ValueError, match="cut short"):
        unpack_samples(pack_sample(make_records()[0])[:-1])


def test_ping():
    teensy, _ = connect()
    assert teensy.ping() == 1


def test_play_delivers_every_sample_on_time():
    teensy, fake = connect()
    records = make_records()

    status = teensy.play(records)

    assert status.state == DONE
    assert status.played == len(records)
    assert status.late == 0
    assert [(r.sensor, r.payload) for r in fake.played] == [(r.sensor, r.payload) for r in records]
    assert [r.time_s for r in fake.played] == pytest.approx([r.time_s for r in records], abs=1e-6)


def test_small_buffer_is_topped_up_without_overflowing():
    teensy, fake = connect(buffer_bytes=300)  # about 17 ms of data

    status = teensy.play(make_records(), poll_s=0.005)

    assert status.played == len(make_records())
    assert status.late == 0
    assert fake.used == 0


def test_buffer_running_dry_counts_late_samples():
    teensy, _ = connect(buffer_bytes=100)  # about 6 ms of data

    status = teensy.play(make_records(), poll_s=0.05)

    assert status.late > 0
    assert status.max_late_us > 0


def test_oversized_load_is_reported():
    teensy, _ = connect(buffer_bytes=100)
    teensy.send(LOAD, bytes(MAX_PAYLOAD))

    with pytest.raises(TeensyError, match="code 3"):
        teensy.status()


def test_unknown_message_is_reported():
    teensy, _ = connect()
    teensy.send(0x7F)

    with pytest.raises(TeensyError, match="code 2"):
        teensy.status()


def test_events_are_collected():
    teensy, fake = connect()
    teensy.play(make_records(seconds=0.1))

    fake.emit_event(LED_EVENT, 0b110)
    teensy.status()

    [event] = teensy.events
    assert (event.kind, event.value) == (LED_EVENT, 0b110)
    assert event.time_us > 0


def test_cli_run_fake_then_compare(tmp_path, capsys):
    log, out = str(tmp_path / "flight.frm"), str(tmp_path / "played.frm")
    write_log(log, make_records())

    assert main(["run", log, "--fake", "--seconds", "0.2", "--out", out]) == 0
    assert main(["compare", log, out, "--seconds", "0.2"]) == 0
    assert "late 0" in capsys.readouterr().out


def test_cli_run_needs_out_or_fake(tmp_path):
    log = str(tmp_path / "flight.frm")
    write_log(log, [Record("imu", 0.0, bytes(15))])

    with pytest.raises(SystemExit):
        main(["run", log])
