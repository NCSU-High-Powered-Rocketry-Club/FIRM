# Laptop - Teensy protocol

How the laptop sends a recorded flight to the Teensy over USB serial, and what the Teensy reports
back. The laptop side is `host/firm_emu/protocol.py`; `host/firm_emu/fake_teensy.py` is a Python
stand-in for the Teensy that speaks the same protocol.

## Idea

- The laptop streams samples ahead of time. The Teensy buffers them and plays each one when it is
  due, using its own clock, so USB delays do not affect timing.
- The Teensy reports how much buffer space is free. The laptop never sends more than that.
- A sample that arrives after its due time is played right away and counted as late.

## Frames

Every message is one frame:

```
COBS( type | payload | crc ) 0x00
```

| Field | Size | Notes |
|-------|------|-------|
| type | 1 | Message type, see below |
| payload | 0-1024 | Depends on the type |
| crc | 2 | CRC-16/CCITT-FALSE of type and payload, little endian |

COBS encoding removes every 0x00 byte, so 0x00 only marks the end of a frame. After a reset or a
bad frame, the reader just waits for the next 0x00. All numbers are little endian.

## Laptop to Teensy

| Type | Name | Payload | Effect |
|------|------|---------|--------|
| 0x01 | PING | none | Teensy replies PONG |
| 0x02 | LOAD | samples, back to back | Add samples to the buffer |
| 0x03 | START | none | Start the clock at 0 and begin playing |
| 0x04 | END | none | No more samples are coming |
| 0x05 | STOP | none | Stop, clear the buffer, go back to idle |
| 0x06 | GET_STATUS | none | Teensy replies STATUS |

A sample is one log record with a time in microseconds:

| Size | Field |
|------|-------|
| 1 | Sensor ID: `B`, `I`, `M`, `A` (same as the log) |
| 4 | Due time in microseconds since START, uint32 |
| N | Raw register bytes (6, 15, 7 or 6, same as the log) |

A uint32 of microseconds lasts 71 minutes, longer than any flight log.

## Teensy to laptop

| Type | Name | Payload |
|------|------|---------|
| 0x81 | PONG | protocol version (uint8, currently 1) |
| 0x82 | STATUS | see below |
| 0x83 | EVENT | kind (uint8), time in microseconds (uint32), value (uint8) |
| 0x84 | ERROR | error code (uint8) |

STATUS payload:

| Size | Field |
|------|-------|
| 1 | State: 0 idle, 1 running, 2 done |
| 4 | Free buffer bytes |
| 4 | Samples played |
| 4 | Samples played late |
| 4 | Worst lateness in microseconds |
| 4 | Time since START in microseconds |

The state is done once END was received and the buffer is empty.

EVENT kinds (FIRM outputs the Teensy watches):

| Kind | Meaning | Value |
|------|---------|-------|
| 1 | Debug LEDs changed | LED bits: DEBUG0 = bit 2, DEBUG1 = bit 1, DEBUG2 = bit 0 |

Pyro, buzzer and other outputs get new kinds when v2.0 support is added.

ERROR codes:

| Code | Meaning |
|------|---------|
| 1 | Bad frame (COBS or CRC) |
| 2 | Unknown message type |
| 3 | LOAD does not fit in the buffer; the whole frame is dropped |
| 4 | LOAD has a bad sample (unknown sensor ID or cut short) |

## A replay

```
laptop                         Teensy
STOP                    ->               clear anything left over
GET_STATUS              ->
                        <-     STATUS    free = 262144
LOAD, LOAD, ...         ->               fill the buffer
START                   ->               clock starts, samples play when due
every 50 ms:
  GET_STATUS            ->
                        <-     STATUS    free = space played since last time
  LOAD, ...             ->               top the buffer back up
END                     ->               after the last sample
GET_STATUS ...          ->               until state is done
```

The buffer is 256 KB, about 15 s of LOG90.FRM, so the laptop can stall for seconds without the
Teensy running dry.
