import init, {
  FIRMDataParser,
  FIRMCommandBuilder,
  MagnetometerCalibrator,
  MockLogParser,
  mock_header_size,
} from '../../pkg/firm_client.js';
import {
  FIRMPacket,
  FIRMResponse,
  DeviceInfo,
  DeviceConfig,
  DeviceProtocol,
  CalibrationValues,
} from './types.js';

const RESPONSE_TIMEOUT_MS = 5000;

/** Options for connecting to a FIRM device over Web Serial. */
export interface FIRMConnectOptions {
  /** Serial baud rate (default: 2000000). */
  baudRate?: number;
}

export interface MockStreamOptions {
  realtime?: boolean;
  speed?: number;
  chunkSize?: number;
  startTimeoutMs?: number;
}

/**
 * High-level browser API for the FIRM parser.
 *
 * - Handles Web Serial connection.
 * - Streams raw bytes into the Rust parser (wasm).
 * - Exposes methods to read parsed telemetry packets.
 */
export class FIRMClient {
  /** Underlying WASM-backed streaming parser. */
  private dataParser: FIRMDataParser;

  /** Serial port (Web Serial API). Kept so we can close/reconnect cleanly. */
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private port: any | null = null;

  /** Subscribers for raw incoming serial bytes. */
  private rawBytesListeners: ((bytes: Uint8Array) => void)[] = [];

  /** Subscribers for raw outgoing serial bytes. */
  private outgoingBytesListeners: ((bytes: Uint8Array) => void)[] = [];

  /** Subscribers for parsed data packets (snoop; does not consume queue). */
  private packetListeners: ((pkt: FIRMPacket) => void)[] = [];

  /** Reader for the Web Serial stream. */
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;

  /** Writer for the Web Serial stream. */
  private writer: WritableStreamDefaultWriter<Uint8Array> | null = null;

  /** Whether the background read loop is currently running. */
  private running = false;

  private packetQueue: FIRMPacket[] = [];
  private packetWaiters: ((pkt: FIRMPacket | null) => void)[] = [];
  private responseQueue: FIRMResponse[] = [];
  private responseWaiters: ((res: FIRMResponse) => void)[] = [];

  private closed = false;

  private constructor(wasm: FIRMDataParser) {
    this.dataParser = wasm;
  }

  private async sleep(ms: number): Promise<void> {
    await new Promise((resolve) => setTimeout(resolve, ms));
  }

  private async safeCall(obj: unknown, fn: string): Promise<void> {
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const target = obj as any;
    if (!target || typeof target[fn] !== 'function') return;
    try {
      if (fn === 'close' || fn === 'cancel') {
        await target[fn]();
      } else {
        target[fn]();
      }
    } catch {}
  }

  /**
   * Connects to a serial device and starts the background read loop.
   *
   * @param options Connection options (e.g. baudRate).
   * @returns A connected FIRM instance.
   */
  static async connect(options: FIRMConnectOptions = {}): Promise<FIRMClient> {
    if (!('serial' in navigator)) throw new Error('Web Serial API not available');
    await init();
    const dataParser = new FIRMDataParser();
    const baudRate = options.baudRate ?? 2000000;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const port = await (navigator as any).serial.requestPort();
    await port.open({ baudRate });
    const reader = port.readable.getReader();
    const writer = port.writable.getWriter();
    const firm = new FIRMClient(dataParser);
    firm.port = port;
    firm.reader = reader;
    firm.writer = writer;
    firm
      .startReadLoop()
      .catch((err) => console.warn('[FIRM] read loop task failed (ignored):', err));
    return firm;
  }

  /**
   * Internal read loop that continuously reads from Web Serial,
   * feeds raw bytes into the WASM parser, and enqueues parsed packets.
   */
  private async startReadLoop(): Promise<void> {
    if (!this.reader) return;
    this.running = true;
    try {
      while (this.running) {
        const { value, done } = await this.reader.read();
        if (done || !this.running) break;
        if (value?.length) {
          this.rawBytesListeners.forEach((fn) => {
            try {
              fn(value);
            } catch (e) {
              console.warn('[FIRM] raw bytes listener error:', e);
            }
          });
          this.dataParser.parse_bytes(value);
          let pkt;
          while ((pkt = this.dataParser.get_packet() as FIRMPacket | null)) this.enqueuePacket(pkt);
          let res;
          while ((res = this.dataParser.get_response() as FIRMResponse | null))
            this.enqueueResponse(res);
        }
      }
    } catch (err) {
      console.warn('[FIRM] read loop ended:', err);
      this.flushWaitersWithNull();
    } finally {
      this.running = false;
      this.flushWaitersWithNull();
      await this.safeCall(this.reader, 'cancel');
      await this.safeCall(this.reader, 'releaseLock');
      await this.safeCall(this.writer, 'close');
      await this.safeCall(this.writer, 'releaseLock');
      await this.safeCall(this.port, 'close');
      this.reader = null;
      this.writer = null;
      this.port = null;
    }
  }

  /**
   * Sends raw bytes to the device.
   * @param bytes The bytes to send.
   */
  async sendBytes(bytes: Uint8Array): Promise<void> {
    if (!this.writer) throw new Error('Writer not available');

    this.outgoingBytesListeners.forEach((fn) => {
      try {
        fn(bytes);
      } catch (e) {
        console.warn('[FIRM] outgoing bytes listener error:', e);
      }
    });
    await this.writer.write(bytes);
  }

  /**
   * @param listener Callback invoked with each incoming chunk.
   * @returns Unsubscribe function.
   */
  onRawBytes(listener: (bytes: Uint8Array) => void): () => void {
    this.rawBytesListeners.push(listener);
    return () => {
      const idx = this.rawBytesListeners.indexOf(listener);
      if (idx !== -1) {
        this.rawBytesListeners.splice(idx, 1);
      }
    };
  }

  /**
   * @param listener Callback invoked with each outgoing write.
   * @returns Unsubscribe function.
   */
  onOutgoingBytes(listener: (bytes: Uint8Array) => void): () => void {
    this.outgoingBytesListeners.push(listener);
    return () => {
      const idx = this.outgoingBytesListeners.indexOf(listener);
      if (idx !== -1) {
        this.outgoingBytesListeners.splice(idx, 1);
      }
    };
  }

  /**
   * Sends a command to get device info.
   * @returns The device info, or null if the request timed out.
   */
  private async sendAndWait<T>(
    buildCmd: () => Uint8Array,
    matcher: (res: FIRMResponse) => T | undefined,
    timeout = RESPONSE_TIMEOUT_MS,
  ): Promise<T | null> {
    await this.sendBytes(buildCmd());
    try {
      return await this.waitForResponse(matcher, timeout);
    } catch {
      return null;
    }
  }

  /**
   * Streams a mock log file to the device (used for hardware mock mode).
   */
  async streamMockLogFile(file: File, options: MockStreamOptions = {}): Promise<number> {
    if (!this.writer) throw new Error('Writer not available');

    const realtime = options.realtime ?? true;
    const speed = options.speed ?? 1.0;
    const chunkSize = options.chunkSize ?? 8192;
    const startTimeoutMs = options.startTimeoutMs ?? RESPONSE_TIMEOUT_MS;

    if (speed <= 0) throw new Error('speed must be > 0');

    await this.sendBytes(FIRMCommandBuilder.build_mock());
    const ok = await this.waitForResponse(
      (res) => ('Mock' in res ? res.Mock : undefined),
      startTimeoutMs,
    ).catch(() => null);

    if (!ok) throw new Error('Mock mode not acknowledged');

    const data = new Uint8Array(await file.arrayBuffer());
    const headerSize = mock_header_size();
    if (data.length < headerSize) throw new Error('Log file too small');

    const header = data.slice(0, headerSize);
    const body = data.slice(headerSize);

    const parser = new MockLogParser();
    parser.read_header(header);
    await this.sendBytes(parser.build_header_packet(header));

    let sent = 0;

    // We want ONE burst of 75 total, not 75 per chunk.
    // So we track whether we've already done the preload.
    let didPreload = false;

    for (let offset = 0; offset < body.length; offset += chunkSize) {
      parser.parse_bytes(body.slice(offset, offset + chunkSize));

      // First drain does the preload burst, later drains do only batching
      if (!didPreload) {
        sent += await this.drainMockPacketsBatched(parser, realtime, speed, 75, 10);
        didPreload = true;
      } else {
        // No preload after the first call; still batch in 10s
        sent += await this.drainMockPacketsBatched(parser, realtime, speed, 0, 10);
      }
    }

    // Final drain in case parser buffered leftovers
    if (!didPreload) {
      sent += await this.drainMockPacketsBatched(parser, realtime, speed, 75, 10);
    } else {
      sent += await this.drainMockPacketsBatched(parser, realtime, speed, 0, 10);
    }

    return sent;  
  }

  /** Sends multiple frames as a single byte array. */
  private async sendBatch(frames: Uint8Array[]): Promise<void> {
    let total = 0;
    for (const f of frames) total += f.length;

    const out = new Uint8Array(total);
    let off = 0;
    for (const f of frames) {
      out.set(f, off);
      off += f.length;
    }

    await this.sendBytes(out);
  }

  private async drainMockPacketsBatched(
    parser: MockLogParser,
    realtime: boolean,
    speed: number,
    preloadCount = 75,
    batchSize = 10,
  ): Promise<number> {
    // Stage parsed packets here so we can burst + batch
    const staged: { bytes: Uint8Array; delaySeconds: number }[] = [];

    const popOne = () => {
      if (staged.length > 0) return staged.shift()!;
      const pkt = parser.get_packet_with_delay() as
        | { bytes: Uint8Array; delaySeconds: number }
        | null;
      return pkt ?? null;
    };

    const fillUntil = (n: number) => {
      while (staged.length < n) {
        const pkt = parser.get_packet_with_delay() as
          | { bytes: Uint8Array; delaySeconds: number }
          | null;
        if (!pkt) break;
        staged.push(pkt);
      }
    };

    let sent = 0;

    // -------------------------
    // 1) PRELOAD burst (no sleep)
    // -------------------------
    fillUntil(preloadCount);

    const batch: Uint8Array[] = [];
    while (batch.length < preloadCount) {
      const pkt = popOne();
      if (!pkt) break;
      batch.push(pkt.bytes);
    }

    await this.sendBatch(batch);
    sent += batch.length;

    // Reset pacing after burst: we pace relative to "now"
    const streamStart = performance.now();
    let totalDelaySeconds = 0;

    // -----------------------------------------
    // 2) MAIN: batches of N (one sleep per batch)
    // -----------------------------------------
    while (true) {
      // Make sure we have at least one packet available
      fillUntil(batchSize);
      if (staged.length === 0) {
        const pkt = popOne();
        if (!pkt) break;
        staged.push(pkt);
      }

      // Build a batch up to batchSize
      const batch = staged.splice(0, batchSize);
      if (batch.length === 0) break;

      let batchDelaySeconds = 0;
      for (const pkt of batch) batchDelaySeconds += pkt.delaySeconds;

      await this.sendBatch(batch.map((p) => p.bytes));
      sent += batch.length;

      // Sleep once per batch to approximate original pacing
      if (realtime && batchDelaySeconds > 0) {
        totalDelaySeconds += batchDelaySeconds;

        const elapsedSeconds = (performance.now() - streamStart) / 1000;
        const targetSeconds = totalDelaySeconds / speed;

        // Only sleep if we're not already behind
        if (elapsedSeconds <= targetSeconds) {
          await this.sleep((batchDelaySeconds * 1000) / speed);
        }
      }
    }

    return sent;
  }

  /**
   * Gets device info from the FIRM device.
   * @returns The device info, or null if the request timed out.
   */
  async getDeviceInfo(): Promise<DeviceInfo | null> {
    return this.sendAndWait(
      () => FIRMCommandBuilder.build_get_device_info(),
      (res) => ('GetDeviceInfo' in res ? res.GetDeviceInfo : undefined),
    );
  }

  /**
   * Gets device configuration from the FIRM device.
   * @returns The device configuration, or null if the request timed out.
   */
  async getDeviceConfig(): Promise<DeviceConfig | null> {
    return this.sendAndWait(
      () => FIRMCommandBuilder.build_get_device_config(),
      (res) => ('GetDeviceConfig' in res ? res.GetDeviceConfig : undefined),
    );
  }

  /**
   * Gets device calibration values from the FIRM device.
   * @returns The calibration values, or null if the request timed out.
   */
  async getCalibration(): Promise<CalibrationValues | null> {
    return this.sendAndWait(
      () => FIRMCommandBuilder.build_get_calibration(),
      (res) => ('GetCalibration' in res ? res.GetCalibration : undefined),
    );
  }

  /**
   * Sets device configuration on the FIRM device.
   * @param name the device name, 32 characters max.
   * @param frequency the data frequency in Hz, 1-1000 Hz.
   * @param protocol the communication protocol.
   * @returns True if the configuration was set successfully, false otherwise.
   */
  async setDeviceConfig(
    name: string,
    frequency: number,
    protocol: DeviceProtocol,
  ): Promise<boolean> {
    return (
      (await this.sendAndWait(
        () => FIRMCommandBuilder.build_set_device_config(name, frequency, protocol),
        (res) => ('SetDeviceConfig' in res ? res.SetDeviceConfig : undefined),
      )) ?? false
    );
  }

  async setIMUCalibration(
    accel_offsets: number[],
    accel_scale_matrix: number[],
    gyro_offsets: number[],
    gyro_scale_matrix: number[],
  ): Promise<boolean> {
    const accelOffsetsF32 = new Float32Array(accel_offsets);
    const accelScaleF32 = new Float32Array(accel_scale_matrix);
    const gyroOffsetsF32 = new Float32Array(gyro_offsets);
    const gyroScaleF32 = new Float32Array(gyro_scale_matrix);

    return (
      (await this.sendAndWait(
        () =>
          FIRMCommandBuilder.build_set_imu_calibration(
            accelOffsetsF32,
            accelScaleF32,
            gyroOffsetsF32,
            gyroScaleF32,
          ),
        (res) => ('SetIMUCalibration' in res ? res.SetIMUCalibration : undefined),
      )) ?? false
    );
  }

  async setMagnetometerCalibration(offsets: number[], scale_matrix: number[]): Promise<boolean> {
    const offsetsF32 = new Float32Array(offsets);
    const scaleF32 = new Float32Array(scale_matrix);

    return (
      (await this.sendAndWait(
        () => FIRMCommandBuilder.build_set_magnetometer_calibration(offsetsF32, scaleF32),
        (res) => ('SetMagnetometerCalibration' in res ? res.SetMagnetometerCalibration : undefined),
      )) ?? false
    );
  }

  /**
   * Runs a full magnetometer calibration sequence and applies it to the device.
   *
   * Mirrors the Rust helper `run_and_apply_magnetometer_calibration`:
   * 1) Collects samples for `collectionDurationMs` while you rotate the device.
   * 2) Fits offsets + soft-iron matrix.
   * 3) Sends `SetMagnetometerCalibration` and returns the device acknowledgement.
   *
   * @returns `true/false` if the device responded, or `null` if calibration failed or timed out.
   */
  async runAndApplyMagnetometerCalibration(
    collectionDurationMs: number,
    applyTimeoutMs = RESPONSE_TIMEOUT_MS,
  ): Promise<boolean | null> {
    if (!this.running) throw new Error('Not connected');
    if (!(collectionDurationMs > 0)) throw new Error('collectionDurationMs must be > 0');

    const calibrator = new MagnetometerCalibrator();
    calibrator.start();

    const unsubscribe = this.onPacket((pkt) => {
      try {
        calibrator.add_sample(pkt as unknown as object);
      } catch {
        // Ignore per-sample errors (should be rare; keeps stream alive)
      }
    });

    await this.sleep(collectionDurationMs);

    unsubscribe();
    calibrator.stop();

    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const result = calibrator.calculate() as any | null;
    if (!result) return null;

    const offsetsF32 = new Float32Array(result.offsets as number[]);
    const scaleF32 = new Float32Array(result.scaleMatrix as number[]);

    return await this.sendAndWait(
      () => FIRMCommandBuilder.build_set_magnetometer_calibration(offsetsF32, scaleF32),
      (res) => ('SetMagnetometerCalibration' in res ? res.SetMagnetometerCalibration : undefined),
      applyTimeoutMs,
    );
  }

  /**
   * Sends a cancel command to the device (e.g., to abort a calibration).
   * @returns True if acknowledged, false if timeout or not acknowledged.
   */
  async sendCancelCommand(): Promise<boolean> {
    return (
      (await this.sendAndWait(
        () => FIRMCommandBuilder.build_cancel(),
        (res) => ('Cancel' in res ? res.Cancel : undefined),
      )) ?? false
    );
  }

  /**
   * Sends a reboot command to the device.
   */
  async reboot(): Promise<void> {
    await this.sendBytes(FIRMCommandBuilder.build_reboot());
  }

  /**
   * Enqueues a newly parsed packet or resolves a pending waiter.
   *
   * @param dataPacket Parsed FIRM data packet.
   */
  private enqueuePacket(dataPacket: FIRMPacket): void {
    this.packetListeners.forEach((fn) => {
      try {
        fn(dataPacket);
      } catch (e) {
        console.warn('[FIRM] packet listener error:', e);
      }
    });
    (this.packetWaiters.shift() || this.packetQueue.push.bind(this.packetQueue))(dataPacket);
  }

  /**
   * Subscribe to parsed data packets.
   *
   * This is a non-consuming "snoop" hook (packets still go into the normal queue).
   */
  private onPacket(listener: (pkt: FIRMPacket) => void): () => void {
    this.packetListeners.push(listener);
    return () => {
      const idx = this.packetListeners.indexOf(listener);
      if (idx !== -1) this.packetListeners.splice(idx, 1);
    };
  }

  /**
   * Enqueues a newly parsed response and notifies waiters.
   *
   * @param response Parsed FIRM response.
   */
  private enqueueResponse(response: FIRMResponse): void {
    this.responseQueue.push(response);
    this.responseWaiters.forEach((waiter) => waiter(response));
  }

  /**
   * Waits for a specific response that matches the predicate.
   *
   * @param matcher Function that returns the desired value if the response matches, or undefined.
   * @param timeoutMs Max time to wait in milliseconds.
   */
  private async waitForResponse<T>(
    matcher: (res: FIRMResponse) => T | undefined,
    timeoutMs: number,
  ): Promise<T> {
    // 1. Check existing queue
    for (let i = 0; i < this.responseQueue.length; i++) {
      const match = matcher(this.responseQueue[i]);
      if (match !== undefined) {
        this.responseQueue.splice(i, 1); // Consume it
        return match;
      }
    }

    // 2. Wait for new responses
    return new Promise<T>((resolve, reject) => {
      const timeoutId = setTimeout(() => {
        cleanup();
        reject(new Error('Timeout waiting for response'));
      }, timeoutMs);

      const onResponse = (res: FIRMResponse) => {
        const match = matcher(res);
        if (match !== undefined) {
          // We found it! Remove from queue.
          const idx = this.responseQueue.indexOf(res);
          if (idx !== -1) {
            this.responseQueue.splice(idx, 1);
          }
          cleanup();
          resolve(match);
        }
      };

      const cleanup = () => {
        clearTimeout(timeoutId);
        const idx = this.responseWaiters.indexOf(onResponse);
        if (idx !== -1) {
          this.responseWaiters.splice(idx, 1);
        }
      };

      this.responseWaiters.push(onResponse);
    });
  }

  /**
   * Resolves all pending waiters with null (used on shutdown/error).
   */
  private flushWaitersWithNull(): void {
    while (this.packetWaiters.length)
      (this.packetWaiters.shift() as (pkt: FIRMPacket | null) => void)(null);
  }

  /**
   * Internal helper that waits for the next packet in FIFO order.
   *
   * @returns Next packet, or null if the stream has ended.
   */
  private async waitForNextPacket(): Promise<FIRMPacket | null> {
    if (this.packetQueue.length > 0) {
      return this.packetQueue.shift()!;
    }

    if (!this.running) {
      return null;
    }

    return new Promise<FIRMPacket | null>((resolve) => {
      this.packetWaiters.push(resolve);
    });
  }

  /**
   * Returns the most recent packet, discarding any older queued packets.
   *
   * If no packets are queued, waits for the next one to arrive.
   * Returns null if the stream has ended and no packets remain.
   *
   * @returns The latest FIRMPacket, or null if no more packets will arrive.
   */
  async getMostRecentDataPacket(): Promise<FIRMPacket | null> {
    if (!this.running && this.packetQueue.length === 0) {
      return null;
    }

    if (this.packetQueue.length > 0) {
      const latest = this.packetQueue[this.packetQueue.length - 1];
      this.packetQueue = [];
      return latest;
    }

    return this.waitForNextPacket();
  }

  /**
   * Async iterator over packets in arrival order.
   *
   * Usage:
   *   for await (const pkt of firm.getDataPackets()) {
   *     console.log(pkt.timestamp_seconds);
   *   }
   *
   * @returns Async generator yielding FIRMPacket objects.
   */
  async *getDataPackets(): AsyncGenerator<FIRMPacket, void> {
    while (true) {
      const pkt = await this.waitForNextPacket();
      if (pkt === null) return;
      yield pkt;
    }
  }

  /**
   * Stops reading from the serial device and clears pending waiters.
   *
   * @returns Resolves when the reader has been cancelled and released.
   */
  async close(): Promise<void> {
    try {
      if (this.closed) return;
      this.closed = true;

      this.running = false;
      await this.safeCall(this.reader, 'cancel');
      await this.safeCall(this.reader, 'releaseLock');
      await this.safeCall(this.writer, 'close');
      await this.safeCall(this.writer, 'releaseLock');
      await this.safeCall(this.port, 'close');
      this.reader = null;
      this.writer = null;
      this.port = null;
      this.flushWaitersWithNull();
    } catch (e) {
      // Swallow any unexpected error so close() never rejects
      // (prevents unhandled promise rejection on reboot/disconnect)
      console.warn('[FIRM] close() failed (swallowed):', e);
      // Re-throw for debug if needed:
      // throw e;
    }
  }
}
