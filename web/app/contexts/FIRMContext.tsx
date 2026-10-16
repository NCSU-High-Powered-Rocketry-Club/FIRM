import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { FIRM as FIRMClient } from "firm-client";
import type { DeviceConfig, DeviceInfo, FIRMConnectOptions, FIRMPacket } from "firm-client";

export type FIRMInstance = FIRMClient;

type FIRMContextValue = {
  firm: FIRMInstance | null;
  isConnected: boolean;
  isConnecting: boolean;
  connect: () => Promise<void>;
  disconnect: () => Promise<void>;
  deviceInfo: DeviceInfo | null;
  deviceConfig: DeviceConfig | null;
  isLoadingDeviceMeta: boolean;
  refreshDeviceMeta: () => Promise<void>;

  // Telemetry (dev + charts)
  latestPacket: FIRMPacket | null;
  receivedBytes: number;
  sentBytes: number;
  recentRxHex: string;
  recentTxHex: string;
  packetsPerSecond: number;

  hideDataPackets: boolean;
  setHideDataPackets: (value: boolean) => void;

  pauseByteStream: boolean;
  setPauseByteStream: (value: boolean) => void;
};

const FIRMContext = createContext<FIRMContextValue | undefined>(undefined);

const MAX_HEX_LOG_CHARS = 2000;
// latestPacket is published at most this often; every context consumer re-renders on each update.
const PACKET_PUBLISH_INTERVAL_MS = 50;
const DATA_MESSAGE_ID = 0x01;
const DATA_MESSAGE_LENGTH = 1 + 88;
const RESPONSE_MESSAGE_LENGTHS: Readonly<Record<number, number>> = {
  0x02: 1 + 16, // DeviceInfo_t
  0x03: 1 + 38, // DeviceConfig_t
  0x04: 1 + 1, // SetDeviceConfig acknowledgement
  0x06: 1 + 1, // Mock acknowledgement
  0x07: 1 + 1, // Magnetometer calibration acknowledgement
  0x08: 1 + 1, // IMU calibration acknowledgement
  0x09: 1 + 192, // Four Calibration_t values
  0x0a: 1 + 1, // Cancel acknowledgement
};

type WebSerialPort = NonNullable<FIRMConnectOptions["port"]>;
type NavigatorWithSerial = Navigator & {
  serial: EventTarget & { requestPort(): Promise<WebSerialPort> };
};

function getWebSerial(): NavigatorWithSerial["serial"] {
  return (navigator as NavigatorWithSerial).serial;
}

/**
 * Returns true if the event target is an element the user can type into.
 * Used to prevent global shortcuts from firing while typing.
 */
export function isEditableTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName.toLowerCase();
  return tag === "input" || tag === "textarea" || tag === "select" || target.isContentEditable;
}

function bytesToHex(bytes: Uint8Array): string {
  let out = "";
  for (let i = 0; i < bytes.length; i++) {
    out += bytes[i].toString(16).padStart(2, "0") + (i === bytes.length - 1 ? "" : " ");
  }
  return out;
}

function appendHexLog(prev: string, chunkHex: string, maxChars: number): string {
  if (!chunkHex) return prev;
  const next = prev ? prev + "\n" + chunkHex : chunkHex;
  if (next.length <= maxChars) return next;
  return next.slice(next.length - maxChars);
}

function concatBytes(a: Uint8Array, b: Uint8Array): Uint8Array {
  if (a.length === 0) return b;
  if (b.length === 0) return a;
  const bytes = new Uint8Array(a.length + b.length);
  bytes.set(a, 0);
  bytes.set(b, a.length);
  return bytes;
}

export function FIRMProvider({ children }: { children: ReactNode }) {
  const [firm, setFIRM] = useState<FIRMInstance | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);

  const [deviceInfo, setDeviceInfo] = useState<DeviceInfo | null>(null);
  const [deviceConfig, setDeviceConfig] = useState<DeviceConfig | null>(null);
  const [isLoadingDeviceMeta, setIsLoadingDeviceMeta] = useState(false);

  const [latestPacket, setLatestPacket] = useState<FIRMPacket | null>(null);
  const [receivedBytes, setReceivedBytes] = useState(0);
  const [sentBytes, setSentBytes] = useState(0);
  const [recentRxHex, setRecentRxHex] = useState("");
  const [recentTxHex, setRecentTxHex] = useState("");
  const [packetsPerSecond, setPacketsPerSecond] = useState(0);

  const [hideDataPackets, setHideDataPackets] = useState(false);
  const [pauseByteStream, setPauseByteStream] = useState(false);

  // Refs so async work can tell whether it still belongs to the current connection.
  // `sessionRef` changes on every connect attempt and disconnect; anything that awaited across
  // a change must drop its result.
  const sessionRef = useRef(0);
  const firmRef = useRef<FIRMInstance | null>(null);
  const portRef = useRef<WebSerialPort | null>(null);
  const connectingRef = useRef(false);

  const disconnect = useCallback(async () => {
    sessionRef.current += 1;
    connectingRef.current = false;
    const current = firmRef.current;
    firmRef.current = null;
    portRef.current = null;
    if (current) {
      try {
        await current.close();
      } catch (err) {
        console.warn("FIRM close failed (device may have been unplugged):", err);
      }
    }

    setFIRM(null);
    setIsConnected(false);
    setIsConnecting(false);
    setDeviceInfo(null);
    setDeviceConfig(null);

    setLatestPacket(null);
    setReceivedBytes(0);
    setSentBytes(0);
    setRecentRxHex("");
    setRecentTxHex("");
    setPacketsPerSecond(0);
    setIsLoadingDeviceMeta(false);
  }, []);

  // Close the port if the provider unmounts while connected.
  useEffect(() => {
    return () => {
      sessionRef.current += 1;
      void firmRef.current?.close();
      firmRef.current = null;
    };
  }, []);

  // Auto-reset UI state if our serial device disconnects. The event fires for every port this
  // origin has been granted, so ignore ports other than the connected one.
  useEffect(() => {
    if (!("serial" in navigator)) return;

    const handleNativeDisconnect = (event: Event) => {
      const port = (event as Event & { port?: unknown }).port ?? event.target;
      if (!portRef.current || port !== portRef.current) return;
      console.log("USB serial disconnected:", event);
      void disconnect();
    };

    const serial = getWebSerial();
    if (!serial) return;

    serial.addEventListener("disconnect", handleNativeDisconnect);
    return () => {
      serial.removeEventListener("disconnect", handleNativeDisconnect);
    };
  }, [disconnect]);

  const loadDeviceMeta = useCallback(async (instance: FIRMInstance, session: number) => {
    setIsLoadingDeviceMeta(true);
    try {
      const [info, cfg] = await Promise.all([instance.getDeviceInfo(), instance.getDeviceConfig()]);
      if (sessionRef.current !== session) return;
      setDeviceInfo(info);
      setDeviceConfig(cfg);
    } catch (err) {
      // Best-effort; some devices may not support metadata requests.
      console.error("Failed to load device metadata:", err);
    } finally {
      if (sessionRef.current === session) setIsLoadingDeviceMeta(false);
    }
  }, []);

  const refreshDeviceMeta = useCallback(async () => {
    const instance = firmRef.current;
    if (!instance) return;
    await loadDeviceMeta(instance, sessionRef.current);
  }, [loadDeviceMeta]);

  const connect = useCallback(async () => {
    if (firmRef.current || connectingRef.current) return;

    if (!("serial" in navigator)) {
      alert("Web Serial API is not available in this browser.");
      return;
    }

    sessionRef.current += 1;
    const session = sessionRef.current;
    connectingRef.current = true;

    try {
      setIsConnecting(true);

      setLatestPacket(null);
      setReceivedBytes(0);
      setSentBytes(0);
      setRecentRxHex("");
      setRecentTxHex("");
      setPacketsPerSecond(0);
      setHideDataPackets(false);
      setPauseByteStream(false);

      const port = await getWebSerial().requestPort();
      if (sessionRef.current !== session) return;
      const instance = await FIRMClient.connect({ baudRate: 115200, port });
      if (sessionRef.current !== session) {
        await instance.close();
        return;
      }
      firmRef.current = instance;
      portRef.current = port;
      connectingRef.current = false;
      setFIRM(instance);
      setIsConnected(true);
      setIsConnecting(false);

      await loadDeviceMeta(instance, session);
    } catch (err) {
      if (sessionRef.current !== session) return;
      // The user dismissed the port picker; nothing failed.
      if (err instanceof DOMException && err.name === "NotFoundError") return;

      console.error("[FIRM] Connection failed:", err);
      alert("Failed to connect to FIRM. Check the device and try again.");

      setFIRM(null);
      setIsConnected(false);
      setDeviceInfo(null);
      setDeviceConfig(null);

      setLatestPacket(null);
      setReceivedBytes(0);
      setSentBytes(0);
      setRecentRxHex("");
      setRecentTxHex("");
      setPacketsPerSecond(0);
    } finally {
      if (sessionRef.current === session) {
        connectingRef.current = false;
        setIsConnecting(false);
      }
    }
  }, [loadDeviceMeta]);

  // Track RX/TX byte counts and hex logs.
  useEffect(() => {
    if (!firm) return;

    let buffer: Uint8Array<ArrayBufferLike> = new Uint8Array(0);

    const stripDataMessages = (chunk: Uint8Array): Uint8Array => {
      if (!hideDataPackets) {
        buffer = new Uint8Array(0);
        return chunk;
      }

      buffer = concatBytes(buffer, chunk);

      const out: number[] = [];

      // Messages have no header/length/CRC: the leading ID determines a fixed total size.
      let i = 0;
      while (i < buffer.length) {
        const id = buffer[i];
        const messageLen =
          id === DATA_MESSAGE_ID ? DATA_MESSAGE_LENGTH : RESPONSE_MESSAGE_LENGTHS[id];
        if (messageLen !== undefined) {
          if (i + messageLen > buffer.length) break;
          if (id === DATA_MESSAGE_ID) {
            i += messageLen;
            continue;
          }
          for (let j = 0; j < messageLen; j++) out.push(buffer[i + j]);
          i += messageLen;
          continue;
        }

        // Unknown byte: preserve it for diagnostics and continue searching for an ID.
        out.push(buffer[i]);
        i += 1;
      }

      buffer = buffer.slice(i);
      return new Uint8Array(out);
    };

    const unsubRx = firm.onRawBytes((bytes) => {
      // Allow connection-level bytes counter to keep increasing even while paused.
      setReceivedBytes((n) => n + bytes.length);

      // Pause affects RX display only.
      if (pauseByteStream) return;

      const forwarded = stripDataMessages(bytes);

      if (forwarded.length > 0) {
        setRecentRxHex((prev) => appendHexLog(prev, bytesToHex(forwarded), MAX_HEX_LOG_CHARS));
      }
    });

    const unsubTx = firm.onOutgoingBytes((bytes) => {
      setSentBytes((n) => n + bytes.length);
      setRecentTxHex((prev) => appendHexLog(prev, bytesToHex(bytes), MAX_HEX_LOG_CHARS));
    });

    return () => {
      unsubRx();
      unsubTx();
    };
  }, [firm, hideDataPackets, pauseByteStream]);

  // Own the packet stream: update latestPacket on each parsed packet.
  useEffect(() => {
    if (!firm) return;

    let cancelled = false;
    let count = 0;
    let lastRateTs = performance.now();
    let pendingPacket: FIRMPacket | null = null;

    const updateRate = () => {
      const now = performance.now();
      const elapsedMs = now - lastRateTs;
      if (elapsedMs >= 1000) {
        setPacketsPerSecond(Math.round((count * 1000) / elapsedMs));
        count = 0;
        lastRateTs = now;
      }
    };

    (async () => {
      try {
        for await (const pkt of firm.getDataPackets()) {
          if (cancelled) return;
          pendingPacket = pkt;
          count += 1;
          updateRate();
        }
      } catch {
        // ignore
      }
    })();

    const rateTimer = window.setInterval(updateRate, 250);
    const publishTimer = window.setInterval(() => {
      if (pendingPacket === null) return;
      setLatestPacket(pendingPacket);
      pendingPacket = null;
    }, PACKET_PUBLISH_INTERVAL_MS);

    return () => {
      cancelled = true;
      window.clearInterval(rateTimer);
      window.clearInterval(publishTimer);
      setPacketsPerSecond(0);
    };
  }, [firm]);

  const value: FIRMContextValue = {
    firm,
    isConnected,
    isConnecting,
    connect,
    disconnect,
    deviceInfo,
    deviceConfig,
    isLoadingDeviceMeta,
    refreshDeviceMeta,

    latestPacket,
    receivedBytes,
    sentBytes,
    recentRxHex,
    recentTxHex,
    packetsPerSecond,

    hideDataPackets,
    setHideDataPackets,
    pauseByteStream,
    setPauseByteStream,
  };

  return <FIRMContext.Provider value={value}>{children}</FIRMContext.Provider>;
}

export function useFIRM(): FIRMContextValue {
  const ctx = useContext(FIRMContext);
  if (!ctx) throw new Error("useFIRM must be used within a FIRMProvider");
  return ctx;
}
