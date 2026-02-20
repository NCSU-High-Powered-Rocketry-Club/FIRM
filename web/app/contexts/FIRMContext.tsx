import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useState,
  type ReactNode,
} from "react";
import { FIRM as FIRMClient } from "firm-client";
import type { DeviceConfig, DeviceInfo, FIRMPacket } from "firm-client";

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

  const disconnect = useCallback(async () => {
    if (firm) {
      try {
        await firm.close();
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
  }, [firm]);

  // Auto-reset UI state if the serial device disconnects.
  useEffect(() => {
    if (!("serial" in navigator)) return;

    const handleNativeDisconnect = (event: Event) => {
      console.log("USB serial disconnected:", event);
      disconnect();
    };

    const serial = (navigator as unknown as { serial: EventTarget }).serial;
    if (!serial) return;

    serial.addEventListener("disconnect", handleNativeDisconnect);
    return () => {
      serial.removeEventListener("disconnect", handleNativeDisconnect);
    };
  }, [disconnect]);

  const refreshDeviceMeta = useCallback(async () => {
    if (!firm) return;

    setIsLoadingDeviceMeta(true);
    try {
      const [info, cfg] = await Promise.all([firm.getDeviceInfo(), firm.getDeviceConfig()]);
      setDeviceInfo(info);
      setDeviceConfig(cfg);
    } catch (err) {
      console.error("Failed to refresh device metadata:", err);
    } finally {
      setIsLoadingDeviceMeta(false);
    }
  }, [firm]);

  const connect = useCallback(async () => {
    if (isConnected || isConnecting) return;

    if (!("serial" in navigator)) {
      alert("Web Serial API is not available in this browser.");
      return;
    }

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

      const instance = await FIRMClient.connect({ baudRate: 115200 });
      setFIRM(instance);
      setIsConnected(true);

      try {
        const [info, cfg] = await Promise.all([instance.getDeviceInfo(), instance.getDeviceConfig()]);
        setDeviceInfo(info);
        setDeviceConfig(cfg);
      } catch {
        // Best-effort; some devices may not support metadata requests.
      }
    } catch (err) {
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
      setIsConnecting(false);
    }
  }, [isConnected, isConnecting]);

  // Track RX/TX byte counts and hex logs.
  useEffect(() => {
    if (!firm) return;

    const MAX_LOG_CHARS = 2000;

    // Streaming filter state (kept in closure for chunk-boundary safety)
    const DATA_HEADER = 0xa55a;
    const RESPONSE_HEADER = 0x5aa5;
    const HEADER_LEN = 2 + 2 + 4; // header + identifier + length
    const CRC_LEN = 2;

    let buffer = new Uint8Array(0);

    const readU16LE = (b: Uint8Array, off: number) => b[off] | (b[off + 1] << 8);
    const readU32LE = (b: Uint8Array, off: number) =>
      (b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24)) >>> 0;

    const concat = (a: Uint8Array, b: Uint8Array) => {
      if (a.length === 0) return b;
      if (b.length === 0) return a;
      const out = new Uint8Array(a.length + b.length);
      out.set(a, 0);
      out.set(b, a.length);
      return out;
    };

    const stripDataFrames = (chunk: Uint8Array): { forwarded: Uint8Array; strippedBytes: number } => {
      // Fast path: filter disabled
      if (!hideDataPackets) {
        buffer = new Uint8Array(0);
        return { forwarded: chunk, strippedBytes: 0 };
      }

      buffer = concat(buffer, chunk);

      const out: number[] = [];
      let stripped = 0;

      // We scan for known headers; when we don't have enough bytes to decide, we keep remainder in buffer
      let i = 0;
      while (i < buffer.length) {
        // Need at least 2 bytes to check a header
        if (i + 2 > buffer.length) break;

        const hdr = readU16LE(buffer, i);

        // If this looks like a frame header, make sure we have full frame before acting
        if (hdr === DATA_HEADER || hdr === RESPONSE_HEADER) {
          if (i + HEADER_LEN > buffer.length) break; // wait for more

          const payloadLen = readU32LE(buffer, i + 4);
          const frameLen = HEADER_LEN + payloadLen + CRC_LEN;

          if (payloadLen > 10_000_000) {
            // Probably a false-positive header in random data; treat as plain bytes.
            out.push(buffer[i]);
            i += 1;
            continue;
          }

          if (i + frameLen > buffer.length) break; // wait for more

          if (hdr === DATA_HEADER) {
            // Drop full data frame
            stripped += frameLen;
            i += frameLen;
            continue;
          }

          // Response frame: keep
          for (let j = 0; j < frameLen; j++) out.push(buffer[i + j]);
          i += frameLen;
          continue;
        }

        // Not a recognized header => keep byte
        out.push(buffer[i]);
        i += 1;
      }

      // Keep remainder for next chunk
      buffer = buffer.slice(i);

      return { forwarded: new Uint8Array(out), strippedBytes: stripped };
    };

    const unsubRx = firm.onRawBytes((bytes) => {
      // Allow connection-level bytes counter to keep increasing even while paused.
      setReceivedBytes((n) => n + bytes.length);

      // Pause affects RX display only.
      if (pauseByteStream) return;

      const { forwarded } = stripDataFrames(bytes);

      if (forwarded.length > 0) {
        setRecentRxHex((prev) => appendHexLog(prev, bytesToHex(forwarded), MAX_LOG_CHARS));
      }
    });

    const unsubTx = firm.onOutgoingBytes((bytes) => {
      setSentBytes((n) => n + bytes.length);
      setRecentTxHex((prev) => appendHexLog(prev, bytesToHex(bytes), MAX_LOG_CHARS));
    });

    return () => {
      try {
        unsubRx();
      } catch {
        // best-effort cleanup; ignore
      }
      try {
        unsubTx();
      } catch {
        // best-effort cleanup; ignore
      }
    };
  }, [firm, hideDataPackets, pauseByteStream]);

  // Own the packet stream: update latestPacket on each parsed packet.
  useEffect(() => {
    if (!firm) return;

    let cancelled = false;
    let count = 0;
    let lastRateTs = performance.now();

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
          setLatestPacket(pkt);
          count += 1;
          updateRate();
        }
      } catch {
        // ignore
      }
    })();

    const rateTimer = window.setInterval(updateRate, 250);

    return () => {
      cancelled = true;
      window.clearInterval(rateTimer);
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
