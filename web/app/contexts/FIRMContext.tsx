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

    const MAX_LOG_CHARS = 30000;

    const unsubRx = firm.onRawBytes((bytes) => {
      setReceivedBytes((n) => n + bytes.length);
      setRecentRxHex((prev) => appendHexLog(prev, bytesToHex(bytes), MAX_LOG_CHARS));
    });

    const unsubTx = firm.onOutgoingBytes((bytes) => {
      setSentBytes((n) => n + bytes.length);
      setRecentTxHex((prev) => appendHexLog(prev, bytesToHex(bytes), MAX_LOG_CHARS));
    });

    return () => {
      try {
        unsubRx();
      } catch {}
      try {
        unsubTx();
      } catch {}
    };
  }, [firm]);

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
  };

  return <FIRMContext.Provider value={value}>{children}</FIRMContext.Provider>;
}

export function useFIRM(): FIRMContextValue {
  const ctx = useContext(FIRMContext);
  if (!ctx) throw new Error("useFIRM must be used within a FIRMProvider");
  return ctx;
}
