import React, {
  createContext,
  useCallback,
  useContext,
  useState,
  useEffect,
  type ReactNode,
} from "react";
import { FIRM as FirmClient } from "firm-client";
import type { DeviceInfo, DeviceConfig, FIRMPacket } from "firm-client";

export type FirmInstance = FirmClient;

type FirmContextValue = {
  firm: FirmInstance | null;
  isConnected: boolean;
  isConnecting: boolean;
  connect: () => Promise<void>;
  disconnect: () => Promise<void>; // Added disconnect function
  deviceInfo: DeviceInfo | null;
  deviceConfig: DeviceConfig | null;
  isLoadingDeviceMeta: boolean;
  refreshDeviceMeta: () => Promise<void>;

  // Dev telemetry
  latestPacket: FIRMPacket | null;
  receivedBytes: number;
  sentBytes: number;
  recentRxHex: string;
  recentTxHex: string;
  packetsPerSecond: number;
};

const FirmContext = createContext<FirmContextValue | undefined>(undefined);

// Used by global keyboard shortcuts to avoid triggering while typing
export function isEditableTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName.toLowerCase();
  if (tag === "input" || tag === "textarea" || tag === "select") return true;
  if (target.isContentEditable) return true;
  return false;
}

function bytesToHex(bytes: Uint8Array): string {
  let out = "";
  for (let i = 0; i < bytes.length; i++) {
    const b = bytes[i];
    out += b.toString(16).padStart(2, "0") + (i === bytes.length - 1 ? "" : " ");
  }
  return out;
}

function appendHexLog(prev: string, chunkHex: string, maxChars: number): string {
  if (!chunkHex) return prev;
  const next = prev ? prev + "\n" + chunkHex : chunkHex;
  if (next.length <= maxChars) return next;
  // Trim from the start to keep tail (most recent) visible
  return next.slice(next.length - maxChars);
}

export function FirmProvider({ children }: { children: ReactNode }) {
  const [firm, setFirm] = useState<FirmInstance | null>(null);
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
    // If there is an active instance, try to close it gently
    if (firm) {
      try {
        // This might throw if the device is already gone (unplugged), so we catch it
        await firm.close();
      } catch (err) {
        console.warn("Error closing firm instance (device likely unplugged):", err);
      }
    }

    // Reset all state
    setFirm(null);
    setIsConnected(false);
    setIsConnecting(false);
    setDeviceInfo(null);
    setDeviceConfig(null);

    setLatestPacket(null);
    setReceivedBytes(0);
    setSentBytes(0);
    setRecentRxHex("");
    setRecentTxHex("");
  }, [firm]);

  useEffect(() => {
    if (!("serial" in navigator)) return;

    const handleNativeDisconnect = (event: Event) => {
      console.log("USB Device disconnected physically:", event);
      // We assume if *any* serial disconnect happens while we are connected,
      // we should reset our app state.
      disconnect();
    };

    // For some reason I can't get TS to recognize the serial API, so we cast it here.
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
      console.error("Failed to refresh meta:", err);
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

      // reset telemetry on connect attempt
      setLatestPacket(null);
      setReceivedBytes(0);
      setSentBytes(0);
      setRecentRxHex("");
      setRecentTxHex("");

      const instance = await FirmClient.connect({ baudRate: 115200 });
      setFirm(instance);
      setIsConnected(true);

      // Fetch device meta right after connect
      try {
        const [info, cfg] = await Promise.all([
          instance.getDeviceInfo(),
          instance.getDeviceConfig(),
        ]);
        setDeviceInfo(info);
        setDeviceConfig(cfg);
      } catch {
        // If meta fetch fails, just keep going
      }
    } catch (err) {
      console.error("[FIRM] Connection failed:", err);
      alert("Failed to connect to FIRM. Check the device and try again.");
      // Ensure clean state if connection threw halfway
      setFirm(null);
      setIsConnected(false);
      setDeviceInfo(null);
      setDeviceConfig(null);

      setLatestPacket(null);
      setReceivedBytes(0);
      setSentBytes(0);
      setRecentRxHex("");
      setRecentTxHex("");
    } finally {
      setIsConnecting(false);
    }
  }, [isConnected, isConnecting]);

  // Telemetry: byte counters + hex logs
  useEffect(() => {
    if (!firm) return;

    // keep last ~30k chars per log (fits nicely in a scroll box)
    const MAX_LOG_CHARS = 30000;

    const unsubRx = firm.onRawBytes((bytes) => {
      setReceivedBytes((n) => n + bytes.length);
      const hex = bytesToHex(bytes);
      setRecentRxHex((prev) => appendHexLog(prev, hex, MAX_LOG_CHARS));
    });

    const unsubTx = firm.onOutgoingBytes((bytes) => {
      setSentBytes((n) => n + bytes.length);
      const hex = bytesToHex(bytes);
      setRecentTxHex((prev) => appendHexLog(prev, hex, MAX_LOG_CHARS));
    });

    return () => {
      try {
        unsubRx();
      } catch {
        // ignore
      }
      try {
        unsubTx();
      } catch {
        // ignore
      }
    };
  }, [firm]);

  // Telemetry: latest packet (FirmContext owns the packet stream)
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
        // ignore; disconnect handler will reset state
      }
    })();

    const rateTimer = window.setInterval(() => {
      // keep rate from staying stale if packets stop
      updateRate();
    }, 250);

    return () => {
      cancelled = true;
      window.clearInterval(rateTimer);
      setPacketsPerSecond(0);
    };
  }, [firm]);

  const value: FirmContextValue = {
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

  return <FirmContext.Provider value={value}>{children}</FirmContext.Provider>;
}

export function useFirm(): FirmContextValue {
  const ctx = useContext(FirmContext);
  if (!ctx) throw new Error("useFirm must be used within a FirmProvider");
  return ctx;
}
