import React, { createContext, useCallback, useContext, useState, useEffect, type ReactNode } from "react";
import { FIRM as FirmClient } from "firm-client";
import type { DeviceInfo, DeviceConfig } from "firm-client";

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
};

const FirmContext = createContext<FirmContextValue | undefined>(undefined);

export function FirmProvider({ children }: { children: ReactNode }) {
  const [firm, setFirm] = useState<FirmInstance | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);

  const [deviceInfo, setDeviceInfo] = useState<DeviceInfo | null>(null);
  const [deviceConfig, setDeviceConfig] = useState<DeviceConfig | null>(null);
  const [isLoadingDeviceMeta, setIsLoadingDeviceMeta] = useState(false);

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
  }, [firm]);

  // --- 2. Listen for Physical Disconnects (The Fix) ---
  useEffect(() => {
    if (!("serial" in navigator)) return;

    const handleNativeDisconnect = (event: Event) => {
      console.log("USB Device disconnected physically:", event);
      // We assume if *any* serial disconnect happens while we are connected,
      // we should reset our app state.
      disconnect();
    };

    const serial = (navigator as any).serial;
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
      const [info, cfg] = await Promise.all([
        firm.getDeviceInfo(),
        firm.getDeviceConfig(),
      ]);

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
    } finally {
      setIsConnecting(false);
    }
  }, [isConnected, isConnecting]);

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
  };

  return <FirmContext.Provider value={value}>{children}</FirmContext.Provider>;
}

export function useFirm(): FirmContextValue {
  const ctx = useContext(FirmContext);
  if (!ctx) throw new Error("useFirm must be used within a FirmProvider");
  return ctx;
}
