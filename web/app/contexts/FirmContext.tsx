import React, { createContext, useCallback, useContext, useState, type ReactNode } from "react";
import { FIRM as FirmClient } from "firm-client";
import type { DeviceInfo, DeviceConfig } from "firm-client";

export type FirmInstance = FirmClient;

type FirmContextValue = {
  firm: FirmInstance | null;
  isConnected: boolean;
  isConnecting: boolean;
  connect: () => Promise<void>;
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

  const refreshDeviceMeta = useCallback(async () => {
    if (!firm) return;

    setIsLoadingDeviceMeta(true);
    try {
      // These already return null on timeout in your library
      const [info, cfg] = await Promise.all([
        firm.getDeviceInfo(),
        firm.getDeviceConfig(),
      ]);

      setDeviceInfo(info);
      setDeviceConfig(cfg);
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
      // (don’t block UI forever; your library times out to null)
      try {
        const [info, cfg] = await Promise.all([
          instance.getDeviceInfo(),
          instance.getDeviceConfig(),
        ]);
        setDeviceInfo(info);
        setDeviceConfig(cfg);
      } catch {
        // If something unexpected happens, just keep meta null
      }
    } catch (err) {
      console.error("[FIRM] Connection failed:", err);
      alert("Failed to connect to FIRM. Check the device and try again.");
      setIsConnected(false);
      setFirm(null);
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
