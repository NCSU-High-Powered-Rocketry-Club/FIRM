// src/firm/FirmContext.tsx
import React, { createContext, useCallback, useContext, useState, type ReactNode } from "react";
import { FIRM as FirmClient } from "firm-client";

export type FirmInstance = FirmClient;

type FirmContextValue = {
  firm: FirmInstance | null;
  isConnected: boolean;
  isConnecting: boolean;
  connect: () => Promise<void>;
};

const FirmContext = createContext<FirmContextValue | undefined>(undefined);

export function FirmProvider({ children }: { children: ReactNode }) {
  const [firm, setFirm] = useState<FirmInstance | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);

  const connect = useCallback(async () => {
    if (isConnected || isConnecting) {
      return;
    }

    if (!("serial" in navigator)) {
      alert("Web Serial API is not available in this browser.");
      return;
    }

    try {
      setIsConnecting(true);
      const instance = await FirmClient.connect({ baudRate: 115200 });
      setFirm(instance);
      setIsConnected(true);
      console.log("[FIRM] ✅ Connected");

      // Simple debug: print packets to console
      (async () => {
        for await (const pkt of instance.getDataPackets()) {
          console.log("[FIRM] Packet:", pkt);
        }
      })();
    } catch (err) {
      console.error("[FIRM] Connection failed:", err);
      alert("Failed to connect to FIRM. Check the device and try again.");
      setIsConnected(false);
    } finally {
      setIsConnecting(false);
    }
  }, [isConnected, isConnecting]);

  const value: FirmContextValue = {
    firm,
    isConnected,
    isConnecting,
    connect,
  };

  return <FirmContext.Provider value={value}>{children}</FirmContext.Provider>;
}

export function useFirm(): FirmContextValue {
  const ctx = useContext(FirmContext);
  if (!ctx) {
    throw new Error("useFirm must be used within a FirmProvider");
  }
  return ctx;
}
