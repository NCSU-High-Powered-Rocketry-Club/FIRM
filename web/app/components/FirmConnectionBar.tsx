// src/components/FirmConnectionBar.tsx
import React from "react";
import {useFirm} from "~/contexts/FirmContext";

export function FirmConnectionBar() {
  const { isConnected, isConnecting, connect } = useFirm();

  return (
    <div className="mx-auto max-w-5xl w-full px-6 pt-6 flex items-center gap-3 text-sm text-gray-600">
      <span
        className={
          isConnected
            ? "inline-flex items-center gap-1 rounded-full bg-green-100 px-3 py-1 text-xs font-medium text-green-800"
            : "inline-flex items-center gap-1 rounded-full bg-gray-200 px-3 py-1 text-xs font-medium text-gray-700"
        }
      >
        <span
          className={
            isConnected
              ? "h-2 w-2 rounded-full bg-green-500"
              : "h-2 w-2 rounded-full bg-gray-500"
          }
        />
        {isConnected ? "FIRM connected" : "No device connected"}
      </span>

      <button
        type="button"
        onClick={connect}
        disabled={isConnecting || isConnected}
        className={`inline-flex items-center rounded-full border px-3 py-1 text-xs font-medium ${
          isConnected
            ? "border-green-300 text-green-700 bg-green-50 cursor-default"
            : isConnecting
              ? "border-gray-300 text-gray-500 bg-gray-100 cursor-wait"
              : "border-gray-300 text-gray-700 hover:bg-gray-100"
        }`}
      >
        {isConnected
          ? "Connected"
          : isConnecting
            ? "Connecting..."
            : "Connect to FIRM"}
      </button>
    </div>
  );
}
