import React from "react";
import { Cpu, GaugeCircle, Wifi, RefreshCcw } from "lucide-react";
import { useFIRM } from "~/contexts/FIRMContext";

function safeStr(v: unknown): string {
  if (v === null || v === undefined) return "—";
  if (typeof v === "string") return v.trim() || "—";
  if (typeof v === "number" || typeof v === "boolean") return String(v);
  return "—";
}

export function DeviceInfoPanel() {
  const { isConnected, isLoadingDeviceMeta, refreshDeviceMeta, deviceInfo, deviceConfig } =
    useFIRM();

  const items = [
    {
      label: "Device",
      icon: <Cpu className="h-4 w-4 text-theme" />,
      value: safeStr(deviceConfig?.name ?? "Unknown"),
    },
    {
      label: "Firmware",
      icon: <Wifi className="h-4 w-4 text-theme" />,
      value: safeStr(deviceInfo?.firmware_version ?? "Unknown"),
    },
    {
      label: "Frequency",
      icon: <GaugeCircle className="h-4 w-4 text-theme" />,
      value: deviceConfig?.frequency != null ? safeStr(deviceConfig?.frequency) + "Hz" : "Unknown",
    },
    {
      label: "Protocol",
      icon: <Wifi className="h-4 w-4 text-theme" />,
      value: safeStr(deviceConfig?.protocol ?? "Unknown"),
    },
  ];

  return (
    <section className="rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-4 shadow-sm text-slate-900">
      <div className="mb-3 flex items-center justify-between">
        <h2 className="text-lg font-semibold leading-tight">Device Info</h2>

        <button
          type="button"
          onClick={refreshDeviceMeta}
          disabled={!isConnected || isLoadingDeviceMeta}
          className="inline-flex items-center gap-2 rounded-lg border border-slate-300 px-3 py-1.5 text-sm font-medium text-slate-700 hover:bg-slate-50 disabled:opacity-50"
          title="Refresh device info"
        >
          <RefreshCcw className="h-4 w-4" />
          {isLoadingDeviceMeta ? "Refreshing..." : "Refresh"}
        </button>
      </div>

      <ul className="flex flex-wrap items-center gap-x-10 gap-y-3 text-sm leading-relaxed">
        {items.map((item) => (
          <li key={item.label} className="flex items-center gap-2">
            {item.icon}
            <span className="text-slate-500">{item.label}:</span>
            <span className="font-medium">{item.value}</span>
          </li>
        ))}
      </ul>

      {!deviceInfo && isConnected && !isLoadingDeviceMeta && (
        <p className="mt-3 text-sm text-slate-500">
          No device info yet (timeout or not supported). Try &quot;Refresh&quot;.
        </p>
      )}
    </section>
  );
}
