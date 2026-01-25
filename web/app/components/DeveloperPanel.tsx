import React, { useEffect, useMemo, useRef, useState } from "react";
import { useFirm } from "~/contexts/FirmContext";
import type { FIRMPacket } from "firm-client";

function prettyBytes(n: number): string {
  const units = ["B", "KB", "MB", "GB"];
  let i = 0;
  let value = n;
  while (value >= 1024 && i < units.length - 1) {
    value /= 1024;
    i += 1;
  }
  return `${i === 0 ? value : value.toFixed(2)} ${units[i]}`;
}

export function DeveloperPanel({ visible }: { visible: boolean }) {
  const {
    receivedBytes,
    sentBytes,
    recentRxHex,
    recentTxHex,
    latestPacket,
    packetsPerSecond,
  } = useFirm();

  const rxRef = useRef<HTMLTextAreaElement | null>(null);
  const txRef = useRef<HTMLTextAreaElement | null>(null);

  // Keep a ref to the latest packet so our interval doesn't have to restart constantly.
  const latestPacketRef = useRef<FIRMPacket | null>(null);
  useEffect(() => {
    latestPacketRef.current = latestPacket;
  }, [latestPacket]);

  // Throttle packet rendering so we don't re-render at full packet rate.
  const [uiPacket, setUiPacket] = useState<FIRMPacket | null>(null);
  useEffect(() => {
    if (!visible) return;

    const id = window.setInterval(() => {
      setUiPacket(latestPacketRef.current);
    }, 100); // target 10Hz

    return () => {
      window.clearInterval(id);
    };
  }, [visible]);

  // Auto-scroll hex logs to bottom when they change.
  useEffect(() => {
    if (!visible) return;
    const el = rxRef.current;
    if (!el) return;
    el.scrollTop = el.scrollHeight;
  }, [visible, recentRxHex]);

  useEffect(() => {
    if (!visible) return;
    const el = txRef.current;
    if (!el) return;
    el.scrollTop = el.scrollHeight;
  }, [visible, recentTxHex]);

  const packetText = useMemo(() => {
    if (!uiPacket) return "No packets received yet.";
    // If you want the lightweight debug string, swap this line:
    // return `Packet: ${uiPacket.timestamp_seconds}`;
    try {
      return JSON.stringify(uiPacket, null, 2);
    } catch {
      return String(uiPacket);
    }
  }, [uiPacket]);

  if (!visible) return null;

  return (
    <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-4 shadow-sm text-slate-900">
      <div className="mb-3 flex items-center justify-between gap-3">
        <div>
          <h2 className="text-lg font-semibold leading-tight">Developer Panel</h2>
          <div className="mt-0.5 text-xs text-slate-500">
            Packets/sec: <span className="font-mono">{packetsPerSecond}</span>
          </div>
        </div>
        <div className="text-xs text-slate-500">
          <span className="rounded-md border border-slate-300 bg-slate-50 px-2 py-1">
            Press <span className="font-mono">d</span> to toggle
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 gap-3 md:grid-cols-2">
        {/* RX column */}
        <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">RX</div>
          <div className="mt-1 text-sm font-medium text-slate-900">
            {receivedBytes.toLocaleString()} bytes{" "}
            <span className="text-slate-500">({prettyBytes(receivedBytes)})</span>
          </div>

          <div className="mt-2">
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-500">
              Recent RX bytes (hex)
            </div>
            <textarea
              ref={rxRef}
              readOnly
              value={recentRxHex}
              className="h-40 w-full resize-none overflow-auto rounded-lg border border-slate-200 bg-slate-950 p-3 font-mono text-xs text-slate-100"
              spellCheck={false}
            />
          </div>
        </div>

        {/* TX column */}
        <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">TX</div>
          <div className="mt-1 text-sm font-medium text-slate-900">
            {sentBytes.toLocaleString()} bytes{" "}
            <span className="text-slate-500">({prettyBytes(sentBytes)})</span>
          </div>

          <div className="mt-2">
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-500">
              Recent TX bytes (hex)
            </div>
            <textarea
              ref={txRef}
              readOnly
              value={recentTxHex}
              className="h-40 w-full resize-none overflow-auto rounded-lg border border-slate-200 bg-slate-950 p-3 font-mono text-xs text-slate-100"
              spellCheck={false}
            />
          </div>
        </div>
      </div>

      {/* Packet at bottom */}
      <div className="mt-3">
        <div className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-500">
          Latest FIRMPacket
        </div>
        <pre className="max-h-[28rem] overflow-auto rounded-lg border border-slate-200 bg-slate-950 p-3 text-xs text-slate-100 whitespace-pre">
          {packetText}
        </pre>
      </div>
    </section>
  );
}
