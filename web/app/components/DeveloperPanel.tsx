import React, { useEffect, useMemo, useRef, useState } from "react";
import { useFIRM } from "~/contexts/FIRMContext";
import type { FIRMPacket } from "firm-client";

const RX_IDENTIFIERS = new Set(["01", "02", "03", "04", "06", "07", "08", "09", "0a"]);
const TX_IDENTIFIERS = new Set([
  "02",
  "03",
  "04",
  "05",
  "06",
  "07",
  "08",
  "09",
  "0a",
  "41",
  "42",
  "48",
  "49",
  "4d",
]);

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

function renderHexWithIdHighlight(hex: string, identifiers: ReadonlySet<string>) {
  // Highlights known one-byte message identifiers.
  // hex is formatted as "aa bb cc" with optional newlines between chunks.
  const tokens = hex.split(/\s+/).filter(Boolean);
  const nodes: React.ReactNode[] = [];

  for (let i = 0; i < tokens.length; i++) {
    const t = tokens[i];
    if (identifiers.has(t.toLowerCase())) {
      nodes.push(
        <span
          key={`h-${i}`}
          style={{ backgroundColor: "color-mix(in srgb, var(--color-theme) 12%, transparent)" }}
          className="rounded px-0.5"
        >
          {t}
        </span>,
      );
    } else {
      nodes.push(<span key={`b-${i}`}>{t}</span>);
    }

    if (i < tokens.length - 1) nodes.push(" ");
  }

  return nodes;
}

function highlightRx(hex: string) {
  return renderHexWithIdHighlight(hex, RX_IDENTIFIERS);
}

function highlightTx(hex: string) {
  return renderHexWithIdHighlight(hex, TX_IDENTIFIERS);
}

export function DeveloperPanel({ visible }: { visible: boolean }) {
  const {
    receivedBytes,
    sentBytes,
    recentRxHex,
    recentTxHex,
    latestPacket,
    packetsPerSecond,
    hideDataPackets,
    setHideDataPackets,
    pauseByteStream,
    setPauseByteStream,
  } = useFIRM();

  const rxRef = useRef<HTMLPreElement | null>(null);
  const txRef = useRef<HTMLPreElement | null>(null);

  const [showPacketViewer, setShowPacketViewer] = useState(true);
  const [packetText, setPacketText] = useState("Waiting for packets…");

  const didInitialScrollRef = useRef(false);

  const rxHighlighted = useMemo(() => highlightRx(recentRxHex), [recentRxHex]);
  const txHighlighted = useMemo(() => highlightTx(recentTxHex), [recentTxHex]);

  // Scroll RX/TX logs to bottom once when the panel is opened.
  useEffect(() => {
    if (!visible) {
      didInitialScrollRef.current = false;
      return;
    }

    if (didInitialScrollRef.current) return;

    const id = window.setTimeout(() => {
      if (rxRef.current) rxRef.current.scrollTop = rxRef.current.scrollHeight;
      if (txRef.current) txRef.current.scrollTop = txRef.current.scrollHeight;
      didInitialScrollRef.current = true;
    }, 0);

    return () => window.clearTimeout(id);
  }, [visible]);

  const latestPacketRef = useRef<FIRMPacket | null>(null);
  useEffect(() => {
    latestPacketRef.current = latestPacket;
  }, [latestPacket]);

  useEffect(() => {
    if (!visible || !showPacketViewer) return;

    const UPDATE_MS = 100;
    const MAX_CHARS = 2000;

    const id = window.setInterval(() => {
      const pkt = latestPacketRef.current;
      if (!pkt) {
        setPacketText("No packets received yet.");
        return;
      }

      const text = JSON.stringify(pkt, null, 2);
      setPacketText(text.length > MAX_CHARS ? text.slice(0, MAX_CHARS) + "\n…(truncated)" : text);
    }, UPDATE_MS);

    return () => window.clearInterval(id);
  }, [visible, showPacketViewer]);

  if (!visible) return null;

  const hexBoxClassName =
    "h-40 w-full resize-none overflow-auto rounded-lg border border-slate-200 bg-slate-50 p-3 font-mono text-xs text-slate-800 shadow-inner focus:outline-none";

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
        <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">
            Received Bytes
          </div>
          <div className="mt-1 text-sm font-medium text-slate-900">
            {receivedBytes.toLocaleString()} bytes{" "}
            <span className="text-slate-500">({prettyBytes(receivedBytes)})</span>
          </div>

          <div className="mt-2 flex flex-wrap items-center gap-3">
            <label className="flex items-center gap-2 text-xs text-slate-600 select-none">
              <input
                type="checkbox"
                className="h-4 w-4 accent-[var(--color-theme)]"
                checked={hideDataPackets}
                onChange={(e) => setHideDataPackets(e.target.checked)}
              />
              Hide Data Packets
            </label>

            <button
              type="button"
              className="rounded-md border border-slate-300 bg-white px-2.5 py-1 text-xs text-slate-700 shadow-sm hover:bg-slate-50"
              onClick={() => setPauseByteStream(!pauseByteStream)}
            >
              {pauseByteStream ? "Play" : "Pause"}
            </button>
          </div>

          <div className="mt-2">
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-500">
              Recent received bytes (hex)
            </div>
            <pre ref={rxRef} className={hexBoxClassName + " whitespace-pre-wrap"}>
              {rxHighlighted}
            </pre>
          </div>
        </div>

        <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">
            Transmitted Bytes
          </div>
          <div className="mt-1 text-sm font-medium text-slate-900">
            {sentBytes.toLocaleString()} bytes{" "}
            <span className="text-slate-500">({prettyBytes(sentBytes)})</span>
          </div>

          <div className="mt-2">
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-500">
              Recent transmitted bytes (hex)
            </div>
            <pre ref={txRef} className={hexBoxClassName + " whitespace-pre-wrap"}>
              {txHighlighted}
            </pre>
          </div>
        </div>
      </div>

      <div className="mt-3">
        <div className="mb-2 flex items-center justify-between gap-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">
            Latest FIRMPacket
          </div>
          <label className="flex items-center gap-2 text-xs text-slate-600 select-none">
            <input
              type="checkbox"
              className="h-4 w-4 accent-[var(--color-theme)]"
              checked={showPacketViewer}
              onChange={(e) => setShowPacketViewer(e.target.checked)}
            />
            Enable packet JSON viewer
          </label>
        </div>

        {showPacketViewer ? (
          <pre className="max-h-[28rem] overflow-auto rounded-lg border border-slate-200 bg-slate-50 p-3 text-xs text-slate-800 whitespace-pre shadow-inner">
            {packetText}
          </pre>
        ) : (
          <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 text-xs text-slate-600">
            Disabled to keep the app responsive in dev. Toggle it on if you need to inspect packets.
          </div>
        )}
      </div>
    </section>
  );
}
