import React, { useEffect, useMemo, useRef, useState } from "react";
import { Download, Loader2, Save, Upload } from "lucide-react";
import { useFIRM } from "~/contexts/FIRMContext";
import type { FIRMPacket } from "firm-client";

// JSON shape for calibration import/export.
type CalibrationJson = {
  offsets: number[];
  scaleMatrix: number[];
};

type CalibrationDraft = {
  offsets: [string, string, string];
  scale: [[string, string, string], [string, string, string], [string, string, string]];
};

type IMUCalibrationDraft = {
  accelerometer: CalibrationDraft;
  gyroscope: CalibrationDraft;
};

function makeDefaultDraft(): CalibrationDraft {
  return {
    offsets: ["0", "0", "0"],
    scale: [
      ["1", "0", "0"],
      ["0", "1", "0"],
      ["0", "0", "1"],
    ],
  };
}

function makeDefaultImuDraft(): IMUCalibrationDraft {
  return {
    accelerometer: makeDefaultDraft(),
    gyroscope: makeDefaultDraft(),
  };
}

function parseFiniteNumber(input: string): number | null {
  const trimmed = input.trim();
  if (!trimmed) return null;
  const n = Number(trimmed);
  if (!Number.isFinite(n)) return null;
  return n;
}

function draftToNumbers(draft: CalibrationDraft): { offsets: number[]; scaleMatrix: number[] } | null {
  const offsets: number[] = [];
  for (const v of draft.offsets) {
    const n = parseFiniteNumber(v);
    if (n === null) return null;
    offsets.push(n);
  }

  const scaleMatrix: number[] = [];
  for (const row of draft.scale) {
    for (const v of row) {
      const n = parseFiniteNumber(v);
      if (n === null) return null;
      scaleMatrix.push(n);
    }
  }

  return { offsets, scaleMatrix };
}

function numbersToDraft(offsets: number[], scaleMatrix: number[]): CalibrationDraft {
  const off: [string, string, string] = [
    String(offsets[0] ?? 0),
    String(offsets[1] ?? 0),
    String(offsets[2] ?? 0),
  ];

  const s = [...scaleMatrix];
  while (s.length < 9) s.push(0);

  const scale: CalibrationDraft["scale"] = [
    [String(s[0] ?? 1), String(s[1] ?? 0), String(s[2] ?? 0)],
    [String(s[3] ?? 0), String(s[4] ?? 1), String(s[5] ?? 0)],
    [String(s[6] ?? 0), String(s[7] ?? 0), String(s[8] ?? 1)],
  ];

  return { offsets: off, scale };
}

function downloadJson(filename: string, obj: unknown) {
  const blob = new Blob([JSON.stringify(obj, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  URL.revokeObjectURL(url);
}

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
    firm,
    isConnected,
    receivedBytes,
    sentBytes,
    recentRxHex,
    recentTxHex,
    latestPacket,
    packetsPerSecond,
  } = useFIRM();

  const rxRef = useRef<HTMLTextAreaElement | null>(null);
  const txRef = useRef<HTMLTextAreaElement | null>(null);

  // ---- Calibration input state (local drafts) ----
  const [imuDraft, setImuDraft] = useState<IMUCalibrationDraft>(makeDefaultImuDraft);
  const [magDraft, setMagDraft] = useState<CalibrationDraft>(makeDefaultDraft);

  const [imuStatus, setImuStatus] = useState<string | null>(null);
  const [magStatus, setMagStatus] = useState<string | null>(null);
  const [imuError, setImuError] = useState<string | null>(null);
  const [magError, setMagError] = useState<string | null>(null);
  const [isApplyingImu, setIsApplyingImu] = useState(false);
  const [isApplyingMag, setIsApplyingMag] = useState(false);

  // Clear transient messages when panel hidden.
  useEffect(() => {
    if (visible) return;
    setImuStatus(null);
    setMagStatus(null);
    setImuError(null);
    setMagError(null);
    setIsApplyingImu(false);
    setIsApplyingMag(false);
  }, [visible]);

  const latestPacketRef = useRef<FIRMPacket | null>(null);
  useEffect(() => {
    latestPacketRef.current = latestPacket;
  }, [latestPacket]);

  const [uiPacket, setUiPacket] = useState<FIRMPacket | null>(null);
  useEffect(() => {
    if (!visible) return;

    const id = window.setInterval(() => {
      setUiPacket(latestPacketRef.current);
    }, 100);

    return () => {
      window.clearInterval(id);
    };
  }, [visible]);

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
    try {
      return JSON.stringify(uiPacket, null, 2);
    } catch {
      return String(uiPacket);
    }
  }, [uiPacket]);

  if (!visible) return null;

  const hexBoxClassName =
    "h-40 w-full resize-none overflow-auto rounded-lg border border-slate-200 bg-slate-50 p-3 font-mono text-xs text-slate-800 shadow-inner focus:outline-none";

  const inputClassName =
    "w-full rounded-md border border-slate-300 bg-white px-2 py-1 text-xs font-mono text-slate-900 shadow-inner disabled:bg-slate-100 disabled:text-slate-400 focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme";

  const smallBtnClassName =
    "inline-flex items-center gap-1 rounded-md border border-slate-300 bg-white px-2.5 py-1.5 text-xs font-medium text-slate-700 shadow-sm hover:bg-slate-50 disabled:cursor-not-allowed disabled:bg-slate-100 disabled:text-slate-400";

  const primaryBtnClassName =
    "inline-flex items-center gap-1 rounded-md bg-theme px-3 py-1.5 text-xs font-semibold text-white shadow-sm hover:brightness-95 disabled:cursor-not-allowed disabled:opacity-60";

  const canSend = !!firm && isConnected;

  const applyImu = async () => {
    setImuStatus(null);
    setImuError(null);

    if (!firm || !isConnected) {
      setImuError("Device not connected.");
      return;
    }

    const accel = draftToNumbers(imuDraft.accelerometer);
    const gyro = draftToNumbers(imuDraft.gyroscope);

    if (!accel || !gyro) {
      setImuError("Please fill all IMU calibration fields (accelerometer + gyroscope) with valid finite numbers.");
      return;
    }

    setIsApplyingImu(true);
    try {
      const ok = await firm.setIMUCalibration(
        accel.offsets,
        accel.scaleMatrix,
        gyro.offsets,
        gyro.scaleMatrix,
      );

      if (ok) {
        setImuStatus("IMU calibration applied.");
        setTimeout(() => setImuStatus(null), 3000);
      } else {
        setImuError("Device failed to acknowledge IMU calibration (Timeout).");
      }
    } catch (err) {
      console.error(err);
      setImuError("Failed to send IMU calibration.");
    } finally {
      setIsApplyingImu(false);
    }
  };

  const applyMag = async () => {
    setMagStatus(null);
    setMagError(null);

    if (!firm || !isConnected) {
      setMagError("Device not connected.");
      return;
    }

    const parsed = draftToNumbers(magDraft);
    if (!parsed) {
      setMagError("Please fill all magnetometer calibration fields with valid finite numbers.");
      return;
    }

    setIsApplyingMag(true);
    try {
      const ok = await firm.setMagnetometerCalibration(parsed.offsets, parsed.scaleMatrix);
      if (ok) {
        setMagStatus("Magnetometer calibration applied.");
        setTimeout(() => setMagStatus(null), 3000);
      } else {
        setMagError("Device failed to acknowledge magnetometer calibration (Timeout).");
      }
    } catch (err) {
      console.error(err);
      setMagError("Failed to send magnetometer calibration.");
    } finally {
      setIsApplyingMag(false);
    }
  };

  const handleUploadCalibration = (
    event: React.ChangeEvent<HTMLInputElement>,
    setDraft: React.Dispatch<React.SetStateAction<CalibrationDraft>>,
    setError: React.Dispatch<React.SetStateAction<string | null>>,
  ) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const text = e.target?.result as string;
        const data = JSON.parse(text) as Partial<CalibrationJson>;
        const offsets = Array.isArray(data.offsets) ? data.offsets : null;
        const scaleMatrix = Array.isArray(data.scaleMatrix) ? data.scaleMatrix : null;

        if (!offsets || offsets.length !== 3) throw new Error("offsets must be length 3");
        if (!scaleMatrix || scaleMatrix.length !== 9) throw new Error("scaleMatrix must be length 9");
        if (![...offsets, ...scaleMatrix].every((n) => typeof n === "number" && Number.isFinite(n))) {
          throw new Error("all values must be finite numbers");
        }

        setDraft(numbersToDraft(offsets, scaleMatrix));
        setError(null);
      } catch (err) {
        console.error(err);
        setError("Invalid calibration file. Expected { offsets: [3], scaleMatrix: [9] }.");
      } finally {
        event.target.value = "";
      }
    };

    reader.readAsText(file);
  };

  const CalibrationCard = ({
    title,
    draft,
    setDraft,
    onApply,
    onReset,
    status,
    error,
    setError,
    isApplying,
    downloadName,
    uploadId,
    hideActions,
  }: {
    title: string;
    draft: CalibrationDraft;
    setDraft: React.Dispatch<React.SetStateAction<CalibrationDraft>>;
    onApply: () => void;
    onReset: () => void;
    status: string | null;
    error: string | null;
    setError: React.Dispatch<React.SetStateAction<string | null>>;
    isApplying: boolean;
    downloadName: string;
    uploadId: string;
    hideActions?: boolean;
  }) => {
    return (
      <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
        <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">
            {title}
          </div>

          {!hideActions && (
            <div className="flex flex-wrap items-center gap-2">
              <button
                type="button"
                className={smallBtnClassName}
                disabled={isApplying}
                onClick={() => {
                  const parsed = draftToNumbers(draft);
                  if (!parsed) return;
                  downloadJson(downloadName, {
                    offsets: parsed.offsets,
                    scaleMatrix: parsed.scaleMatrix,
                  } satisfies CalibrationJson);
                }}
                title="Download JSON"
              >
                <Download className="h-3.5 w-3.5" /> Export
              </button>

              <label className={smallBtnClassName} title="Upload JSON">
                <Upload className="h-3.5 w-3.5" /> Import
                <input
                  id={uploadId}
                  type="file"
                  accept="application/json"
                  className="hidden"
                  onChange={(e) => handleUploadCalibration(e, setDraft, setError)}
                />
              </label>

              <button
                type="button"
                className={smallBtnClassName}
                disabled={isApplying}
                onClick={onReset}
                title="Reset offsets to 0 and scale matrix to identity"
              >
                Reset
              </button>

              <button
                type="button"
                className={primaryBtnClassName}
                onClick={onApply}
                disabled={!canSend || isApplying}
              >
                {isApplying ? (
                  <Loader2 className="h-3.5 w-3.5 animate-spin" />
                ) : (
                  <Save className="h-3.5 w-3.5" />
                )}
                Apply
              </button>
            </div>
          )}
        </div>

        <div className="grid grid-cols-1 gap-3 md:grid-cols-2">
          <div>
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-500">
              Offsets (x, y, z)
            </div>
            <div className="grid grid-cols-3 gap-2">
              {([0, 1, 2] as const).map((i) => (
                <input
                  key={i}
                  type="text"
                  inputMode="decimal"
                  disabled={!canSend || isApplying}
                  value={draft.offsets[i]}
                  onChange={(e) =>
                    setDraft((prev) => {
                      const next: CalibrationDraft = {
                        offsets: [...prev.offsets] as CalibrationDraft["offsets"],
                        scale: prev.scale.map((r) => [...r]) as CalibrationDraft["scale"],
                      };
                      next.offsets[i] = e.target.value;
                      return next;
                    })
                  }
                  className={inputClassName}
                  placeholder={i === 0 ? "x" : i === 1 ? "y" : "z"}
                />
              ))}
            </div>
          </div>

          <div>
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-500">
              Scale matrix (row-major 3×3)
            </div>
            <div className="grid grid-cols-3 gap-2">
              {([0, 1, 2] as const).flatMap((r) =>
                ([0, 1, 2] as const).map((c) => {
                  const key = `${r}-${c}`;
                  return (
                    <input
                      key={key}
                      type="text"
                      inputMode="decimal"
                      disabled={!canSend || isApplying}
                      value={draft.scale[r][c]}
                      onChange={(e) =>
                        setDraft((prev) => {
                          const next: CalibrationDraft = {
                            offsets: [...prev.offsets] as CalibrationDraft["offsets"],
                            scale: prev.scale.map((row) => [...row]) as CalibrationDraft["scale"],
                          };
                          next.scale[r][c] = e.target.value;
                          return next;
                        })
                      }
                      className={inputClassName}
                    />
                  );
                }),
              )}
            </div>
          </div>
        </div>

        <div className="mt-2 text-xs">
          {!canSend && <div className="text-slate-500">Connect a device to apply calibration.</div>}
          {status && <div className="text-emerald-600 font-medium">{status}</div>}
          {error && <div className="text-red-600 font-medium">{error}</div>}
        </div>
      </div>
    );
  };

  const IMUCalibrationCard = ({
    title,
    draft,
    setDraft,
    onApply,
    status,
    error,
    setError,
    isApplying,
  }: {
    title: string;
    draft: IMUCalibrationDraft;
    setDraft: React.Dispatch<React.SetStateAction<IMUCalibrationDraft>>;
    onApply: () => void;
    status: string | null;
    error: string | null;
    setError: React.Dispatch<React.SetStateAction<string | null>>;
    isApplying: boolean;
  }) => {
    return (
      <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
        <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">{title}</div>

          <div className="flex flex-wrap items-center gap-2">
            <button
              type="button"
              className={smallBtnClassName}
              disabled={isApplying}
              onClick={() => {
                setDraft(makeDefaultImuDraft());
                setError(null);
              }}
              title="Reset accel+gyro offsets to 0 and scale matrices to identity"
            >
              Reset
            </button>

            <button
              type="button"
              className={primaryBtnClassName}
              onClick={onApply}
              disabled={!canSend || isApplying}
            >
              {isApplying ? (
                <Loader2 className="h-3.5 w-3.5 animate-spin" />
              ) : (
                <Save className="h-3.5 w-3.5" />
              )}
              Apply
            </button>
          </div>
        </div>

        <div className="grid grid-cols-1 gap-3">
          <CalibrationCard
            title="Accelerometer"
            draft={draft.accelerometer}
            setDraft={(next) =>
              setDraft((prev) => ({
                ...prev,
                accelerometer: typeof next === "function" ? next(prev.accelerometer) : next,
              }))
            }
            onApply={() => {}}
            onReset={() =>
              setDraft((prev) => ({
                ...prev,
                accelerometer: makeDefaultDraft(),
              }))
            }
            status={null}
            error={null}
            setError={() => {}}
            isApplying={isApplying}
            downloadName="accel-calibration.json"
            uploadId="accel-cal-upload"
            hideActions
          />

          <CalibrationCard
            title="Gyroscope"
            draft={draft.gyroscope}
            setDraft={(next) =>
              setDraft((prev) => ({
                ...prev,
                gyroscope: typeof next === "function" ? next(prev.gyroscope) : next,
              }))
            }
            onApply={() => {}}
            onReset={() =>
              setDraft((prev) => ({
                ...prev,
                gyroscope: makeDefaultDraft(),
              }))
            }
            status={null}
            error={null}
            setError={() => {}}
            isApplying={isApplying}
            downloadName="gyro-calibration.json"
            uploadId="gyro-cal-upload"
            hideActions
          />
        </div>

        <div className="mt-2 text-xs">
          {!canSend && <div className="text-slate-500">Connect a device to apply calibration.</div>}
          {status && <div className="text-emerald-600 font-medium">{status}</div>}
          {error && <div className="text-red-600 font-medium">{error}</div>}
        </div>
      </div>
    );
  };

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

          <div className="mt-2">
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-500">
              Recent received bytes (hex)
            </div>
            <textarea
              ref={rxRef}
              readOnly
              value={recentRxHex}
              className={hexBoxClassName}
              spellCheck={false}
            />
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
            <textarea
              ref={txRef}
              readOnly
              value={recentTxHex}
              className={hexBoxClassName}
              spellCheck={false}
            />
          </div>
        </div>
      </div>

      <div className="mt-3">
        <div className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-500">
          Calibration
        </div>
        <div className="grid grid-cols-1 gap-3 lg:grid-cols-2">
          <IMUCalibrationCard
            title="IMU Calibration"
            draft={imuDraft}
            setDraft={setImuDraft}
            onApply={applyImu}
            status={imuStatus}
            error={imuError}
            setError={setImuError}
            isApplying={isApplyingImu}
          />

          <CalibrationCard
            title="Magnetometer Calibration"
            draft={magDraft}
            setDraft={setMagDraft}
            onApply={applyMag}
            onReset={() => setMagDraft(makeDefaultDraft())}
            status={magStatus}
            error={magError}
            setError={setMagError}
            isApplying={isApplyingMag}
            downloadName="mag-calibration.json"
            uploadId="mag-cal-upload"
          />
        </div>
      </div>

      <div className="mt-3">
        <div className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-500">
          Latest FIRMPacket
        </div>
        <pre className="max-h-[28rem] overflow-auto rounded-lg border border-slate-200 bg-slate-50 p-3 text-xs text-slate-800 whitespace-pre shadow-inner">
          {packetText}
        </pre>
      </div>
    </section>
  );
}
