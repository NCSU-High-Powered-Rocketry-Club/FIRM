import React, { useCallback, useMemo, useState } from "react";
import { Loader2, Save } from "lucide-react";
import { useFIRM } from "~/contexts/FIRMContext";
import type {CalibrationValues} from "firm-client";

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
  return Number.isFinite(n) ? n : null;
}

function draftToNumbers(
  draft: CalibrationDraft,
): { offsets: number[]; scaleMatrix: number[] } | null {
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

function formatNumberForInput(n: number): string {
  return String(n);
}

function offsetsToDraft(offsets: [number, number, number]): CalibrationDraft["offsets"] {
  return [
    formatNumberForInput(offsets[0]),
    formatNumberForInput(offsets[1]),
    formatNumberForInput(offsets[2]),
  ];
}

function matrix9ToDraftScale(
  m: [number, number, number, number, number, number, number, number, number],
): CalibrationDraft["scale"] {
  // Row-major 3x3:
  // [ m0 m1 m2
  //   m3 m4 m5
  //   m6 m7 m8 ]
  return [
    [formatNumberForInput(m[0]), formatNumberForInput(m[1]), formatNumberForInput(m[2])],
    [formatNumberForInput(m[3]), formatNumberForInput(m[4]), formatNumberForInput(m[5])],
    [formatNumberForInput(m[6]), formatNumberForInput(m[7]), formatNumberForInput(m[8])],
  ];
}

function calibrationValuesToDrafts(cal: CalibrationValues): {
  imu: IMUCalibrationDraft;
  mag: CalibrationDraft;
} {
  return {
    imu: {
      accelerometer: {
        offsets: offsetsToDraft(cal.imu_accelerometer_offsets),
        scale: matrix9ToDraftScale(cal.imu_accelerometer_scale_matrix),
      },
      gyroscope: {
        offsets: offsetsToDraft(cal.imu_gyroscope_offsets),
        scale: matrix9ToDraftScale(cal.imu_gyroscope_scale_matrix),
      },
    },
    mag: {
      offsets: offsetsToDraft(cal.magnetometer_offsets),
      scale: matrix9ToDraftScale(cal.magnetometer_scale_matrix),
    },
  };
}

export function CalibrationPanel({ visible }: { visible: boolean }) {
  const { firm, isConnected } = useFIRM();

  const [imuDraft, setImuDraft] = useState<IMUCalibrationDraft>(makeDefaultImuDraft);
  const [magDraft, setMagDraft] = useState<CalibrationDraft>(makeDefaultDraft);

  const [imuError, setImuError] = useState<string | null>(null);
  const [magError, setMagError] = useState<string | null>(null);
  const [imuStatus, setImuStatus] = useState<string | null>(null);
  const [magStatus, setMagStatus] = useState<string | null>(null);

  const [isApplyingImu, setIsApplyingImu] = useState(false);
  const [isApplyingMag, setIsApplyingMag] = useState(false);

  const canSend = !!firm && isConnected;

  const inputClassName =
    "w-full rounded-md border border-slate-300 bg-white px-2 py-1 text-xs font-mono text-slate-900 shadow-inner disabled:bg-slate-100 disabled:text-slate-400 focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme";

  const smallBtnClassName =
    "inline-flex items-center gap-2 rounded-md border border-slate-300 bg-white px-2.5 py-1.5 text-xs font-medium text-slate-700 shadow-sm hover:bg-slate-50 disabled:cursor-not-allowed disabled:bg-slate-100 disabled:text-slate-400";

  const primaryBtnClassName =
    "inline-flex items-center gap-2 rounded-md bg-theme px-3 py-1.5 text-xs font-semibold text-white shadow-sm hover:brightness-95 disabled:cursor-not-allowed disabled:opacity-60";

  const setDraftOffset = useCallback(
    (
      setDraft: React.Dispatch<React.SetStateAction<CalibrationDraft>>,
      idx: 0 | 1 | 2,
      value: string,
    ) => {
      setDraft((prev) => ({
        offsets: [
          ...prev.offsets.slice(0, idx),
          value,
          ...prev.offsets.slice(idx + 1),
        ] as CalibrationDraft["offsets"],
        scale: prev.scale.map((r) => [...r]) as CalibrationDraft["scale"],
      }));
    },
    [],
  );

  const setDraftScale = useCallback(
    (
      setDraft: React.Dispatch<React.SetStateAction<CalibrationDraft>>,
      r: 0 | 1 | 2,
      c: 0 | 1 | 2,
      value: string,
    ) => {
      setDraft((prev) => {
        const nextScale = prev.scale.map((row) => [...row]) as CalibrationDraft["scale"];
        nextScale[r][c] = value;
        return { offsets: [...prev.offsets] as CalibrationDraft["offsets"], scale: nextScale };
      });
    },
    [],
  );

  const refreshCalibrationFromDevice = useCallback(async () => {
    if (!firm || !isConnected) return;

    try {
      const cal = await firm.getCalibration();
      if (!cal) return;

      const { imu, mag } = calibrationValuesToDrafts(cal);
      setImuDraft(imu);
      setMagDraft(mag);
    } catch (e) {
      console.warn("Failed to refresh calibration from device:", e);
    }
  }, [firm, isConnected]);

  React.useEffect(() => {
    if (!visible) return;
    if (!firm || !isConnected) return;

    refreshCalibrationFromDevice().catch(() => {
      /* ignore initial load errors */
    });
  }, [visible, firm, isConnected, refreshCalibrationFromDevice]);

  const applyImu = useCallback(async () => {
    setImuError(null);
    setImuStatus(null);

    if (!firm || !isConnected) {
      setImuError("Device not connected.");
      return;
    }

    const accel = draftToNumbers(imuDraft.accelerometer);
    const gyro = draftToNumbers(imuDraft.gyroscope);
    if (!accel || !gyro) {
      setImuError("Fill all IMU fields with valid finite numbers.");
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
        await refreshCalibrationFromDevice();
      } else {
        setImuError("IMU calibration not acknowledged (timeout).");
      }
    } catch (e) {
      console.error(e);
      setImuError("Failed to send IMU calibration.");
    } finally {
      setIsApplyingImu(false);
      window.setTimeout(() => setImuStatus(null), 3000);
    }
  }, [firm, isConnected, imuDraft, refreshCalibrationFromDevice]);

  const applyMag = useCallback(async () => {
    setMagError(null);
    setMagStatus(null);

    if (!firm || !isConnected) {
      setMagError("Device not connected.");
      return;
    }

    const mag = draftToNumbers(magDraft);
    if (!mag) {
      setMagError("Fill all magnetometer fields with valid finite numbers.");
      return;
    }

    setIsApplyingMag(true);
    try {
      const ok = await firm.setMagnetometerCalibration(mag.offsets, mag.scaleMatrix);
      if (ok) {
        setMagStatus("Magnetometer calibration applied.");
        await refreshCalibrationFromDevice();
      } else {
        setMagError("Magnetometer calibration not acknowledged (timeout).");
      }
    } catch (e) {
      console.error(e);
      setMagError("Failed to send magnetometer calibration.");
    } finally {
      setIsApplyingMag(false);
      window.setTimeout(() => setMagStatus(null), 3000);
    }
  }, [firm, isConnected, magDraft, refreshCalibrationFromDevice]);

  const CalibrationBlock = useMemo(() => {
    const Comp = ({
                    title,
                    draft,
                    setDraft,
                    onApply,
                    isApplying,
                    status,
                    error,
                  }: {
      title: string;
      draft: CalibrationDraft;
      setDraft: React.Dispatch<React.SetStateAction<CalibrationDraft>>;
      onApply: () => void;
      isApplying: boolean;
      status: string | null;
      error: string | null;
    }) => (
      <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
        <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">{title}</div>

          <div className="flex flex-wrap items-center gap-2">
            <button
              type="button"
              className={smallBtnClassName}
              disabled={isApplying}
              onClick={() => setDraft(makeDefaultDraft())}
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
                <Loader2 className="h-4 w-4 animate-spin" />
              ) : (
                <Save className="h-4 w-4" />
              )}
              Apply
            </button>
          </div>
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
                  onChange={(e) => setDraftOffset(setDraft, i, e.target.value)}
                  className={inputClassName}
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
                ([0, 1, 2] as const).map((c) => (
                  <input
                    key={`${r}-${c}`}
                    type="text"
                    inputMode="decimal"
                    disabled={!canSend || isApplying}
                    value={draft.scale[r][c]}
                    onChange={(e) => setDraftScale(setDraft, r, c, e.target.value)}
                    className={inputClassName}
                  />
                )),
              )}
            </div>
          </div>
        </div>

        <div className="mt-2 text-xs">
          {!canSend && <div className="text-slate-500">Connect a device to apply calibration.</div>}
          {status && <div className="font-medium text-emerald-600">{status}</div>}
          {error && <div className="font-medium text-red-600">{error}</div>}
        </div>
      </div>
    );

    Comp.displayName = "CalibrationBlock";
    return Comp;
  }, [
    canSend,
    inputClassName,
    primaryBtnClassName,
    setDraftOffset,
    setDraftScale,
    smallBtnClassName,
  ]);

  if (!visible) return null;

  return (
    <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-4 shadow-sm text-slate-900">
      <div className="mb-3 flex items-center justify-between gap-3">
        <div>
          <h2 className="text-lg font-semibold leading-tight">Calibration Panel</h2>
          <div className="mt-0.5 text-xs text-slate-500">
            Set explicit calibration coefficients (no sampling routine).
          </div>
        </div>
        <div className="text-xs text-slate-500">
          <span className="rounded-md border border-slate-300 bg-slate-50 px-2 py-1">
            Press <span className="font-mono">c</span> to toggle
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 gap-3 lg:grid-cols-2">
        <div className="rounded-lg border border-slate-200 bg-slate-50 px-3 py-2">
          <div className="mb-2 flex items-center justify-between gap-2">
            <div className="text-xs font-semibold uppercase tracking-wide text-slate-500">IMU</div>
            <button
              type="button"
              className={smallBtnClassName}
              disabled={isApplyingImu || isApplyingMag}
              onClick={() => {
                setImuDraft(makeDefaultImuDraft());
                setImuError(null);
                setImuStatus(null);
                setMagDraft(makeDefaultDraft());
                setMagError(null);
                setMagStatus(null);
              }}
              title="Reset all calibration coefficient inputs"
            >
              Reset all
            </button>
          </div>

          <div className="grid grid-cols-1 gap-3">
            <CalibrationBlock
              title="Accelerometer"
              draft={imuDraft.accelerometer}
              setDraft={(next) =>
                setImuDraft((prev) => ({
                  ...prev,
                  accelerometer: typeof next === "function" ? next(prev.accelerometer) : next,
                }))
              }
              onApply={applyImu}
              isApplying={isApplyingImu}
              status={imuStatus}
              error={imuError}
            />

            <CalibrationBlock
              title="Gyroscope"
              draft={imuDraft.gyroscope}
              setDraft={(next) =>
                setImuDraft((prev) => ({
                  ...prev,
                  gyroscope: typeof next === "function" ? next(prev.gyroscope) : next,
                }))
              }
              onApply={applyImu}
              isApplying={isApplyingImu}
              status={imuStatus}
              error={imuError}
            />
          </div>
        </div>

        <CalibrationBlock
          title="Magnetometer"
          draft={magDraft}
          setDraft={setMagDraft}
          onApply={applyMag}
          isApplying={isApplyingMag}
          status={magStatus}
          error={magError}
        />
      </div>
    </section>
  );
}
