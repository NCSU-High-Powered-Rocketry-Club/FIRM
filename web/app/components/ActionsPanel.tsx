import React, { Fragment, useState, useRef, useEffect } from "react";
import { Listbox, Transition } from "@headlessui/react";
import { ChevronsUpDown, Check, Download, RefreshCcw, XCircle } from "lucide-react";
import { useFirm } from "~/contexts/FirmContext";

// The length each calibration takes
const TIME_CAL_IMU_SECONDS = 10;
const TIME_CAL_MAG_SECONDS = 10;

type LogOption = {
  id: string;
  label: string;
  description: string;
};

const LOG_OPTIONS: LogOption[] = [
  {
    id: "latest",
    label: "Most Recent Session",
    description: "The last active run from this device",
  },
  { id: "flight-001", label: "Flight 001", description: "Test launch profile – 2025-03-12" },
  { id: "flight-002", label: "Flight 002", description: "Full stack integration test" },
  {
    id: "ground-cal",
    label: "Ground Calibration",
    description: "Static sensor characterization log",
  },
];

function classNames(...classes: Array<string | false | null | undefined>) {
  return classes.filter(Boolean).join(" ");
}

export function ActionsPanel() {
  const { firm } = useFirm();
  const [selectedLog, setSelectedLog] = useState<LogOption>(LOG_OPTIONS[0]);

  // Calibration State
  const [activeCal, setActiveCal] = useState<"IMU" | "MAG" | null>(null);
  const [countdown, setCountdown] = useState<number>(0);

  // Ref for the timer interval so we can clear it easily
  const timerRef = useRef<number | null>(null);

  // Cleanup timer on unmount
  useEffect(() => {
    return () => stopTimer();
  }, []);

  const stopTimer = () => {
    if (timerRef.current) {
      window.clearInterval(timerRef.current);
      timerRef.current = null;
    }
  };

  const startTimer = (seconds: number) => {
    stopTimer();
    setCountdown(seconds);
    timerRef.current = window.setInterval(() => {
      setCountdown((prev) => {
        if (prev <= 1) {
          stopTimer();
          return 0;
        }
        return prev - 1;
      });
    }, 1000);
  };

  const handleCalibrateIMU = async () => {
    if (!firm) return;
    setActiveCal("IMU");
    startTimer(TIME_CAL_IMU_SECONDS);

    try {
      const success = await firm.runIMUCalibration();
      if (!success) {
        console.warn("IMU Calibration failed or timed out");
      }
    } catch (err) {
      console.error("IMU Calibration error:", err);
    } finally {
      setActiveCal(null);
      stopTimer();
    }
  };

  const handleCalibrateMag = async () => {
    if (!firm) return;
    setActiveCal("MAG");
    startTimer(TIME_CAL_MAG_SECONDS);

    try {
      const success = await firm.runMagnetometerCalibration();
      if (!success) {
        console.warn("Mag Calibration failed or timed out");
      }
    } catch (err) {
      console.error("Mag Calibration error:", err);
    } finally {
      setActiveCal(null);
      stopTimer();
    }
  };

  const handleCancel = async () => {
    if (!firm) return;
    // Don't wait for the cancel response to update UI, makes it feel snappier
    stopTimer();
    setActiveCal(null);

    try {
      await firm.sendCancelCommand();
    } catch (err) {
      console.error("Failed to send cancel command:", err);
    }
  };

  const handleDownloadLog = () => console.log("Download log:", selectedLog);

  // Helper to render the button state
  const renderCalibrateButton = (type: "IMU" | "MAG", label: string, onClick: () => void) => {
    const isRunning = activeCal === type;
    const isOtherRunning = activeCal !== null && !isRunning;

    // If THIS specific calibration is running, show Cancel button
    if (isRunning) {
      return (
        <button
          type="button"
          onClick={handleCancel}
          className="inline-flex items-center gap-2 rounded-md bg-red-50 px-3 py-1.5 text-xs font-semibold text-red-600 border border-red-200 shadow-sm hover:bg-red-100 focus:outline-none focus:ring-2 focus:ring-red-500 focus:ring-offset-1"
        >
          <XCircle className="h-4 w-4" />
          Cancel ({countdown}s)
        </button>
      );
    }

    // Standard Button
    return (
      <button
        type="button"
        onClick={onClick}
        disabled={!firm || isOtherRunning}
        className={classNames(
          "inline-flex items-center gap-2 rounded-md px-3 py-1.5 text-xs font-semibold text-white shadow-sm focus:outline-none focus:ring-2 focus:ring-offset-1 focus:ring-offset-white transition-all",
          !firm || isOtherRunning
            ? "bg-slate-300 cursor-not-allowed"
            : "bg-[var(--color-theme)] hover:brightness-110 focus:ring-[var(--color-theme)]",
        )}
      >
        <RefreshCcw className={classNames("h-4 w-4", isRunning && "animate-spin")} />
        {label}
      </button>
    );
  };

  return (
    <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3 pb-4 shadow-sm text-slate-900">
      <h2 className="mb-3 text-lg font-semibold leading-tight flex items-center gap-2">Actions</h2>

      <div className="grid grid-cols-1 gap-5 md:grid-cols-4 text-sm">
        {/* --- Calibrate IMU --- */}
        <div className="col-span-1 flex flex-col justify-between rounded-lg border border-transparent bg-white">
          <div>
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">
              Calibrate IMU
            </p>
            <p className="text-xs text-slate-500 mt-1">
              Zero accel/gyro. Keep device still and level.
            </p>
          </div>
          <div className="mt-3 flex items-center gap-3">
            {renderCalibrateButton("IMU", "Calibrate IMU", handleCalibrateIMU)}
          </div>
        </div>

        {/* --- Calibrate Magnetometer --- */}
        <div className="col-span-1 flex flex-col justify-between rounded-lg border border-transparent bg-white">
          <div>
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">
              Calibrate Magnetometer
            </p>
            <p className="text-xs text-slate-500 mt-1">
              Rotate device in all axes to map hard/soft iron.
            </p>
          </div>
          <div className="mt-3 flex items-center gap-3">
            {renderCalibrateButton("MAG", "Calibrate Mag", handleCalibrateMag)}
          </div>
        </div>

        {/* --- Download Log (Logic Preserved, UI Only) --- */}
        <div className="col-span-1 md:col-span-2 flex flex-col justify-between rounded-lg border border-transparent bg-white">
          <div>
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">
              Download Log File
            </p>
            <p className="text-xs text-slate-500 mt-1">
              Choose a recorded session from FIRM and download for offline analysis.
            </p>
          </div>
          <div className="mt-3 flex flex-wrap items-center gap-3">
            <Listbox value={selectedLog} onChange={setSelectedLog}>
              <div className="relative w-full sm:max-w-[220px]">
                <Listbox.Button className="relative w-full cursor-default rounded-md border border-slate-300 bg-white py-2 pl-3 pr-8 text-left text-sm text-slate-800 shadow-sm hover:bg-slate-50 focus:border-[var(--color-theme)] focus:outline-none focus:ring-1 focus:ring-[var(--color-theme)]">
                  <span className="block truncate">{selectedLog.label}</span>
                  <span className="pointer-events-none absolute inset-y-0 right-0 flex items-center pr-2">
                    <ChevronsUpDown className="h-4 w-4 text-slate-400" />
                  </span>
                </Listbox.Button>
                <Transition
                  as={Fragment}
                  leave="transition ease-in duration-100"
                  leaveFrom="opacity-100"
                  leaveTo="opacity-0"
                >
                  <Listbox.Options className="absolute z-20 mt-1 w-full origin-top-left rounded-md border border-slate-200 bg-white py-1 text-sm shadow-lg focus:outline-none max-h-40 overflow-y-auto">
                    {LOG_OPTIONS.map((log) => (
                      <Listbox.Option
                        key={log.id}
                        value={log}
                        className={({ active }) =>
                          classNames(
                            "relative cursor-pointer select-none px-3 py-2",
                            active ? "bg-slate-50 text-slate-900" : "text-slate-800",
                          )
                        }
                      >
                        {({ selected }) => (
                          <div className="flex items-start gap-2">
                            {selected && (
                              <Check className="mt-[2px] h-4 w-4 text-[var(--color-theme)]" />
                            )}
                            <div>
                              <p
                                className={classNames(
                                  "truncate",
                                  selected ? "font-semibold" : "font-medium",
                                )}
                              >
                                {log.label}
                              </p>
                              <p className="truncate text-[11px] text-slate-500">
                                {log.description}
                              </p>
                            </div>
                          </div>
                        )}
                      </Listbox.Option>
                    ))}
                  </Listbox.Options>
                </Transition>
              </div>
            </Listbox>

            <button
              type="button"
              onClick={handleDownloadLog}
              className="inline-flex shrink-0 items-center gap-2 rounded-md bg-[var(--color-theme)] px-3 py-1.5 text-xs font-semibold text-white shadow-sm hover:brightness-110 focus:outline-none focus:ring-2 focus:ring-[var(--color-theme)] focus:ring-offset-1 focus:ring-offset-white"
            >
              <Download className="h-4 w-4 text-white" />
              Download Log
            </button>
          </div>
        </div>
      </div>
    </section>
  );
}
