import React, { Fragment, useState } from "react";
import { Listbox, Transition } from "@headlessui/react";
import { ChevronsUpDown, Check, Download, RefreshCcw, Rocket } from "lucide-react";

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
  const [selectedLog, setSelectedLog] = useState<LogOption>(LOG_OPTIONS[0]);

  const handleRunCalibration = () => console.log("Run Calibration clicked");
  const handleDownloadLog = () => console.log("Download log:", selectedLog);
  const handleCustomAction = () => console.log("Custom action triggered");

  return (
    <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3 pb-4 shadow-sm text-slate-900">
      <h2 className="mb-3 text-lg font-semibold leading-tight">Actions</h2>

      <div className="grid grid-cols-1 gap-5 md:grid-cols-4 text-sm">
        {/* Run Calibration – 1/4 */}
        <div className="col-span-1 flex flex-col justify-between rounded-lg border border-white bg-slate-50/70">
          <div>
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">
              Run Calibration
            </p>
            <p className="text-xs text-slate-500">
              Zero sensors and collect baseline data for FIRM.
            </p>
          </div>
          <div className="mt-3 flex items-center gap-3">
            <button
              type="button"
              onClick={handleRunCalibration}
              className="inline-flex items-center gap-2 rounded-md bg-theme px-3 py-1.5 text-xs font-semibold text-white shadow-sm hover:brightness-110 focus:outline-none focus:ring-2 focus:ring-theme/70 focus:ring-offset-1 focus:ring-offset-white"
            >
              <RefreshCcw className="h-4 w-4" />
              Run Calibration
            </button>
            <p className="text-[11px] text-slate-500">
              Last run: <span className="font-medium text-slate-700">N/A</span>
            </p>
          </div>
        </div>

        {/* Custom Action – 1/4 */}
        <div className="col-span-1 flex flex-col justify-between rounded-lg border border-white bg-slate-50/70">
          <div>
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">
              Custom Action
            </p>
            <p className="text-xs text-slate-500">
              Placeholder for a future FIRM control action or automation trigger.
            </p>
          </div>
          <div className="mt-3 flex items-center gap-3">
            <button
              type="button"
              onClick={handleCustomAction}
              className="inline-flex items-center gap-2 rounded-md bg-theme px-3 py-1.5 text-xs font-semibold text-white shadow-sm hover:brightness-110 focus:outline-none focus:ring-2 focus:ring-theme/70 focus:ring-offset-1 focus:ring-offset-white"
            >
              <Rocket className="h-4 w-4 text-white" />
              Execute Action
            </button>
          </div>
        </div>

        {/* Download Log File – 2/4 */}
        <div className="col-span-1 md:col-span-2 flex flex-col justify-between rounded-lg border border-white bg-slate-50/70">
          <div>
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">
              Download Log File
            </p>
            <p className="text-xs text-slate-500">
              Choose a recorded session from FIRM and download its log file for offline analysis.
            </p>
          </div>
          <div className="mt-3 flex flex-wrap items-center gap-3">
            <Listbox value={selectedLog} onChange={setSelectedLog}>
              <div className="relative w-full sm:max-w-[220px]">
                <Listbox.Button className="relative w-full cursor-default rounded-md border border-slate-300 bg-white py-2 pl-3 pr-8 text-left text-sm text-slate-800 shadow-sm hover:bg-slate-50 focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme">
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
                  <Listbox.Options className="absolute z-20 mt-1 max-h-60 w-full overflow-auto rounded-md border border-slate-200 bg-white py-1 text-sm shadow-lg focus:outline-none">
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
                            {selected && <Check className="mt-[2px] h-4 w-4 text-theme" />}
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
              className="inline-flex shrink-0 items-center gap-2 rounded-md bg-theme px-3 py-1.5 text-xs font-semibold text-white shadow-sm hover:brightness-110 focus:outline-none focus:ring-2 focus:ring-theme/70 focus:ring-offset-1 focus:ring-offset-white"
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
