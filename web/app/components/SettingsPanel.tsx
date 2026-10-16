import React, { Fragment, useEffect, useState } from "react";
import { Listbox, Transition } from "@headlessui/react";
import { Check, ChevronsUpDown, Download, Upload, Save, Loader2 } from "lucide-react";
import { useFIRM } from "~/contexts/FIRMContext";

const PROTOCOL_MAP: Record<string, number> = {
  USB: 1,
  UART: 2,
  I2C: 3,
  SPI: 4,
};

const PROTOCOL_MAP_INVERSE: Record<number, string> = Object.entries(PROTOCOL_MAP).reduce(
  (acc, [key, val]) => ({ ...acc, [val]: key }),
  {},
);

const PROTOCOL_OPTIONS = ["USB", "UART", "I2C", "SPI"];

function classNames(...classes: Array<string | boolean | null | undefined>) {
  return classes.filter(Boolean).join(" ");
}

export function SettingsPanel() {
  const { firm, deviceConfig, refreshDeviceMeta, isConnected } = useFIRM();

  const [deviceName, setDeviceName] = useState("FIRM Device");
  const [freqInput, setFreqInput] = useState("100");
  const [protocol, setProtocol] = useState("USB");

  const [freqError, setFreqError] = useState<string | null>(null);
  const [configError, setConfigError] = useState<string | null>(null);
  const [successMsg, setSuccessMsg] = useState<string | null>(null);
  const [isSaving, setIsSaving] = useState(false);

  useEffect(() => {
    if (deviceConfig) {
      if (deviceConfig.name) setDeviceName(deviceConfig.name);
      if (deviceConfig.frequency) setFreqInput(String(deviceConfig.frequency));

      if (deviceConfig.protocol) {
        const protoStr = PROTOCOL_MAP_INVERSE[deviceConfig.protocol];
        if (protoStr) setProtocol(protoStr);
      }
    }
  }, [deviceConfig]);

  const handleFreqChange = (value: string) => {
    if (value === "" || /^\d*$/.test(value)) {
      setFreqInput(value);
      setFreqError(null);
    }
  };

  const validateFrequency = (): boolean => {
    if (freqInput === "") {
      setFreqError("Required");
      return false;
    }
    const numeric = Number(freqInput);
    if (!Number.isInteger(numeric) || numeric < 1 || numeric > 1000) {
      setFreqError("1-1000 Hz");
      return false;
    }
    setFreqError(null);
    return true;
  };

  const handleApply = async (event: React.FormEvent) => {
    event.preventDefault();
    setSuccessMsg(null);
    setConfigError(null);

    if (!firm || !isConnected) {
      setConfigError("Device not connected.");
      return;
    }

    if (!validateFrequency()) return;

    setIsSaving(true);
    try {
      const freqNum = Number(freqInput);
      const protoNum = PROTOCOL_MAP[protocol] || 1; // Default to 1 (USB) if undefined

      // Call the actual library function
      const success = await firm.setDeviceConfig(deviceName, freqNum, protoNum);

      if (success) {
        setSuccessMsg("Settings saved successfully.");
        // Refresh the context so other components see the new config
        await refreshDeviceMeta();

        // Clear success message after 3 seconds
        setTimeout(() => setSuccessMsg(null), 3000);
      } else {
        setConfigError("Device failed to acknowledge settings (Timeout).");
      }
    } catch (err) {
      console.error(err);
      setConfigError("Failed to send settings.");
    } finally {
      setIsSaving(false);
    }
  };

  // --- Import / Export Handlers (JSON only) ---

  const buildConfigObject = () => ({
    deviceName,
    updateFrequencyHz: Number(freqInput),
    protocol, // storing as string in JSON for readability
  });

  const handleDownloadConfig = () => {
    const config = buildConfigObject();
    const blob = new Blob([JSON.stringify(config, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = "firm-settings.json";
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  const handleUploadConfig: React.ChangeEventHandler<HTMLInputElement> = (event) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const text = e.target?.result as string;
        const data = JSON.parse(text);

        // Basic validation
        if (
          typeof data.deviceName !== "string" ||
          typeof data.updateFrequencyHz !== "number" ||
          !PROTOCOL_OPTIONS.includes(data.protocol)
        ) {
          throw new Error("Invalid config format");
        }

        setDeviceName(data.deviceName);
        setFreqInput(String(data.updateFrequencyHz));
        setProtocol(data.protocol);
        setFreqError(null);
        setConfigError(null);
      } catch (err) {
        console.error(err);
        setConfigError("Invalid settings file.");
      } finally {
        event.target.value = ""; // allow re-upload
      }
    };
    reader.readAsText(file);
  };

  return (
    <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-5 shadow-sm text-slate-900">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-lg font-semibold leading-tight">FIRM Settings</h2>
        {/* Status Indicator */}
        <div className="text-sm">
          {isSaving && (
            <span className="text-theme flex items-center gap-1">
              <Loader2 className="h-3 w-3 animate-spin" /> Saving...
            </span>
          )}
          {successMsg && (
            <span className="text-emerald-600 font-medium flex items-center gap-1">
              <Check className="h-3 w-3" /> {successMsg}
            </span>
          )}
          {configError && <span className="text-red-600 font-medium">{configError}</span>}
        </div>
      </div>

      <form onSubmit={handleApply} className="space-y-4 text-sm">
        {/* Row 1 — Device Name and Frequency */}
        <div className="grid grid-cols-1 gap-4 md:grid-cols-2">
          <div className="flex flex-col gap-1">
            <label
              htmlFor="deviceName"
              className="text-xs font-semibold uppercase tracking-wide text-slate-500"
            >
              Device Name
            </label>
            <input
              id="deviceName"
              type="text"
              maxLength={32}
              disabled={!isConnected}
              value={deviceName}
              onChange={(e) => setDeviceName(e.target.value)}
              className="w-full rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 shadow-inner disabled:bg-slate-100 disabled:text-slate-400 focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme"
              placeholder="FIRM Device"
            />
          </div>

          <div className="flex flex-col gap-1">
            <label
              htmlFor="frequency"
              className="text-xs font-semibold uppercase tracking-wide text-slate-500"
            >
              Frequency (Hz)
            </label>
            <div className="relative">
              <input
                id="frequency"
                type="text"
                inputMode="numeric"
                disabled={!isConnected}
                value={freqInput}
                onChange={(e) => handleFreqChange(e.target.value)}
                onBlur={validateFrequency}
                className={`w-full rounded-md border bg-white px-3 py-2 text-sm text-slate-900 shadow-inner disabled:bg-slate-100 disabled:text-slate-400 focus:outline-none focus:ring-1 ${
                  freqError
                    ? "border-red-500 focus:border-red-500 focus:ring-red-500"
                    : "border-slate-300 focus:border-theme focus:ring-theme"
                }`}
                placeholder="100"
              />
            </div>
            {freqError && <p className="text-xs text-red-500 mt-1">{freqError}</p>}
          </div>
        </div>

        {/* Row 2 — Protocol */}
        <div className="flex flex-col gap-1">
          <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">Protocol</p>

          <Listbox value={protocol} onChange={setProtocol} disabled={!isConnected}>
            <div className="relative w-full md:w-1/2">
              <Listbox.Button className="relative w-full cursor-default rounded-md border border-slate-300 bg-white py-2 pl-3 pr-8 text-left text-sm text-slate-800 shadow-sm disabled:bg-slate-100 disabled:text-slate-400 hover:bg-slate-50 focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme">
                <span className="block truncate">{protocol}</span>
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
                <Listbox.Options className="absolute z-20 mt-1 w-full origin-top-left rounded-md border border-slate-200 bg-white py-1 text-sm shadow-lg focus:outline-none max-h-60 overflow-y-auto">
                  {PROTOCOL_OPTIONS.map((option) => (
                    <Listbox.Option
                      key={option}
                      value={option}
                      className={({ active }) =>
                        classNames(
                          "relative cursor-pointer select-none px-3 py-2",
                          active ? "bg-slate-50 text-slate-900" : "text-slate-800",
                        )
                      }
                    >
                      {({ selected }) => (
                        <div className="flex items-center gap-2">
                          {selected && <Check className="h-4 w-4 text-theme" />}
                          <span
                            className={classNames(
                              "truncate",
                              selected ? "font-semibold" : "font-medium",
                            )}
                          >
                            {option}
                          </span>
                        </div>
                      )}
                    </Listbox.Option>
                  ))}
                </Listbox.Options>
              </Transition>
            </div>
          </Listbox>
          <p className="text-xs text-slate-500 mt-1">
            Interface FIRM uses to communicate with host.
          </p>
        </div>

        {/* Actions */}
        <div className="pt-2 border-t border-slate-100 mt-4">
          <div className="flex items-center justify-between gap-3">
            {/* Left: Import / Export */}
            <div className="flex items-center gap-2 text-xs">
              <button
                type="button"
                onClick={handleDownloadConfig}
                className="inline-flex cursor-pointer items-center gap-1 rounded-md border border-slate-300
               px-2.5 py-1.5 text-xs font-medium text-slate-700 shadow-sm
               transition-colors duration-150 hover:border-theme hover:text-theme hover:bg-theme/5"
              >
                <Download className="h-3.5 w-3.5" />
                <span>Export</span>
              </button>

              <div>
                <input
                  id="firm-config-upload"
                  type="file"
                  accept="application/json"
                  className="hidden"
                  onChange={handleUploadConfig}
                />
                <label
                  htmlFor="firm-config-upload"
                  className="inline-flex cursor-pointer items-center gap-1 rounded-md border border-slate-300
                 px-2.5 py-1.5 text-xs font-medium text-slate-700 shadow-sm
                 transition-colors duration-150 hover:border-theme hover:text-theme hover:bg-theme/5"
                >
                  <Upload className="h-3.5 w-3.5" />
                  <span>Import</span>
                </label>
              </div>
            </div>

            {/* Right: Apply */}
            <div className="flex gap-2">
              <button
                type="submit"
                disabled={!isConnected || isSaving}
                className="inline-flex items-center gap-1 rounded-md border border-transparent bg-theme px-4 py-1.5 text-xs font-medium text-white shadow-sm hover:bg-theme-dark focus:outline-none focus:ring-2 focus:ring-theme focus:ring-offset-2 disabled:opacity-60 disabled:cursor-not-allowed"
              >
                {isSaving ? (
                  <Loader2 className="h-3.5 w-3.5 animate-spin" />
                ) : (
                  <Save className="h-3.5 w-3.5" />
                )}
                <span>{isSaving ? "Saving..." : "Apply Settings"}</span>
              </button>
            </div>
          </div>
        </div>
      </form>
    </section>
  );
}
