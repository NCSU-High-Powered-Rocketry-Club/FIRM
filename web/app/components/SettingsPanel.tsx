import React, { Fragment, useState } from "react";
import { Menu, Listbox, Transition } from "@headlessui/react";
import { ChevronDown, ChevronsUpDown, Check } from "lucide-react";

type TelemetryFieldId =
  | "accelX"
  | "accelY"
  | "accelZ"
  | "gyroX"
  | "gyroY"
  | "gyroZ"
  | "magX"
  | "magY"
  | "magZ"
  | "pressure"
  | "altitude"
  | "temperature";

type TelemetryField = {
  id: TelemetryFieldId;
  label: string;
  group: "IMU" | "Magnetometer" | "Barometer";
};

type ProtocolOption = "USB" | "UART" | "I2C" | "SPI";

const TELEMETRY_FIELDS: TelemetryField[] = [
  { id: "accelX", label: "Accel X", group: "IMU" },
  { id: "accelY", label: "Accel Y", group: "IMU" },
  { id: "accelZ", label: "Accel Z", group: "IMU" },
  { id: "gyroX", label: "Gyro X", group: "IMU" },
  { id: "gyroY", label: "Gyro Y", group: "IMU" },
  { id: "gyroZ", label: "Gyro Z", group: "IMU" },

  { id: "magX", label: "Mag X", group: "Magnetometer" },
  { id: "magY", label: "Mag Y", group: "Magnetometer" },
  { id: "magZ", label: "Mag Z", group: "Magnetometer" },

  { id: "pressure", label: "Pressure", group: "Barometer" },
  { id: "altitude", label: "Altitude", group: "Barometer" },
  { id: "temperature", label: "Temperature", group: "Barometer" },
];

const PROTOCOL_OPTIONS: ProtocolOption[] = ["USB", "UART", "I2C", "SPI"];

function classNames(...classes: Array<string | boolean | null | undefined>) {
  return classes.filter(Boolean).join(" ");
}

export function SettingsPanel() {
  const [deviceName, setDeviceName] = useState("FIRM Flight Computer");
  const [freqInput, setFreqInput] = useState("100");
  const [freqError, setFreqError] = useState<string | null>(null);
  const [protocol, setProtocol] = useState<ProtocolOption>("USB");

  const [selectedFields, setSelectedFields] = useState<Set<TelemetryFieldId>>(
    new Set(["accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ", "pressure", "altitude"]),
  );

  const handleFreqChange = (value: string) => {
    if (value === "" || /^\d*$/.test(value)) {
      setFreqInput(value);
      setFreqError(null);
    }
  };

  const validateFrequency = () => {
    if (freqInput === "") {
      setFreqError("Frequency is required.");
      return;
    }

    const numeric = Number(freqInput);
    if (!Number.isInteger(numeric) || numeric < 1 || numeric > 500) {
      setFreqError("Frequency must be between 1 and 500 Hz.");
    } else {
      setFreqError(null);
    }
  };

  const toggleField = (id: TelemetryFieldId) => {
    setSelectedFields((prev) => {
      const next = new Set(prev);
      next.has(id) ? next.delete(id) : next.add(id);
      return next;
    });
  };

  const handleApply = (event: React.FormEvent) => {
    event.preventDefault();
    validateFrequency();
    const numericFreq = Number(freqInput);
    if (!freqError && numericFreq >= 1 && numericFreq <= 500) {
      const config = {
        deviceName,
        updateFrequencyHz: numericFreq,
        protocol,
        fields: Array.from(selectedFields),
      };
      console.log("Apply FIRM settings:", config);
    }
  };

  const groupedFields: Record<string, TelemetryField[]> = TELEMETRY_FIELDS.reduce(
    (acc, field) => {
      if (!acc[field.group]) acc[field.group] = [];
      acc[field.group].push(field);
      return acc;
    },
    {} as Record<string, TelemetryField[]>,
  );

  const selectedLabels = TELEMETRY_FIELDS.filter((f) => selectedFields.has(f.id)).map(
    (f) => f.label,
  );

  return (
    <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-5 shadow-sm text-slate-900">
      <h2 className="mb-3 text-lg font-semibold leading-tight">FIRM Settings</h2>

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
              value={deviceName}
              onChange={(e) => setDeviceName(e.target.value)}
              className="w-full rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 shadow-inner focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme"
              placeholder="FIRM Flight Computer"
            />
          </div>

          <div className="flex flex-col gap-1">
            <label
              htmlFor="frequency"
              className="text-xs font-semibold uppercase tracking-wide text-slate-500"
            >
              Update Frequency (1–500 Hz)
            </label>
            <input
              id="frequency"
              type="text"
              inputMode="numeric"
              value={freqInput}
              onChange={(e) => handleFreqChange(e.target.value)}
              onBlur={validateFrequency}
              className="w-full rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 shadow-inner focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme"
              placeholder="100"
            />
            {freqError && <p className="text-xs text-red-500">{freqError}</p>}
          </div>
        </div>

        {/* Row 2 — Fields and Protocol */}
        <div className="grid grid-cols-1 gap-4 md:grid-cols-2">
          {/* Data Packet Fields */}
          <div className="flex flex-col gap-2">
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">
              Data Packet Fields
            </p>

            <Menu as="div" className="relative w-full text-left">
              <div>
                <Menu.Button className="inline-flex w-full items-center justify-between rounded-md border border-slate-300 bg-white px-3 py-2 text-sm font-medium text-slate-700 shadow-sm hover:bg-slate-50 focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme">
                  <span>
                    {selectedFields.size === 0
                      ? "Select fields"
                      : `${selectedFields.size} field${
                          selectedFields.size > 1 ? "s" : ""
                        } selected`}
                  </span>
                  <ChevronDown className="ml-2 h-4 w-4 text-slate-500" />
                </Menu.Button>
              </div>

              <Menu.Items className="absolute left-0 z-20 mt-1 w-full origin-top-left rounded-md border border-slate-200 bg-white py-2 shadow-lg focus:outline-none max-h-72 overflow-y-auto">
                {Object.entries(groupedFields).map(([groupName, fields]) => (
                  <div key={groupName} className="py-1">
                    <p className="px-3 pb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-400">
                      {groupName}
                    </p>
                    {fields.map((field) => (
                      <Menu.Item key={field.id}>
                        {({ active }) => (
                          <label
                            className={classNames(
                              "flex cursor-pointer items-center gap-2 px-3 py-1.5 text-sm text-slate-700",
                              active && "bg-slate-50",
                            )}
                          >
                            <input
                              type="checkbox"
                              checked={selectedFields.has(field.id)}
                              onChange={() => toggleField(field.id)}
                              className="h-4 w-4 rounded border-slate-300 accent-theme focus:ring-theme"
                            />
                            <span>{field.label}</span>
                          </label>
                        )}
                      </Menu.Item>
                    ))}
                  </div>
                ))}
              </Menu.Items>
            </Menu>

            {selectedFields.size > 0 && (
              <div className="mt-1 flex flex-wrap gap-1">
                {selectedLabels.map((label) => (
                  <span
                    key={label}
                    className="rounded-full bg-theme px-2 py-0.5 text-xs font-medium text-white shadow-sm"
                  >
                    {label}
                  </span>
                ))}
              </div>
            )}
          </div>

          {/* Protocol */}
          <div className="flex flex-col gap-2">
            <p className="text-xs font-semibold uppercase tracking-wide text-slate-500">Protocol</p>

            <Listbox value={protocol} onChange={setProtocol}>
              <div className="relative w-full">
                <Listbox.Button className="relative w-full cursor-default rounded-md border border-slate-300 bg-white py-2 pl-3 pr-8 text-left text-sm text-slate-800 shadow-sm hover:bg-slate-50 focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme">
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

            <p className="text-xs text-slate-500">
              Interface FIRM uses to communicate with the host.
            </p>
          </div>
        </div>

        {/* Actions */}
        <div className="flex justify-end gap-2 pt-1">
          <button
            type="button"
            onClick={() => {
              setDeviceName("FIRM Flight Computer");
              setFreqInput("100");
              setFreqError(null);
              setProtocol("USB");
              setSelectedFields(
                new Set([
                  "accelX",
                  "accelY",
                  "accelZ",
                  "gyroX",
                  "gyroY",
                  "gyroZ",
                  "pressure",
                  "altitude",
                ]),
              );
            }}
            className="rounded-md border border-slate-300 px-3 py-1.5 text-xs font-medium text-slate-700 hover:bg-slate-50"
          >
            Reset
          </button>
          <button
            type="submit"
            className="rounded-md bg-theme px-4 py-1.5 text-xs font-semibold text-white shadow-sm hover:brightness-110 focus:outline-none focus:ring-2 focus:ring-theme/70 focus:ring-offset-1 focus:ring-offset-white"
          >
            Apply Settings
          </button>
        </div>
      </form>
    </section>
  );
}
