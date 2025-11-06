import React, { useState } from "react";

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

export function SettingsPanel() {
    const [deviceName, setDeviceName] = useState<string>("FIRM Flight Computer");
    const [freqInput, setFreqInput] = useState<string>("100"); // Hz, as string for controlled input
    const [freqError, setFreqError] = useState<string | null>(null);

    const [selectedFields, setSelectedFields] = useState<Set<TelemetryFieldId>>(
        () =>
            new Set<TelemetryFieldId>([
                "accelX",
                "accelY",
                "accelZ",
                "gyroX",
                "gyroY",
                "gyroZ",
                "pressure",
                "altitude",
            ])
    );

    const handleFreqChange = (value: string) => {
        // Allow empty (so user can clear and retype) or digits only
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
            setFreqError("Frequency must be an integer between 1 and 500 Hz.");
        } else {
            setFreqError(null);
        }
    };

    const toggleField = (id: TelemetryFieldId) => {
        setSelectedFields((prev) => {
            const next = new Set(prev);
            if (next.has(id)) {
                next.delete(id);
            } else {
                next.add(id);
            }
            return next;
        });
    };

    const handleApply = (event: React.FormEvent) => {
        event.preventDefault();
        validateFrequency();

        // You could early-return here if freqError is set,
        // but that would require validateFrequency to be sync-safe.
        // For now this just logs the config and leaves real serial
        // wiring for later.
        const numericFreq = Number(freqInput);

        if (!freqError && Number.isInteger(numericFreq) && numericFreq >= 1 && numericFreq <= 500) {
            const config = {
                deviceName,
                updateFrequencyHz: numericFreq,
                fields: Array.from(selectedFields),
            };
            // TODO: send config to FIRM over serial/WebSocket/etc.
            console.log("Apply FIRM settings:", config);
        }
    };

    const groupedFields: Record<string, TelemetryField[]> = TELEMETRY_FIELDS.reduce(
        (acc, field) => {
            if (!acc[field.group]) {
                acc[field.group] = [];
            }
            acc[field.group].push(field);
            return acc;
        },
        {} as Record<string, TelemetryField[]>
    );

    return (
        <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3 pb-5 shadow-sm text-slate-900">
            <h2 className="mb-3 text-lg font-semibold leading-tight">FIRM Settings</h2>
            <form onSubmit={handleApply} className="space-y-4 text-sm">
                {/* Row 1: Device name & frequency */}
                <div className="grid grid-cols-1 gap-4 md:grid-cols-2">
                    <div className="flex flex-col gap-1">
                        <label htmlFor="deviceName" className="text-xs font-medium uppercase tracking-wide text-slate-500">
                            Device Name
                        </label>
                        <input
                            id="deviceName"
                            type="text"
                            value={deviceName}
                            onChange={(e) => setDeviceName(e.target.value)}
                            className="rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 shadow-inner focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme"
                            placeholder="FIRM Flight Computer"
                        />
                    </div>

                    <div className="flex flex-col gap-1">
                        <label htmlFor="frequency" className="text-xs font-medium uppercase tracking-wide text-slate-500">
                            Update Frequency (Hz)
                        </label>
                        <div className="flex items-center gap-2">
                            <input
                                id="frequency"
                                type="text"
                                inputMode="numeric"
                                value={freqInput}
                                onChange={(e) => handleFreqChange(e.target.value)}
                                onBlur={validateFrequency}
                                className="w-28 rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 shadow-inner focus:border-theme focus:outline-none focus:ring-1 focus:ring-theme"
                                placeholder="100"
                            />
                            <span className="text-xs text-slate-500">1–500 Hz</span>
                        </div>
                        {freqError && (
                            <p className="text-xs text-red-500">{freqError}</p>
                        )}
                    </div>
                </div>

                {/* Row 2: Fields selection */}
                <div className="rounded-lg border border-slate-200 bg-slate-50/70 p-4">
                    <p className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-500">
                        Telemetry Fields to Send
                    </p>
                    <div className="grid grid-cols-1 gap-4 md:grid-cols-3">
                        {Object.entries(groupedFields).map(([groupName, fields]) => (
                            <div key={groupName}>
                                <h3 className="mb-1 text-xs font-semibold text-slate-600">
                                    {groupName}
                                </h3>
                                <div className="space-y-1">
                                    {fields.map((field) => (
                                        <label
                                            key={field.id}
                                            className="flex items-center gap-2 text-sm text-slate-700"
                                        >
                                            <input
                                                type="checkbox"
                                                checked={selectedFields.has(field.id)}
                                                onChange={() => toggleField(field.id)}
                                                className="h-4 w-4 rounded border-slate-300 text-theme focus:ring-theme"
                                            />
                                            <span>{field.label}</span>
                                        </label>
                                    ))}
                                </div>
                            </div>
                        ))}
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
                            setSelectedFields(
                                new Set<TelemetryFieldId>([
                                    "accelX",
                                    "accelY",
                                    "accelZ",
                                    "gyroX",
                                    "gyroY",
                                    "gyroZ",
                                    "pressure",
                                    "altitude",
                                ])
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
