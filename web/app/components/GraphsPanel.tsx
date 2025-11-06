import React from "react";
import {
    LineChart,
    Line,
    XAxis,
    YAxis,
    CartesianGrid,
    Tooltip,
    ResponsiveContainer,
    RadarChart,
    PolarGrid,
    PolarAngleAxis,
    Radar,
    Legend,
    AreaChart,
    Area,
} from "recharts";

const imuData = [
    { t: 0, ax: 0.1, ay: 0.0, az: 1.0, gx: 0.0, gy: 0.0, gz: 0.05 },
    { t: 1, ax: 0.2, ay: -0.1, az: 0.98, gx: 0.01, gy: 0.02, gz: 0.04 },
    { t: 2, ax: 0.25, ay: -0.05, az: 1.02, gx: 0.02, gy: 0.03, gz: 0.03 },
    { t: 3, ax: 0.15, ay: 0.05, az: 0.99, gx: 0.01, gy: 0.01, gz: 0.02 },
];

const magData = [
    { axis: "X", value: 30 },
    { axis: "Y", value: 18 },
    { axis: "Z", value: 42 },
];

const baroData = [
    { t: 0, pressure: 1012.4, altitude: 210.0 },
    { t: 1, pressure: 1012.1, altitude: 211.5 },
    { t: 2, pressure: 1011.8, altitude: 213.0 },
    { t: 3, pressure: 1011.5, altitude: 215.0 },
];

export function GraphsPanel() {
    // If your theme var is called --color-brand instead, swap it here.
    const themeColor = "var(--color-theme)";

    return (
        <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-4 shadow-sm text-slate-900">
            <h2 className="mb-3 text-lg font-semibold leading-tight">Live Graphs</h2>
            <div className="grid grid-cols-1 gap-4 md:grid-cols-3">
                {/* IMU Graph */}
                <div className="flex flex-col rounded-lg border border-slate-300 bg-slate-50/80 p-4">
                    <h3 className="mb-2 text-sm font-semibold text-slate-800">
                        IMU (Accel &amp; Gyro)
                    </h3>
                    <div className="h-48" style={{ color: themeColor }}>
                        <ResponsiveContainer width="100%" height="100%">
                            <LineChart data={imuData}>
                                <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                                <XAxis dataKey="t" tickLine={false} />
                                <YAxis tickLine={false} />
                                <Tooltip />
                                {/* Theme-colored primary line */}
                                <Line
                                    type="monotone"
                                    dataKey="ax"
                                    stroke="currentColor"
                                    strokeWidth={2}
                                    dot={false}
                                    name="Ax"
                                />
                                {/* Subtle secondary lines */}
                                <Line
                                    type="monotone"
                                    dataKey="ay"
                                    stroke="#9ca3af"
                                    strokeWidth={1}
                                    dot={false}
                                    name="Ay"
                                />
                                <Line
                                    type="monotone"
                                    dataKey="az"
                                    stroke="#6b7280"
                                    strokeWidth={1}
                                    dot={false}
                                    name="Az"
                                />
                            </LineChart>
                        </ResponsiveContainer>
                    </div>
                </div>

                {/* Magnetometer Graph */}
                <div className="flex flex-col rounded-lg border border-slate-300 bg-slate-50/80 p-4">
                    <h3 className="mb-2 text-sm font-semibold text-slate-800">
                        Magnetometer
                    </h3>
                    <div className="h-48">
                        <ResponsiveContainer width="100%" height="100%">
                            <RadarChart data={magData}>
                                <PolarGrid stroke="#e5e7eb" />
                                <PolarAngleAxis dataKey="axis" />
                                <Radar
                                    name="Field"
                                    dataKey="value"
                                    stroke={themeColor}
                                    fill={themeColor}
                                    fillOpacity={0.25}
                                />
                                <Legend />
                            </RadarChart>
                        </ResponsiveContainer>
                    </div>
                </div>

                {/* Barometer Graph */}
                <div className="flex flex-col rounded-lg border border-slate-300 bg-slate-50/80 p-4">
                    <h3 className="mb-2 text-sm font-semibold text-slate-800">
                        Barometer
                    </h3>
                    <div className="h-48">
                        <ResponsiveContainer width="100%" height="100%">
                            <AreaChart data={baroData}>
                                <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                                <XAxis dataKey="t" tickLine={false} />
                                <YAxis tickLine={false} />
                                <Tooltip />
                                <Area
                                    type="monotone"
                                    dataKey="altitude"
                                    stroke={themeColor}
                                    fill={themeColor}
                                    fillOpacity={0.18}
                                    name="Altitude (m)"
                                />
                            </AreaChart>
                        </ResponsiveContainer>
                    </div>
                </div>
            </div>
            <p className="mt-2 text-xs text-slate-500">
                Placeholder data shown. These panels will stream live values from the
                flight computer.
            </p>
        </section>
    );
}
