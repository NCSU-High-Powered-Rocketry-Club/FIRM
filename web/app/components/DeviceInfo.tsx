import React from "react";
import { Cpu, Plug, GaugeCircle, Wifi } from "lucide-react";

export function DeviceInfo() {
    const info = {
        deviceName: "FIRM Flight Computer",
        firmware: "v1.3.2",
        connection: "COM3",
        frequency: "500 Hz",
    };

    const items = [
        {
            label: "Device",
            icon: <Cpu className="h-4 w-4 text-theme" />,
            value: info.deviceName,
        },
        {
            label: "Firmware",
            icon: <Wifi className="h-4 w-4 text-theme" />,
            value: info.firmware,
        },
        {
            label: "Port",
            icon: <Plug className="h-4 w-4 text-theme" />,
            value: info.connection,
        },
        {
            label: "Frequency",
            icon: <GaugeCircle className="h-4 w-4 text-theme" />,
            value: info.frequency,
        },
    ];

    return (
        <section className="rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-4 shadow-sm text-slate-900">
            <h2 className="mb-3 text-lg font-semibold leading-tight">Device Info</h2>
            <ul className="flex flex-wrap items-center gap-x-10 gap-y-3 text-sm leading-relaxed">
                {items.map((item) => (
                    <li key={item.label} className="flex items-center gap-2">
                        {item.icon}
                        <span className="text-slate-500">{item.label}:</span>
                        <span className="font-medium">{item.value}</span>
                    </li>
                ))}
            </ul>
        </section>
    );
}
