import React, { useEffect, useRef, useState } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  AreaChart,
  Area,
} from "recharts";
import { useFirm } from "~/contexts/FirmContext";
import type { FIRMPacket } from "firm-client";

// CONFIGURATION
const MAX_HISTORY_POINTS = 60;
const REFRESH_RATE_MS = 50;

interface GraphDataPoint {
  t: string;
  // Accel
  ax: number;
  ay: number;
  az: number;
  // Mag
  mx: number;
  my: number;
  mz: number;
  // Baro
  alt: number;
}

// Reusable legend component for consistent styling
const LegendHeader = () => (
  <div className="flex items-center gap-3 text-xs font-normal text-slate-500">
    <span className="flex items-center gap-1">
      <span
        className="block h-2 w-2 rounded-full bg-[var(--color-theme)]"
        style={{ filter: "brightness(1.2)" }}
      ></span>{" "}
      X
    </span>
    <span className="flex items-center gap-1">
      <span className="block h-2 w-2 rounded-full bg-[var(--color-theme)]"></span> Y
    </span>
    <span className="flex items-center gap-1">
      <span
        className="block h-2 w-2 rounded-full bg-[var(--color-theme)]"
        style={{ filter: "brightness(0.8)" }}
      ></span>{" "}
      Z
    </span>
  </div>
);

export function GraphsPanel() {
  const { latestPacket } = useFirm();

  // Theme Color (We use CSS filters to make the Light/Dark variants)
  // Ensure this CSS variable is defined in your global CSS or parent component
  const themeColor = "var(--color-theme)";

  const historyBuffer = useRef<GraphDataPoint[]>([]);
  const [graphData, setGraphData] = useState<GraphDataPoint[]>([]);
  const lastPktTs = useRef<number | null>(null);

  const processPacket = (pkt: FIRMPacket) => {
    const tLabel = pkt.timestamp_seconds.toFixed(2);

    // Push to History
    historyBuffer.current.push({
      t: tLabel,
      ax: pkt.raw_acceleration_x_gs,
      ay: pkt.raw_acceleration_y_gs,
      az: pkt.raw_acceleration_z_gs,
      mx: pkt.magnetic_field_x_microteslas,
      my: pkt.magnetic_field_y_microteslas,
      mz: pkt.magnetic_field_z_microteslas,
      alt: pkt.pressure_pascals,
    });

    if (historyBuffer.current.length > MAX_HISTORY_POINTS) {
      historyBuffer.current.shift();
    }
  };

  // Append a point whenever the latest packet changes
  useEffect(() => {
    if (!latestPacket) return;

    // Avoid duplicating the same packet if state updates with identical value
    const ts = latestPacket.timestamp_seconds;
    if (lastPktTs.current === ts) return;
    lastPktTs.current = ts;

    processPacket(latestPacket);
  }, [latestPacket]);

  // Render throttle independent of packet arrival rate
  useEffect(() => {
    let animationFrameId: number;
    let lastRender = 0;

    const loop = (timestamp: number) => {
      if (timestamp - lastRender > REFRESH_RATE_MS) {
        setGraphData([...historyBuffer.current]);
        lastRender = timestamp;
      }
      animationFrameId = requestAnimationFrame(loop);
    };

    animationFrameId = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(animationFrameId);
  }, []);

  return (
    <section className="mt-4 rounded-xl border border-slate-300 bg-white px-6 pt-3.5 pb-4 shadow-sm text-slate-900">
      <h2 className="mb-3 text-lg font-semibold leading-tight flex justify-between items-center">
        <span>Live Data</span>
      </h2>

      <div className="grid grid-cols-1 gap-4 md:grid-cols-3">
        {/* --- 1. IMU Graph (Line Chart) --- */}
        <div className="flex flex-col rounded-lg border border-slate-300 bg-slate-50/80 p-4 relative">
          <div className="flex justify-between items-center mb-2">
            <h3 className="text-sm font-semibold text-slate-800">Accelerometer (gs)</h3>
            <LegendHeader />
          </div>
          <div className="h-48" style={{ color: themeColor }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={graphData}>
                <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                <XAxis dataKey="t" tick={false} />
                <YAxis domain={["auto", "auto"]} width={30} tick={{ fontSize: 10 }} />
                <Tooltip contentStyle={{ fontSize: "12px" }} itemStyle={{ padding: 0 }} />

                {/* X Axis: Light (1.2) */}
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="ax"
                  stroke="currentColor"
                  strokeWidth={2}
                  dot={false}
                  style={{ filter: "brightness(1.2)" }}
                />
                {/* Y Axis: Base Theme Color */}
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="ay"
                  stroke="currentColor"
                  strokeWidth={2}
                  dot={false}
                />
                {/* Z Axis: Dark (0.8) */}
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="az"
                  stroke="currentColor"
                  strokeWidth={2}
                  dot={false}
                  style={{ filter: "brightness(0.8)" }}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>

        {/* --- 2. Magnetometer (Line Chart) --- */}
        <div className="flex flex-col rounded-lg border border-slate-300 bg-slate-50/80 p-4 relative overflow-hidden">
          <div className="flex justify-between items-center mb-2">
            <h3 className="text-sm font-semibold text-slate-800 z-10">Magnetometer (µT)</h3>
            <LegendHeader />
          </div>

          <div className="h-48" style={{ color: themeColor }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={graphData}>
                <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                <XAxis dataKey="t" tick={false} />
                <YAxis domain={["auto", "auto"]} width={30} tick={{ fontSize: 10 }} />
                <Tooltip contentStyle={{ fontSize: "12px" }} />

                {/* X Axis: Light (1.2) */}
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="mx"
                  stroke="currentColor"
                  strokeWidth={2}
                  dot={false}
                  style={{ filter: "brightness(1.2)" }}
                />
                {/* Y Axis: Base */}
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="my"
                  stroke="currentColor"
                  strokeWidth={2}
                  dot={false}
                />
                {/* Z Axis: Dark (0.8) */}
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="mz"
                  stroke="currentColor"
                  strokeWidth={2}
                  dot={false}
                  style={{ filter: "brightness(0.8)" }}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>

        {/* --- 3. Barometer (Area Chart) --- */}
        <div className="flex flex-col rounded-lg border border-slate-300 bg-slate-50/80 p-4">
          <h3 className="mb-2 text-sm font-semibold text-slate-800">Pressure (Pa)</h3>
          <div className="h-48">
            <ResponsiveContainer width="100%" height="100%">
              <AreaChart data={graphData}>
                <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                <XAxis dataKey="t" tick={false} />
                <YAxis domain={["auto", "auto"]} width={30} tick={{ fontSize: 10 }} />
                <Tooltip />
                <Area
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="alt"
                  stroke={themeColor}
                  fill={themeColor}
                  fillOpacity={0.2}
                />
              </AreaChart>
            </ResponsiveContainer>
          </div>
        </div>
      </div>
    </section>
  );
}
