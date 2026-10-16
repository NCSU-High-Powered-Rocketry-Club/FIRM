import { useEffect, useRef, useState } from "react";
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
import { useFIRM } from "~/contexts/FIRMContext";
import type { FIRMPacket } from "firm-client";

// Limits how many samples are retained for each chart.
const MAX_HISTORY_POINTS = 60;
// Controls how often the chart re-renders.
const REFRESH_RATE_MS = 50;

// Distinct per-axis colors so the traces can be told apart.
const AXIS_COLORS = { x: "var(--color-theme)", y: "#2563eb", z: "#059669" } as const;

interface GraphDataPoint {
  t: number;
  // Accel
  ax: number;
  ay: number;
  az: number;
  // Mag
  mx: number;
  my: number;
  mz: number;
  // Baro
  pressure: number;
}

// Reusable legend component for consistent styling
const LegendHeader = () => (
  <div className="flex items-center gap-3 text-xs font-normal text-slate-500">
    {(["x", "y", "z"] as const).map((axis) => (
      <span key={axis} className="flex items-center gap-1">
        <span
          className="block h-2 w-2 rounded-full"
          style={{ backgroundColor: AXIS_COLORS[axis] }}
        ></span>{" "}
        {axis.toUpperCase()}
      </span>
    ))}
  </div>
);

const formatTimeLabel = (t: unknown) => `t = ${Number(t).toFixed(3)} s`;

export function GraphsPanel() {
  const { latestPacket } = useFIRM();

  const themeColor = "var(--color-theme)";

  const historyBuffer = useRef<GraphDataPoint[]>([]);
  const [graphData, setGraphData] = useState<GraphDataPoint[]>([]);
  const lastPktTs = useRef<number | null>(null);
  const bufferChanged = useRef(false);

  const processPacket = (pkt: FIRMPacket) => {
    historyBuffer.current.push({
      t: pkt.timestamp_seconds,
      ax: pkt.raw_acceleration_x_gs,
      ay: pkt.raw_acceleration_y_gs,
      az: pkt.raw_acceleration_z_gs,
      mx: pkt.magnetic_field_x_microteslas,
      my: pkt.magnetic_field_y_microteslas,
      mz: pkt.magnetic_field_z_microteslas,
      pressure: pkt.pressure_pascals,
    });

    if (historyBuffer.current.length > MAX_HISTORY_POINTS) {
      historyBuffer.current.shift();
    }
    bufferChanged.current = true;
  };

  // Append a point whenever the latest packet changes
  useEffect(() => {
    if (!latestPacket) {
      // Disconnected: don't carry the previous session into the next one.
      historyBuffer.current = [];
      lastPktTs.current = null;
      bufferChanged.current = true;
      return;
    }

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
      if (bufferChanged.current && timestamp - lastRender > REFRESH_RATE_MS) {
        bufferChanged.current = false;
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
                <YAxis domain={["auto", "auto"]} width={40} tick={{ fontSize: 10 }} />
                <Tooltip
                  contentStyle={{ fontSize: "12px" }}
                  itemStyle={{ padding: 0 }}
                  labelFormatter={formatTimeLabel}
                />
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="ax"
                  stroke={AXIS_COLORS.x}
                  strokeWidth={2}
                  dot={false}
                />
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="ay"
                  stroke={AXIS_COLORS.y}
                  strokeWidth={2}
                  dot={false}
                />
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="az"
                  stroke={AXIS_COLORS.z}
                  strokeWidth={2}
                  dot={false}
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
                <YAxis domain={["auto", "auto"]} width={40} tick={{ fontSize: 10 }} />
                <Tooltip contentStyle={{ fontSize: "12px" }} labelFormatter={formatTimeLabel} />
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="mx"
                  stroke={AXIS_COLORS.x}
                  strokeWidth={2}
                  dot={false}
                />
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="my"
                  stroke={AXIS_COLORS.y}
                  strokeWidth={2}
                  dot={false}
                />
                <Line
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="mz"
                  stroke={AXIS_COLORS.z}
                  strokeWidth={2}
                  dot={false}
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
                <YAxis domain={["auto", "auto"]} width={55} tick={{ fontSize: 10 }} />
                <Tooltip labelFormatter={formatTimeLabel} />
                <Area
                  isAnimationActive={false}
                  type="monotone"
                  dataKey="pressure"
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
