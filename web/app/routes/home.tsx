import React from "react";
import type { Route } from "./+types/home";
import { Header } from "~/components/Header";
import { DeviceInfoPanel } from "~/components/DeviceInfoPanel";
import { GraphsPanel } from "~/components/GraphsPanel";
import { SettingsPanel } from "~/components/SettingsPanel";
import { ActionsPanel } from "~/components/ActionsPanel";
import {Footer} from "~/components/Footer";

function getBodyContainerClasses(isConnected: boolean): string {
  let classes = "mx-auto flex max-w-5xl flex-col gap-4 px-6 py-6 transition-opacity";
  if (!isConnected) {
    classes = classes + " opacity-40 pointer-events-none";
  }
  return classes;
}

export default function Home() {
  const isFirmConnected = true;
  const bodyClasses = getBodyContainerClasses(isFirmConnected);

  return (
    <div className="min-h-screen bg-white flex flex-col">
      <Header />

      <main className={bodyClasses}>
        {/* Optional little status pill */}
        <div className="mb-2 text-sm text-gray-600">
          <span
            className={
              isFirmConnected
                ? "inline-flex items-center gap-1 rounded-full bg-green-100 px-3 py-1 text-xs font-medium text-green-800"
                : "inline-flex items-center gap-1 rounded-full bg-gray-200 px-3 py-1 text-xs font-medium text-gray-700"
            }
          >
            <span
              className={
                isFirmConnected
                  ? "h-2 w-2 rounded-full bg-green-500"
                  : "h-2 w-2 rounded-full bg-gray-500"
              }
            />
            {isFirmConnected ? "FIRM connected" : "No device connected"}
          </span>
        </div>

        <DeviceInfoPanel />
        <GraphsPanel />
        <SettingsPanel />
        <ActionsPanel />
      </main>

      <Footer />
    </div>
  );
}
