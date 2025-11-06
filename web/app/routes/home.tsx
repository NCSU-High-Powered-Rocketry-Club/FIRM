import React from "react";
import type { Route } from "./+types/home";
import { PageHeader } from "~/components/PageHeader";
import { DeviceInfo } from "~/components/DeviceInfo";
import { GraphsPanel } from "~/components/GraphsPanel";
import {SettingsPanel} from "~/components/SettingsPanel";

export function meta(): Route.MetaDescriptors {
  return [
    { title: "FIRM Dashboard" },
    {
      name: "description",
      content: "Dashboard for the FIRM flight computer.",
    },
  ];
}

function getBodyContainerClasses(isConnected: boolean): string {
  let classes = "mx-auto flex max-w-5xl flex-col gap-4 px-6 py-6 transition-opacity";
  if (!isConnected) {
    classes = classes + " opacity-40 pointer-events-none";
  }
  return classes;
}

export default function Home() {
  // TODO: replace with real connection state
  const isFirmConnected = true;

  const bodyClasses = getBodyContainerClasses(isFirmConnected);

  return (
    <div className="min-h-screen bg-white">
      <PageHeader />

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

        <DeviceInfo />
        <GraphsPanel />
          <SettingsPanel />
      </main>
    </div>
  );
}
