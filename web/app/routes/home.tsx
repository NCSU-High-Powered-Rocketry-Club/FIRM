import React, { useEffect, useState } from "react";
import { Header } from "~/components/Header";
import { DeviceInfoPanel } from "~/components/DeviceInfoPanel";
import { GraphsPanel } from "~/components/GraphsPanel";
import { DeveloperPanel } from "~/components/DeveloperPanel";
import { SettingsPanel } from "~/components/SettingsPanel";
import { ActionsPanel } from "~/components/ActionsPanel";
import { Footer } from "~/components/Footer";
import { ConnectionBar } from "~/components/ConnectionBar";
import { FIRMProvider, useFIRM, isEditableTarget } from "~/contexts/FIRMContext";

function getBodyContainerClasses(isConnected: boolean): string {
  let classes = "mx-auto flex max-w-5xl flex-col gap-4 px-6 py-6 transition-opacity";
  if (!isConnected) {
    classes = classes + " opacity-40 pointer-events-none";
  }
  return classes;
}

export default function Home() {
  return (
    <FIRMProvider>
      <HomeContent />
    </FIRMProvider>
  );
}

function HomeContent() {
  const { isConnected } = useFIRM();
  const bodyClasses = getBodyContainerClasses(isConnected);
  const [showDev, setShowDev] = useState(false);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (isEditableTarget(e.target)) return;
      if (e.key !== "d" && e.key !== "D") return;
      if (e.ctrlKey || e.metaKey || e.altKey) return;

      e.preventDefault();
      setShowDev((v) => !v);
    };

    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, []);

  return (
    <div className="min-h-screen bg-white flex flex-col">
      <Header />
      <ConnectionBar />
      <main className={bodyClasses}>
        <DeviceInfoPanel />
        <GraphsPanel />
        <DeveloperPanel visible={showDev} />
        <SettingsPanel />
        <ActionsPanel />
      </main>
      <Footer />
    </div>
  );
}
