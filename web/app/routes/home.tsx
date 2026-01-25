import React, { useEffect, useState } from "react";
import { Header } from "~/components/Header";
import { DeviceInfoPanel } from "~/components/DeviceInfoPanel";
import { GraphsPanel } from "~/components/GraphsPanel";
import { DeveloperPanel } from "~/components/DeveloperPanel";
import { SettingsPanel } from "~/components/SettingsPanel";
import { ActionsPanel } from "~/components/ActionsPanel";
import { Footer } from "~/components/Footer";
import { FirmConnectionBar } from "~/components/FirmConnectionBar";
import { FirmProvider, useFirm, isEditableTarget } from "~/contexts/FirmContext";

function getBodyContainerClasses(isConnected: boolean): string {
  let classes = "mx-auto flex max-w-5xl flex-col gap-4 px-6 py-6 transition-opacity";
  if (!isConnected) {
    classes = classes + " opacity-40 pointer-events-none";
  }
  return classes;
}

export default function Home() {
  return (
    <FirmProvider>
      <HomeContent />
    </FirmProvider>
  );
}

// Real page content: consumes the firm state
function HomeContent() {
  const { isConnected } = useFirm();
  const bodyClasses = getBodyContainerClasses(isConnected);
  const [showDev, setShowDev] = useState(false);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      // Ignore shortcuts while typing in inputs, textareas, selects, or contenteditable.
      if (isEditableTarget(e.target)) return;

      if (e.key !== "d" && e.key !== "D") return;
      if (e.ctrlKey || e.metaKey || e.altKey) return;

      // Optional: prevent accidental browser behavior for this key
      e.preventDefault();
      setShowDev((v) => !v);
    };

    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, []);

  return (
    <div className="min-h-screen bg-white flex flex-col">
      <Header />

      {/* Pill + button, always clickable */}
      <FirmConnectionBar />

      {/* Panels that get dimmed when disconnected */}
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
