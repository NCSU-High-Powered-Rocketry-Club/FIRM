import React from "react";
import { Rocket } from "lucide-react";

export function Footer() {
  return (
    <footer className="bg-theme text-white mt-10 pt-3 pb-3">
      <div className="mx-auto max-w-5xl px-6 py-4 text-center">
        <div className="flex flex-col items-center gap-1">
          <div className="flex items-center gap-2">
            <Rocket className="h-4 w-4 text-white" />
            <p className="font-medium">NCSU High-Powered Rocketry Club</p>
          </div>
          <p className="text-xs opacity-90">© 2025 FIRM Dashboard</p>
        </div>
      </div>
    </footer>
  );
}
