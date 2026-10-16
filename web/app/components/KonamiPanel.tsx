import { useEffect, useRef, useState } from "react";
import { isEditableTarget } from "~/contexts/FIRMContext";

const KONAMI_SEQUENCE = [
  "ArrowUp",
  "ArrowUp",
  "ArrowDown",
  "ArrowDown",
  "ArrowLeft",
  "ArrowRight",
  "ArrowLeft",
  "ArrowRight",
  "b",
  "a",
  "Enter",
];

export function KonamiPanel() {
  const [isActive, setIsActive] = useState(false);
  const bufferRef = useRef<string[]>([]);
  const timeoutRef = useRef<number | null>(null);

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Ignore typing in form fields.
      if (isEditableTarget(event.target)) return;

      const next = [...bufferRef.current, event.key];
      if (next.length > KONAMI_SEQUENCE.length) next.shift();
      bufferRef.current = next;

      const matched =
        next.length === KONAMI_SEQUENCE.length &&
        next.every((key, i) => key === KONAMI_SEQUENCE[i]);
      if (!matched) return;

      // The sequence ends with Enter; don't let it also activate a focused button.
      event.preventDefault();
      setIsActive(true);

      // reset timer if spammed
      if (timeoutRef.current !== null) {
        window.clearTimeout(timeoutRef.current);
      }

      timeoutRef.current = window.setTimeout(() => {
        setIsActive(false);
      }, 1000);
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      if (timeoutRef.current !== null) {
        window.clearTimeout(timeoutRef.current);
      }
    };
  }, []);

  if (!isActive) return null;

  return (
    <div className="fixed inset-0 z-[9999] flex items-center justify-center bg-black/70 backdrop-blur-sm pointer-events-none">
      <p className="text-8xl font-extrabold tracking-wide text-white drop-shadow-2xl animate-pulse">
        FIRMED UP 👾
      </p>
    </div>
  );
}
