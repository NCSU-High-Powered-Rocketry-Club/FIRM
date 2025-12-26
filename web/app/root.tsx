import {
  isRouteErrorResponse,
  Links,
  Meta,
  Outlet,
  Scripts,
  ScrollRestoration,
} from "react-router";

import type { Route } from "./+types/root";
import "./app.css";
import React, { useEffect } from "react";
import { KonamiPanel } from "./components/KonamiPanel";

export const links: Route.LinksFunction = () => [
  { rel: "preconnect", href: "https://fonts.googleapis.com" },
  {
    rel: "preconnect",
    href: "https://fonts.gstatic.com",
    crossOrigin: "anonymous",
  },
  {
    rel: "stylesheet",
    href: "https://fonts.googleapis.com/css2?family=Inter:ital,opsz,wght@0,14..32,100..900;1,14..32,100..900&display=swap",
  },
];

export function Layout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <head>
        <meta charSet="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <title>FIRM Dashboard</title>
        <Meta />
        <Links />
        {/* Fass CSS for the Instant Loader */}
        <style
          dangerouslySetInnerHTML={{
            __html: `
          #initial-loader {
            position: fixed;
            inset: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            background-color: #ffffff;
            z-index: 9999;
            font-family: system-ui, -apple-system, sans-serif;
            transition: opacity 0.5s ease;
          }
          .spinner {
            width: 40px;
            height: 40px;
            border: 4px solid #e2e8f0;
            border-top-color: #f85e39; /* Theme Orange */
            border-radius: 50%;
            animation: spin 0.8s linear infinite;
            margin-bottom: 16px;
          }
          @keyframes spin { to { transform: rotate(360deg); } }
        `,
          }}
        />
      </head>
      <body>
        <div id="initial-loader">
          <div className="spinner"></div>
          <div style={{ color: "#64748b", fontSize: "14px", fontWeight: 500 }}>Loading...</div>
        </div>

        {children}
        <KonamiPanel />
        <ScrollRestoration />
        <Scripts />
      </body>
    </html>
  );
}

export default function App() {
  // Remove the loading screen once React wakes up
  useEffect(() => {
    const loader = document.getElementById("initial-loader");
    if (loader) {
      // Fade out
      loader.style.opacity = "0";
      // Remove from DOM after transition
      const timer = setTimeout(() => {
        loader.remove();
      }, 500);
      return () => clearTimeout(timer);
    }
  }, []);

  return <Outlet />;
}

export function ErrorBoundary({ error }: Route.ErrorBoundaryProps) {
  let message = "Oops!";
  let details = "An unexpected error occurred.";
  let stack: string | undefined;

  if (isRouteErrorResponse(error)) {
    message = error.status === 404 ? "404" : "Error";
    details =
      error.status === 404 ? "The requested page could not be found." : error.statusText || details;
  } else if (import.meta.env.DEV && error && error instanceof Error) {
    details = error.message;
    stack = error.stack;
  }

  return (
    <main className="pt-16 p-4 container mx-auto">
      <h1>{message}</h1>
      <p>{details}</p>
      {stack && (
        <pre className="w-full p-4 overflow-x-auto">
          <code>{stack}</code>
        </pre>
      )}
    </main>
  );
}
