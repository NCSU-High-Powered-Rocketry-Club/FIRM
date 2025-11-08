import { reactRouter } from "@react-router/dev/vite";
import tailwindcss from "@tailwindcss/vite";
import { defineConfig } from "vite";
import tsconfigPaths from "vite-tsconfig-paths";

export default defineConfig(({ command }) => {
  // Use custom base from environment variable
  const prodBase = process.env.VITE_BASE ?? "/FIRM-Web-App/";

  return {
    base: command === "serve" ? "/" : prodBase,
    plugins: [tailwindcss(), reactRouter(), tsconfigPaths()],
    optimizeDeps: {
      exclude: ["firm-client"],
    },
  };
});
