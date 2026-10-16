import js from "@eslint/js";
import prettierConfig from "eslint-config-prettier";
import react from "eslint-plugin-react";
import reactHooks from "eslint-plugin-react-hooks";
import globals from "globals";
import tseslint from "typescript-eslint";

export default tseslint.config(
  {
    ignores: [
      "**/node_modules/",
      "**/dist/",
      "**/build/",
      "**/pkg/",
      "**/.react-router/",
      "target/",
      "site/",
      ".venv/",
    ],
  },
  js.configs.recommended,
  tseslint.configs.recommended,
  {
    languageOptions: {
      globals: globals.browser,
    },
    rules: {
      "@typescript-eslint/no-unused-vars": [
        "warn",
        { argsIgnorePattern: "^_", varsIgnorePattern: "^_" },
      ],
    },
  },
  {
    files: ["web/**/*.{js,jsx,ts,tsx}"],
    extends: [react.configs.flat.recommended, react.configs.flat["jsx-runtime"]],
    plugins: { "react-hooks": reactHooks },
    settings: {
      react: { version: "detect" },
    },
    // The classic hooks rules only; the preset's React Compiler rules would need app refactors.
    rules: {
      "react-hooks/rules-of-hooks": "error",
      "react-hooks/exhaustive-deps": "warn",
    },
  },
  {
    files: ["eslint.config.js", "client/firm_typescript/examples/**/*.js"],
    languageOptions: {
      globals: globals.node,
    },
  },
  prettierConfig,
);
