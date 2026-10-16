# FIRM Web App

React + TypeScript frontend for configuring and communicating with FIRM over Web Serial. It lives at `web/` in the FIRM monorepo and depends on the local WASM client (`firm-client` from `../client`, not npm) through the root npm workspace.

You need **Node.js + npm** (https://nodejs.org/en/download) and **wasm-pack**, because `npm ci` runs `client/`'s `prepare` script to build the WASM package.

From the **repository root**:

```bash
just sync-web       # npm ci for the workspace (run once, and after dependency changes)
just web-dev        # Vite dev server (usually http://localhost:5173/)
just build-web      # production build
just lint-web       # ESLint and Prettier (client/ and web/), tsc (web/)
```

Or from this directory, after `just sync-web`:

```bash
npm run dev
npm run build
npm run typecheck
```

Lint and formatting config (`eslint.config.js`, `.prettierrc`) live at the repository root and cover `client/` too; run them with `just lint-web`.

`just web-dev` hot-reloads UI changes. `node_modules/firm-client` is a symlink to `client/`; keep `optimizeDeps.exclude` / `preserveSymlinks` in `vite.config.ts` so Vite follows that link.

To test a production build locally (from the repository root):

```bash
just sync-web build-web
npx http-server web/build/client -p 8080
```

(The Vite base defaults to `/`; set `VITE_BASE` to change it.) Then open http://127.0.0.1:8080/.

Documentation is **not** in this folder. It lives at repo-root `docs/` (`mkdocs.yaml`). From the repository root:

```bash
just build-docs
uv run --group docs mkdocs serve
```

The served docs are at http://127.0.0.1:8000/docs/.

Merged changes on `main` that touch `web/`, `docs/`, `client/`, or `mkdocs.yaml` deploy to GitHub Pages: https://firm.ncsurocketry.org/ (app) and https://firm.ncsurocketry.org/docs/ (docs).
