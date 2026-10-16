# FIRM Web App

React + TypeScript frontend for configuring and communicating with FIRM over Web Serial. It lives at `web/` in the FIRM monorepo and depends on the local WASM client (`firm-client` from `../client`, not npm).

You need **Node.js + npm** (https://nodejs.org/en/download) and **wasm-pack**, because `npm ci` in `client/` runs that package's `prepare` script to build the WASM package.

From the **repository root**:

```bash
just sync-web       # npm ci in client/ and web/ (run once, and after dependency changes)
just web-dev        # Vite dev server (usually http://localhost:5173/)
just build-web      # sync-web, then production build
just lint-web       # ESLint + Prettier check
```

Or from this directory, after `just sync-web`:

```bash
npm run dev
npm run build
npm run lint
npm run format:check
```

`just web-dev` hot-reloads UI changes. The app is wired to `file:../client`; keep `optimizeDeps.exclude` / `preserveSymlinks` in `vite.config.ts` so Vite follows that link.

To test a production build locally (from the repository root):

```bash
just build-web
npx http-server web/build/client -p 8080
```

(`just build-web` already sets the Vite base to `/` by default.) Then open http://127.0.0.1:8080/.

Documentation is **not** in this folder. It lives at repo-root `docs/` (`mkdocs.yaml`). From the repository root:

```bash
just build-docs
uv run --group docs mkdocs serve
```

The served docs are at http://127.0.0.1:8000/docs/.

Merged changes on `main` that touch `web/`, `docs/`, `client/`, or `mkdocs.yaml` deploy to GitHub Pages: https://firm.ncsurocketry.org/ (app) and https://firm.ncsurocketry.org/docs/ (docs).
