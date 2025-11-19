# FIRM Web App

The FIRM Web App is a React + TypeScript frontend for configuring and communicating with FIRM over Web Serial. It also contains the FIRM docs site, and eventually will include more features/pages for visualizing flight data.

To work on this project, you will need **Node.js + npm** (install here: https://nodejs.org/en/download) and **Python with uv** for the docs (install uv here: https://docs.astral.sh/uv/getting-started/installation/).

---

# Getting Started

## 1. Working on the React Web App

This folder contains the UI for connecting to and configuring FIRM as well as viewing live data.

### Install dependencies

```bash
npm install
```

### Run the dev server locally

```bash
npm run dev
```

This starts Vite and serves the app at a URL printed in the terminal (usually http://localhost:5173/). Any code changes to the UI will hot-reload.

---

## 2. Working on the Documentation (MkDocs)

The docs live under the top-level `docs/` folder and are built using **MkDocs + Material** and managed through **uv**.

### Install the Python dependencies

```bash
uv sync
```

### Run the docs locally

```bash
uv run mkdocs serve
```

This launches a local dev server (http://127.0.0.1:8000/docs/) where your changes auto-reload as you edit markdown files.

---

To publish changes to the web app or docs, create a PR and merge it to `main`. Then you can view the web app at https://firm.ncsurocketry.org/ and the docs at https://firm.ncsurocketry.org/docs/.
