# Contributing to FIRM – Overview

FIRM is a **multi-technology project** that spans:

- Firmware and embedded software,
- PCB design and low-voltage electronics,
- A browser-based web app for configuration and data viewing.

These docs are the **central handbook** for new and existing contributors.

---

## Purpose of These Instructions

The goal is to:

- Give **new members** a clear, structured path to becoming productive.
- Provide **experienced members** with a place to double-check details they might forget.
- Capture the **rationale behind key design decisions** so we don’t lose it.

This is not meant to be a generic “learn to code” or “learn electronics” guide, but a **focused resource for FIRM itself**.

---

## Intended Audience

These pages are for:

- Rocketry team members who want to work on FIRM:
    - Firmware,
    - PCBs / hardware,
    - Web app / tooling,
    - Analysis scripts.
- People with:
    - Basic programming knowledge,
    - Basic Git familiarity,
    - Basic understanding of electronics.

---

## High-Level Architecture

At a very high level, the FIRM ecosystem includes:

- **Hardware** – the FIRM PCB and related circuitry (powered by microcontroller, sensors, power stages, etc.).
- **Firmware** – code running on the microcontroller: sensor drivers, data logging, communication.
- **Web App** – a React/TypeScript app that talks to FIRM over serial (via Web Serial + WASM parser).
- **Analysis tooling** – Python/Rust/other scripts that process data from flights.

Later pages should include diagrams and mermaid charts that show:

- How data flows from sensors → firmware → storage → web app → CSV/plots.
- How different repos and tools relate to each other.

---

## Getting Started as a Contributor

If you’re new to FIRM, a good sequence is:

1. **Skim the user docs**  
   – [Using FIRM – Quick Start](../users/getting-started.md)  
   so you know what the device looks like from an end user’s perspective.

2. **Set up your development environment**  
   – [Environment Setup (VS Code, Python, Git)](environment-setup.md)  
   so your laptop is ready for FIRM development.

3. **Pick an area of focus**:
    - Hardware / KiCad → see [Hardware & KiCad](hardware-kicad.md)
    - Firmware / Web app → see [Firmware & Web App Workflow](dev-workflow.md)

4. Join the relevant channels (Discord, email lists, etc.) and start with small issues or documentation improvements.

---

## Documentation as a Living Resource

Because FIRM touches so many technologies, **no one person remembers everything**. These docs are meant to:

- Capture **hard-won knowledge** (things we had to debug the hard way).
- Stay **up to date** with the current hardware revision and firmware.
- Make onboarding new contributors much easier.

If you notice missing, outdated, or confusing information:

- Please open an issue or a PR.
- Add diagrams, screenshots, or small clarifying paragraphs wherever they help.

---