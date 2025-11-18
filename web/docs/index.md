# FIRM Documentation

Welcome to the documentation for **FIRM**--the completely custom flight computer aimed at being a highly accurate,
low-cost, and plug-and-play solution for any hobby rocketry data collection needs.

## What Is FIRM?

FIRM (Filtered Inertial Rotation Module) is a fully custom **flight computer** designed to:

- Measure acceleration, rotation, pressure, temperature, and magnetic field.
- Deliver high-quality, low-cost flight data with accuracy and precision on par or better than commercial solutions.
- Be extremely user-friendly and versatile, allowing for easy integration into any type of rocket.

---

## Two Ways to Use These Docs

Most people will come here in one of two roles:

### 👾 I Just Want to Use FIRM

If you just want to use FIRM as an end user to collect flight data, you can skip the contributor docs and jump straight
to the user docs.

Start here:

- **[Quick Start](users/getting-started.md)** - what FIRM is, what you need, and how to use it.
- **[Configuring Your Device](users/configuring-firm.md)** - using the web app to view live data and edit settings.
- **[Logging & Downloading Data](users/data-logging.md)** - how to run a collect and download flight data.
- **[Troubleshooting](users/troubleshooting.md)** - common problems and how to fix them.

---

### 🛠️ I Want to Contribute to FIRM

Developing FIRM is a collaborative effort involving many technologies and disciplines. If you're interested in
programming, the different parts of FIRM span many programming languages including C, Rust, Python, and TypeScript. If
you're interested in hardware design, FIRM's PCB designs are created using KiCad.

Start here:

- **[Contributor Overview](contributors/overview.md)** – what FIRM is as a system and how all the pieces fit together.
- **[Environment Setup (VS Code, Python, Git)](contributors/environment-setup.md)** – how to set up your development
  environment(s).
- **[Hardware & KiCad](contributors/kidcad-setup.md)** – how we organize the KiCad projects, conventions, and best
  practices.

---

## Safety

FIRM is a delicate piece of hardware, you have to be careful about how you handle it.

Make sure you're:

- Cautious about not shorting things on the board.
- Protecting the board inside the rocket during flight.
- Not abusing USB ports or power supplies.

---

These docs are a work in progress and are hosted in
our [FIRM-Web-App repo](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM-Web-App). If anything feels confusing,
incomplete, or out of date, please open an issue or PR - future you (and other members) will appreciate it.

---

## What These Docs Are (and Aren’t)

These docs are:

- An **end-user guide** for people who want to use FIRM to collect flight data.
- A **handbook for new members** who want to help contribute to FIRM.
- A **quick-reference** for experienced contributors who forgot some detail.
- A place to store the **rationale behind design decisions** so we don’t lose that context in Slack or random Google
  Docs.

These docs are **not**:

- A full electronics or programming course.
- A step-by-step "how to build FIRM from scratch" tutorial _(this is subject to change)_.

Instead, they focus on _our_ implementation and choices.

---
