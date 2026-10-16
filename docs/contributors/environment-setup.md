# Contributing - Environment Setup

This page assumes you're already _a little familiar_ with **Git, VS Code, and programming in general**.

You do _not_ need to be an expert, but you should know how to clone a repo, open a project in VS Code, and install basic tools. Each section below briefly explains how to set up your environment for the different parts of the FIRM ecosystem.

---

# 💡 Firmware Development

This is for contributors who want to work on the **embedded firmware** that runs on the FIRM device itself, written in **C**.

Follow the setup instructions from the main FIRM repo:  
**[https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM)**

---

# 🖥️ FIRM Client Development

The FIRM Client handles **parsing live FIRM data** and packing it into an easy-to-use format. Its core is written in **Rust**, with libraries for **Python, JavaScript/TypeScript, and (eventually) Arduino**.

It lives in the `client/` folder of the FIRM repo. Follow the setup instructions in
**[client/README.md](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/blob/main/client/README.md)**

---

# 🌐 Web App Development

The FIRM Web App lets users configure the device and view live data directly in the browser. It's written in **TypeScript** and uses **React** and **Tailwind CSS**.

It lives in the `web/` folder of the FIRM repo. Follow the setup instructions in
**[web/README.md](https://github.com/NCSU-High-Powered-Rocketry-Club/FIRM/blob/main/web/README.md)**

---

If you're unsure which area to start with, look at each repo's Issues page to see what's currently being worked on, and what you think you can help with/would be interested in.
