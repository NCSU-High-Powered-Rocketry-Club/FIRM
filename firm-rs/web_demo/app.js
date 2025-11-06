import init, { FIRM } from "./pkg/firm_rs.js";

async function run() {
  // Initialize the WASM module
  await init();

  const firm = new FIRM();

  const btn = document.getElementById("connect");

  const output = document.getElementById("output");

  btn.addEventListener("click", async () => {
    if (!("serial" in navigator)) {
      output.textContent =
        "Web Serial API not available in this browser. Please use Chrome.";
      return;
    }

    try {
      // Request a port from the user
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });

      const reader = port.readable.getReader();

      // Read loop
      while (true) {
        // Get data from the serial port
        const { value, done } = await reader.read();
        if (done) {
          output.textContent = "Reader closed";
          break;
        }

        // Feed data to the parser
        if (value && value.length > 0) {
          firm.parse_bytes(value);

          while (true) {
            const pkt = firm.get_packet();
            if (pkt === null) break;

            output.textContent = "Packet:\n" + JSON.stringify(pkt, null, 1);
          }
        }
      }

      reader.releaseLock();
    } catch (err) {
      console.error(err);
      output.textContent = "Serial error: " + err;
    }
  });
}

run().catch((e) => console.error(e));
