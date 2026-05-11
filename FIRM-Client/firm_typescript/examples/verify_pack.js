import { execSync } from 'child_process';
import fs from 'fs';
import path from 'path';
import http from 'http';
import { fileURLToPath } from 'url';

// --- CONFIGURATION ---
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT_DIR = path.resolve(__dirname, '../../'); // Go up to FIRM-Client root
const TEMP_DIR = path.join(__dirname, 'temp_install'); // Where we fake the install
const PORT = 8080;

// Colors for console output
const LOG = {
  info: (msg) => console.log(`\x1b[36m[INFO]\x1b[0m ${msg}`),
  success: (msg) => console.log(`\x1b[32m[SUCCESS]\x1b[0m ${msg}`),
  error: (msg) => console.log(`\x1b[31m[ERROR]\x1b[0m ${msg}`),
};

try {
  // 1. CLEANUP PREVIOUS RUNS
  if (fs.existsSync(TEMP_DIR)) {
    fs.rmSync(TEMP_DIR, { recursive: true, force: true });
  }
  fs.mkdirSync(TEMP_DIR);

  // 2. BUILD AND PACK
  LOG.info('Building and Packing project...');
  // Ensure we install deps, build, and then pack.
  // We capture the output filename from npm pack (e.g., firm-client-0.1.5.tgz)
  const packCmd = `npm run build && npm pack --json`;
  const packOutput = execSync(packCmd, { cwd: ROOT_DIR, encoding: 'utf-8' });

  // Parse the JSON output from npm pack to find the filename
  // npm pack --json returns an array of objects
  const packJson = JSON.parse(packOutput.match(/\[.*\]/s)[0]);
  const tarballName = packJson[0].filename;
  const tarballPath = path.join(ROOT_DIR, tarballName);

  LOG.success(`Created tarball: ${tarballName}`);

  // 3. EXTRACT (Simulate Install)
  LOG.info('Extracting to temp_install (simulating npm install)...');
  // We use tar to extract. npm pack creates a 'package' folder inside.
  // We strip that component so it looks like node_modules/firm-client
  const installPath = path.join(TEMP_DIR, 'firm-client');
  fs.mkdirSync(installPath);

  execSync(`tar -xf "${tarballPath}" -C "${installPath}" --strip-components=1`);

  // Clean up the .tgz file
  fs.unlinkSync(tarballPath);

  // 4. VERIFY CRITICAL FILES
  // This is the step that catches your previous bug!
  const wasmPath = path.join(installPath, 'firm_typescript/pkg/firm_client_bg.wasm');
  const jsPath = path.join(installPath, 'firm_typescript/pkg/firm_client.js');

  if (!fs.existsSync(wasmPath) || !fs.existsSync(jsPath)) {
    throw new Error(
      'CRITICAL: WASM/JS files are missing from the packed archive! Check your .npmignore or package.json files list.',
    );
  }
  LOG.success('Verification Passed: WASM and JS files are present in the package.');

  // 5. CREATE TEST PAGE
  // We copy your test.html but inject an Import Map pointing to the unpacked files
  const originalHtml = fs.readFileSync(path.join(__dirname, 'test.html'), 'utf-8');

  // We need to inject the import map before the script tag or in the head
  const importMap = `
    <script type="importmap">
      {
        "imports": {
          "firm-client": "./temp_install/firm-client/firm_typescript/typescript/dist/index.js"
        }
      }
    </script>
    <div style="background:#d4edda; color:#155724; padding:10px; text-align:center; font-weight:bold; border-bottom:1px solid #c3e6cb;">
      ✅ Running against Packed & Extracted Package
    </div>
  `;

  // Insert Import Map into the HTML (replace head tag or prepend to body)
  const testHtml = originalHtml.replace('<head>', `<head>${importMap}`);
  const testHtmlPath = path.join(__dirname, 'test_packed.html');
  fs.writeFileSync(testHtmlPath, testHtml);

  // 6. SERVE
  LOG.info(`Starting test server at http://localhost:${PORT}/test_packed.html`);

  const server = http.createServer((req, res) => {
    // Basic static file server
    const safePath = path.normalize(req.url).replace(/^(\.\.[\/\\])+/, '');
    let filePath = path.join(__dirname, safePath === '/' ? 'test_packed.html' : safePath);

    // Allow serving from the simulated package
    if (req.url.startsWith('/temp_install')) {
      filePath = path.join(__dirname, safePath);
    }

    const ext = path.extname(filePath);
    let contentType = 'text/html';
    if (ext === '.js') contentType = 'text/javascript';
    if (ext === '.wasm') contentType = 'application/wasm';

    fs.readFile(filePath, (err, content) => {
      if (err) {
        if (err.code === 'ENOENT') {
          res.writeHead(404);
          res.end(`File not found: ${filePath}`);
        } else {
          res.writeHead(500);
          res.end(`Server Error: ${err.code}`);
        }
      } else {
        res.writeHead(200, {
          'Content-Type': contentType,
          'Cross-Origin-Opener-Policy': 'same-origin',
          'Cross-Origin-Embedder-Policy': 'require-corp',
        });
        res.end(content);
      }
    });
  });

  server.listen(PORT, () => {
    LOG.success(`Ready! Open http://localhost:${PORT}/test_packed.html in your browser.`);
    LOG.info('Press Ctrl+C to stop.');
  });
} catch (err) {
  LOG.error(err.message);
  if (err.stdout) console.log(err.stdout.toString());
  if (err.stderr) console.log(err.stderr.toString());
  process.exit(1);
}
