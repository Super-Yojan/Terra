#!/usr/bin/env node
// Execute the same deterministic corpus natively and in WebAssembly.
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
const root = fileURLToPath(new URL('../', import.meta.url));
execFileSync('cargo', ['build', '--locked', '-p', 'terra-runtime', '--example', 'replay_wasm', '--target', 'wasm32-unknown-unknown'], { cwd: root, stdio: ['ignore', 'inherit', 'inherit'] });
const native = JSON.parse(execFileSync('cargo', ['run', '--locked', '--quiet', '-p', 'terra-runtime', '--example', 'replay_native'], { cwd: root, encoding: 'utf8' }));
const bytes = readFileSync(new URL('../target/wasm32-unknown-unknown/debug/examples/replay_wasm.wasm', import.meta.url));
const { instance } = await WebAssembly.instantiate(bytes, {});
if (instance.exports.parity_len() !== native.length) throw new Error('Trace lengths differ');
let maximumError = 0;
for (let i = 0; i < native.length; i++) {
    const actual = instance.exports.parity_value(i);
    const error = Math.abs(actual - native[i]);
    const tolerance = i % 5 === 4 ? 0 : 1e-5;
    if (!Number.isFinite(actual) || error > tolerance) throw new Error(`Parity failure at ${i}: native=${native[i]}, wasm=${actual}`);
    maximumError = Math.max(maximumError, error);
}
console.log(`Native/WebAssembly parity: ${native.length / 5} control ticks; discrete statuses exact; maximum output error ${maximumError}`);
