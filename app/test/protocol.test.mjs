// Składanie ramek protokołu z notyfikacji o różnych rozmiarach (MTU 23 … 247).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Device, BLOCK_MAGIC } from '../js/protocol.js';
import { crc32 } from '../js/format.js';

class FakeTransport {
  constructor(chunk) { this.chunk = chunk; this.connected = true; }
  onFrame(cb) { this.cb = cb; }
  async send(text) {
    const [cmd, seq, off, len] = text.trim().split(' ');
    if (cmd !== 'GET') return;
    const data = new Uint8Array(Number(len)).map((_, i) => (i * 7 + Number(off)) & 0xFF);
    const fr = new Uint8Array(20 + data.length);
    const dv = new DataView(fr.buffer);
    dv.setUint32(0, BLOCK_MAGIC, true); dv.setUint32(4, +seq, true); dv.setUint32(8, +off, true);
    dv.setUint32(12, data.length, true); dv.setUint32(16, crc32(data), true);
    fr.set(data, 20);
    setTimeout(() => {
      for (let i = 0; i < fr.length; i += this.chunk) {
        const part = fr.subarray(i, i + this.chunk);
        const pkt = new Uint8Array(part.length + 1);
        pkt[0] = 0x02; pkt.set(part, 1);
        this.cb(pkt);
      }
    }, 1);
  }
}

for (const chunk of [19, 20, 7, 243]) {
  test(`GET składany z notyfikacji po ${chunk} B`, async () => {
    const d = new Device(new FakeTransport(chunk));
    const a = await d.getBlock(5, 0, 1000);
    const b = await d.getBlock(5, 1000, 33);
    assert.equal(a.length, 1000);
    assert.equal(b.length, 33);
    assert.equal(b[0], 1000 & 0xFF);
  });
}
