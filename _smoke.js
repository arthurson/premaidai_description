const { chromium } = require('playwright');

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  const errors = [];
  page.on('console', m => { if (m.type() === 'error') errors.push('console: ' + m.text()); });
  page.on('pageerror', e => errors.push('pageerror: ' + e.message));
  await page.goto('http://127.0.0.1:8765/web/premaid-sim.html');
  await page.waitForFunction(() => !!window.PMSIM, null, { timeout: 60000 });
  await page.waitForTimeout(500);
  const result = await page.evaluate(() => {
    const out = { errors: [], items: [], builtin: null, packets: null, ack: null, info: null };
    const api = window.PMSIM.pma, lib = window.PMA_LIBRARY;
    out.libLen = lib ? lib.length : 0;
    if (!lib) return out;
    function b64bytes(b) { const bin = atob(b), u = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return u; }
    lib.forEach(function (item) {
      let secs;
      try { secs = api.decode(b64bytes(item.data).buffer); } catch (e) { out.errors.push(item.title + ': decode ' + e.message); return; }
      const rec = { title: item.title, sections: secs.length, ok: true };
      if (!secs.length) { rec.ok = false; out.errors.push(item.title + ': no sections'); out.items.push(rec); return; }
      let origN = 0, rawN = 0, loopN = 0, repeats = 0, depth = 0, stretchLo = 255, stretchHi = 0, speedLo = 255, speedHi = 0;
      secs.forEach(function (sec) {
        const a = api.timeline(sec.bytes, false), b = api.timeline(sec.bytes, true);
        origN += a.frames.length; rawN += b.frames.length; loopN += a.loops.length; repeats += a.loopRepeats; depth = Math.max(depth, a.maxLoopDepth);
        const N = a.frames[0].raw.length;
        a.frames.forEach(function (f) { for (let s = 0; s < N; s++) { if (f.stretch[s] < stretchLo) stretchLo = f.stretch[s]; if (f.stretch[s] > stretchHi) stretchHi = f.stretch[s]; if (f.speed[s]) { if (f.speed[s] < speedLo) speedLo = f.speed[s]; if (f.speed[s] > speedHi) speedHi = f.speed[s]; } } });
        rec.checksumBad = (rec.checksumBad || 0) + a.bad;
        rec.other = (rec.other || 0) + a.other;
        rec.dur = (rec.dur || 0) + a.total / 1000;
      });
      rec.frames = origN; rec.rawFrames = rawN; rec.loops = loopN; rec.loopRepeats = repeats; rec.maxLoopDepth = depth;
      rec.stretch = [stretchLo, stretchHi]; rec.speed = speedHi ? [speedLo, speedHi] : null;
      rec.metaFrames = item.frames; rec.metaRawFrames = item.rawFrames; rec.metaLoops = item.loops; rec.metaRepeats = item.loopRepeats; rec.metaDepth = item.maxLoopDepth;
      rec.match = origN === item.frames && rawN === item.rawFrames && loopN === item.loops && repeats === item.loopRepeats && depth === item.maxLoopDepth;
      if (!rec.match) { rec.ok = false; out.errors.push(item.title + ': metadata mismatch'); }
      out.items.push(rec);
    });
    const bsel = document.getElementById('rbBuiltin');
    out.builtin = bsel ? { count: bsel.options.length, first: bsel.options[0].value, last: bsel.options[bsel.options.length - 1].value, labels: Array.from(bsel.options).slice(0, 3).map(o => o.textContent) } : null;
    const stopBtn = document.getElementById('rbStopMotion');
    out.stopDisabled = stopBtn.disabled;
    // packet capture: built-in motion + STOP
    const cap = [];
    window.PMSIM.rb.state.writer = { write: function (u8) { cap.push(Array.from(u8)); return { catch: function () {} }; } };
    window.PMSIM.rb.send(0x3D);
    window.PMSIM.rb.stop();
    window.PMSIM.rb.state.writer = null;
    out.packets = cap;
    out.packetMatch = cap.length === 2 && cap[0].join(',') === '5,31,0,61,39' && cap[1].join(',') === '4,5,0,1';
    // ack handling (success = bit 7 clear, failure = bit 7 set)
    window.PMSIM.rb.rx(Uint8Array.from([4, 0x1F, 0x80, 0x9B]));
    out.ackFail = document.getElementById('rbMotion').textContent;
    window.PMSIM.rb.rx(Uint8Array.from([4, 0x1F, 0x00, 0x1B]));
    out.ackOk = document.getElementById('rbMotion').textContent;
    // synthetic 0x19 + loop packets
    function pkt(cmd, payload) { const b = [3 + payload.length, cmd].concat(payload); let x = 0; for (let i = 0; i < b.length; i++) x ^= b[i]; b.push(x); return b; }
    function frame(raw, spd) { return pkt(0x18, [0x00, spd, 0x02, raw & 255, (raw >> 8) & 255]); }
    const bytes = Uint8Array.from([].concat(
      frame(8000, 10), frame(8100, 10),
      pkt(0x02, [0x00, 0x06, 0x00, 0x03, 0x00]),
      frame(8200, 10),
      pkt(0x07, [0x02, 0x06, 0x00, 0x01, 0x00]),
      frame(8300, 10)
    ));
    const syn = api.timeline(bytes, false);
    out.synthetic = {
      frames: syn.frames.length, rawN: syn.rawN, repeats: syn.loopRepeats, depth: syn.maxLoopDepth, total: syn.total,
      raw: syn.frames.map(f => f.raw[3]),
      match: syn.frames.length === 8 && syn.rawN === 4 && syn.loopRepeats === 2 && syn.maxLoopDepth === 1 && syn.frames[3].raw[3] === 8100 && syn.frames[7].raw[3] === 8300 && syn.total === 1200
    };
    const bytes19 = Uint8Array.from([].concat(
      frame(8000, 10),
      pkt(0x19, [0x00, 0x02, 0x1E]),
      frame(8100, 10),
      pkt(0x19, [0x10, 0x02, 0x0F]),
      frame(8200, 10)
    ));
    const syn19 = api.timeline(bytes19, false);
    out.synthetic19 = {
      speed: syn19.frames.map(f => f.speed[3]),
      stretch: syn19.frames.map(f => f.stretch[3]),
      match: syn19.frames[0].speed[3] === 0 && syn19.frames[1].speed[3] === 30 && syn19.frames[2].speed[3] === 30 && syn19.frames[0].stretch[3] === 60 && syn19.frames[1].stretch[3] === 60 && syn19.frames[2].stretch[3] === 15
    };
    // loop mode UI: "once" must drop the duplicate frames
    const item = lib.find(x => x.loopRepeats > 0);
    api.load(api.decode(b64bytes(item.data).buffer), item.pma, item.bpm || 0);
    const origN = api.P.frames.length;
    const sel = document.getElementById('pLoopMode');
    sel.value = 'once'; sel.dispatchEvent(new Event('change'));
    const onceN = api.P.frames.length;
    sel.value = 'orig'; sel.dispatchEvent(new Event('change'));
    const backN = api.P.frames.length;
    out.loopMode = { origN, onceN, backN, match: onceN === item.rawFrames && backN === origN };
    out.info = { title: item.title, frames: api.P.frames.length, stretch: api.P.infoD.stretch, speed: api.P.infoD.speed, bpm: api.P.infoD.bpm, secs: api.P.secs.length, loopMode: document.getElementById('pLoopMode').value };
    return out;
  });
  console.log(JSON.stringify({ errors, ...result }, null, 2));
  await browser.close();
})().catch(e => { console.error(e); process.exit(1); });



