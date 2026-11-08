const { chromium } = require('playwright');
(async () => {
  const b = await chromium.launch(); const p = await b.newPage();
  await p.goto('http://127.0.0.1:8765/web/premaid-sim.html');
  await p.waitForFunction(() => !!window.PMSIM, null, {timeout:60000});
  const r = await p.evaluate(() => {
    const api = window.PMSIM.pma, it = window.PMA_LIBRARY[0];
    const bin = atob(it.data), u = new Uint8Array(bin.length); for (let i=0;i<bin.length;i++) u[i]=bin.charCodeAt(i);
    const secs = api.decode(u.buffer), tl = api.timeline(secs[0].bytes, true);
    return { raw: tl.frames.slice(0,3).map(f=>f.raw.slice(0,6)), spd: tl.frames.slice(0,3).map(f=>f.spd), dur: tl.total };
  });
  console.log(JSON.stringify(r,null,2)); await b.close();
})();
