import re,glob
for f in glob.glob('pma/*.pma'):
    t=open(f,encoding='utf-8-sig',errors='ignore').read()
    m=re.search(r'\u30c7\u30fc\u30bf=([0-9A-Fa-f ]+)',t)
    if not m: continue
    b=bytes(int(x,16) for x in m.group(1).split())
    i=0; c={}
    while i<len(b):
        L=b[i]
        if L in (0,255) or i+L>len(b): break
        if b[i+1]==0x19: c[b[i+2]]=c.get(b[i+2],0)+1
        i+=L
    print(f, c)
