import os,sys,tempfile,shutil
import xml.etree.ElementTree as ET
import xacro
PKG=os.getcwd()
tmp=tempfile.mkdtemp(prefix='premaidai_check_')
try:
    for d in ('robots','urdf'):
        shutil.copytree(os.path.join(PKG,d),os.path.join(tmp,d))
    for root,_,files in os.walk(tmp):
        for f in files:
            p=os.path.join(root,f)
            s=open(p,encoding='utf-8').read()
            open(p,'w',encoding='utf-8').write(s.replace('$(find premaidai_description)',tmp))
    text=xacro.process_file(os.path.join(tmp,'robots','premaidai.urdf.xacro')).toxml()
finally:
    shutil.rmtree(tmp,ignore_errors=True)
r=ET.fromstring(text)
joints=r.findall('joint'); links=r.findall('link'); trs=r.findall('transmission')
print('links',len(links),'joints',len(joints),'trans',len(trs),'types',sorted(set(j.get('type') for j in joints)))
print('trans missing name',sum(1 for t in trs if not t.get('name')))
print('trans missing joint',sum(1 for t in trs if t.find('joint') is None))
print('first links',[l.get('name') for l in links[:6]])
