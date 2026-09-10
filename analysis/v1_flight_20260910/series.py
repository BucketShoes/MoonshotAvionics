import struct, csv
import os,sys
BIN = os.environ.get("MOONSHOT_BIN") or (sys.argv[1] if len(sys.argv)>1 else "moonshot-fetch.bin")
d=open(BIN,'rb').read()
recs=[]; p=0
while p+10<=len(d):
    rec=struct.unpack_from('<I',d,p)[0]; ln=d[p+4]
    snr=struct.unpack_from('<b',d,p+5)[0]; ts=struct.unpack_from('<I',d,p+6)[0]
    recs.append((rec,ln,snr,ts,d[p+10:p+10+ln])); p+=10+ln
byrec={r[0]:r for r in recs}
LAUNCH=797414
rows=[]
for i in range(474182,476445):
    r=byrec[i]
    if r[1]==0: continue
    t=r[4][0]; pl=r[4]; ts=r[3]; row={'rec':i,'ts':ts,'T':(ts-LAUNCH)/1000.0}
    if t==0xAF:
        dev,latf,lonf,fus,flags=struct.unpack_from('<BHHhH',pl,1)
        row.update(kind='hdr',fusion=fus,phase=flags&0xF,armed=(flags>>4)&1,p1=(flags>>5)&1,p2=(flags>>6)&1,p3=(flags>>7)&1)
    elif t==0x02:
        alt,vv,gnd=struct.unpack_from('<ihh',pl,1)
        row.update(kind='baro',baro_m=alt/100.0,vv=vv/10.0,gnd=gnd)
    elif t==0x04:
        x,y,z=struct.unpack_from('<hhh',pl,1); row.update(kind='acc',ax=x,ay=y,az=z)
    elif t==0x05:
        x,y,z=struct.unpack_from('<hhh',pl,1); row.update(kind='gyr',gx=x/10,gy=y/10,gz=z/10)
    elif t==0x09:
        a,ac,vv=struct.unpack_from('<iHh',pl,1); row.update(kind='peak',pk_alt=a/100.0,pk_acc=ac/100.0,pk_vv=vv/10.0)
    elif t==0x0B:
        ms,fl=struct.unpack_from('<iH',pl,1); row.update(kind='flight',ms_launch=ms,pflags=fl)
    elif t==0x0F:
        fl,ch,dur,hv=struct.unpack_from('<BBHH',pl,1); row.update(kind='pyro',pyflags=fl,pych=ch,pydur=dur,hv=hv)
    elif t==0x07:
        a,v,au,vu,cu,inn=struct.unpack_from('<ihBBBB',pl,1); row.update(kind='kal',k_alt=a/100.0,k_vel=v/10.0)
    else: continue
    rows.append(row)
keys=['rec','ts','T','kind','fusion','phase','armed','p1','p2','p3','baro_m','vv','gnd','ax','ay','az','gx','gy','gz','pk_alt','pk_acc','pk_vv','ms_launch','pflags','pyflags','pych','pydur','hv','k_alt','k_vel']
with open('flight.csv','w',newline='') as f:
    wcsv=csv.DictWriter(f,fieldnames=keys); wcsv.writeheader()
    for r in rows: wcsv.writerow(r)
print("rows",len(rows))
import collections; print(collections.Counter(r['kind'] for r in rows))
