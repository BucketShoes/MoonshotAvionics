import struct, sys, collections
import os,sys
BIN = os.environ.get("MOONSHOT_BIN") or (sys.argv[1] if len(sys.argv)>1 else "moonshot-fetch.bin")
d=open(BIN,'rb').read()
recs=[]
p=0
bad=0
while p+10<=len(d):
    rec,ln,snr,ts=struct.unpack_from('<IbxxxxB',d,p) if False else (None,)*4
    rec=struct.unpack_from('<I',d,p)[0]
    ln=d[p+4]
    snr=struct.unpack_from('<b',d,p+5)[0]
    ts=struct.unpack_from('<I',d,p+6)[0]
    if p+10+ln>len(d): break
    pay=d[p+10:p+10+ln]
    recs.append((rec,ln,snr,ts,pay))
    p+=10+ln
print("records:",len(recs),"bytes consumed",p,"of",len(d))
print("rec range",recs[0][0],recs[-1][0])
# check monotonic
gaps=[(a[0],b[0]) for a,b in zip(recs,recs[1:]) if b[0]!=a[0]+1]
print("non-consecutive count:",len(gaps), gaps[:10])
c=collections.Counter(r[4][0] for r in recs if r[1]>0)
print("payload type histogram:",sorted(c.items()))
