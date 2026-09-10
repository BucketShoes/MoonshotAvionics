import csv, math
rows=list(csv.DictReader(open('flight.csv')))
def f(r,k):
    v=r.get(k); return float(v) if v not in (None,'') else None
acc=[(f(r,'T'),f(r,'ax'),f(r,'ay'),f(r,'az')) for r in rows if r['kind']=='acc']
G=9.80665; SCALE=1.036; AZB=-92.0; AYB=-4.0
GND=263.0; APO=2282.2; TAPO=16.5; TIGN=-0.65
def axial(a): return (a[1]/SCALE)/1000.0*G          # signed, +X = nose
def vecmag(a):
    return math.sqrt((a[1]/SCALE)**2+(a[2]-AYB)**2+(a[3]-AZB)**2)/1000.0*G
def integrate(getter, tstart):
    seg=[a for a in acc if tstart<=a[0]<=TAPO]
    v=0.0; path=0.0; prof={}
    for i in range(len(seg)-1,0,-1):
        t1,t0=seg[i][0],seg[i-1][0]; dt=t1-t0
        if dt<=0 or dt>0.5: continue
        d=0.5*(getter(seg[i])+getter(seg[i-1]))
        vm=v+0.5*(d+G)*dt; path+=vm*dt; v+=(d+G)*dt; prof[t0]=v
    return v,path,prof
print("Backward integration to apogee, VERTICAL flight (GPS says tilt ~2deg)\n")
for name,getter in (("vector |a| (old)",lambda a: math.sqrt(a[1]**2+a[2]**2+a[3]**2)/1000.0*G),
                    ("vector, bias-corrected",vecmag),
                    ("AXIAL only (-ax), scale-corr",lambda a: -axial(a))):
    for tb,burnlabel in ((0.15,"burnout T=+0.15 (0.8s burn)"),(1.42,"burnout T=+1.42 (2.07s full burn)")):
        v0,path,prof=integrate(getter,tb)
        # burnout height from a forward boost estimate: ~0.5*v0*burnduration
        dur=tb-TIGN
        hb=0.5*v0*dur
        need=APO-GND-hb
        print(f" {name:30s} {burnlabel:32s} v_burnout={v0:6.1f}  path={path:6.0f}  h_burnout={hb:5.0f}  need={need:6.0f}  err={100*(path-need)/need:+6.1f}%")
    print()
