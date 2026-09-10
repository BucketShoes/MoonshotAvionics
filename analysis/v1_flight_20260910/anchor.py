import csv, math
rows=list(csv.DictReader(open('flight.csv')))
def f(r,k):
    v=r.get(k); return float(v) if v not in (None,'') else None
acc=[(f(r,'T'),f(r,'ax'),f(r,'ay'),f(r,'az')) for r in rows if r['kind']=='acc']
baro={round(f(r,'T'),3):f(r,'baro_m') for r in rows if r['kind']=='baro'}
bl=sorted(baro.items())
def baroAt(t): return min(bl,key=lambda x:abs(x[0]-t))[1]
G=9.80665; GND=263.0; APO=2282.2; TAPO=16.5; SCALE=1.036
seg=[a for a in acc if a[0]<=TAPO]
def back(tstart):
    v=0.0; rise=0.0
    s=[a for a in seg if a[0]>=tstart]
    for i in range(len(s)-1,0,-1):
        t1,t0=s[i][0],s[i-1][0]; dt=t1-t0
        if dt<=0 or dt>0.5: continue
        d=0.5*(-s[i][1]-s[i-1][1])/SCALE/1000.0*G
        vm=v+0.5*(d+G)*dt; rise+=vm*dt; v+=(d+G)*dt
    return v,rise
print("Anchored ONLY on measured apogee + measured axial decel (no drag model, no motor data)")
print(" from logT   v there (m/s)   rise to apogee (m)   -> altitude there (m AGL)   baro reads (AGL)")
for ts in (2.4,2.5,3.0,3.5,4.0,5.0,6.0):
    v,rise=back(ts)
    print(f"   {ts:5.2f}      {v:8.1f}          {rise:8.0f}              {APO-GND-rise:8.0f}            {baroAt(ts)-GND:8.0f}")
print()
# what does the motor have to do to reach that state?
ITOT=229.0; M0=0.432; MP=0.082
curve=[(0,0),(0.014,159.806),(0.045,110.931),(0.104,144.43),(0.131,135.643),(0.252,154.315),
(0.293,130.701),(0.387,149.921),(1.005,140.037),(1.261,119.168),(1.297,142.233),(1.324,109.833),
(1.387,120.267),(1.518,110.382),(1.595,80.727),(1.685,60.408),(1.797,33.499),(1.896,14.278),(2.072,0.0)]
def impulse(t):
    s=0
    for (t0,f0),(t1,f1) in zip(curve,curve[1:]):
        if t<=t0: break
        b=min(t,t1); fb=f0+(f1-f0)*(b-t0)/(t1-t0); s+=0.5*(f0+fb)*(b-t0)
        if t<t1: break
    return s
TIGN=-0.65
v24,rise24=back(2.4)
print(f"At logT=2.40 (t={2.4-TIGN:.2f}s after ignition): v={v24:.0f} m/s, altitude={APO-GND-rise24:.0f} m AGL")
print(f"Motor delivers {impulse(2.072):.0f} Ns total.")
mavg=(M0+ (M0-MP))/2
print(f"Ideal dv (all impulse, mean mass {mavg*1000:.0f} g) = {ITOT/mavg:.0f} m/s")
print(f"minus gravity loss over {2.4-TIGN:.2f}s = {G*(2.4-TIGN):.0f} m/s -> {ITOT/mavg-G*(2.4-TIGN):.0f} m/s available before drag")
print(f"So drag loss to reach {v24:.0f} m/s = {ITOT/mavg-G*(2.4-TIGN)-v24:.0f} m/s  ({100*(ITOT/mavg-G*(2.4-TIGN)-v24)/(ITOT/mavg):.0f}% of ideal dv)")
