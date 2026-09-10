import math,csv,json
curve=[(0,0),(0.014,159.806),(0.045,110.931),(0.104,144.43),(0.131,135.643),(0.252,154.315),
(0.293,130.701),(0.387,149.921),(1.005,140.037),(1.261,119.168),(1.297,142.233),(1.324,109.833),
(1.387,120.267),(1.518,110.382),(1.595,80.727),(1.685,60.408),(1.797,33.499),(1.896,14.278),(2.072,0.0)]
ITOT=229.0; M0=0.432; MP=0.082; G=9.80665; SITE=303.0; TIGN=-0.65
def thrust(t):
    if t>curve[-1][0]: return 0.0
    for (t0,f0),(t1,f1) in zip(curve,curve[1:]):
        if t0<=t<=t1: return f0+(f1-f0)*(t-t0)/(t1-t0)
    return 0.0
def impulse(t):
    s=0
    for (t0,f0),(t1,f1) in zip(curve,curve[1:]):
        if t<=t0: break
        b=min(t,t1); fb=f0+(f1-f0)*(b-t0)/(t1-t0); s+=0.5*(f0+fb)*(b-t0)
        if t<t1: break
    return s
def atm(h):
    hm=h+SITE; T=293.15-0.0065*hm
    return 101325*(T/293.15)**5.2559/(287.05*T), 20.05*math.sqrt(T)
def cdm(M):
    if M<0.8: return 1.0
    if M<1.05: return 1.0+1.5*(M-0.8)/0.25
    if M<1.4:  return 2.5-0.7*(M-1.05)/0.35
    return 1.8
def run(CdA,dt=0.001):
    t=v=h=0.0; tr=[]; vmax=0; tvmax=0; amax=0
    while t<40:
        F=thrust(t); m=M0-MP*impulse(t)/ITOT
        r,a_s=atm(max(h,0)); M=abs(v)/a_s
        D=0.5*r*v*abs(v)*CdA*cdm(M); acc=(F-D)/m
        tr.append((t,v,h,M,acc/G))
        if acc/G>amax: amax=acc/G
        v+=(acc-G)*dt; h+=v*dt; t+=dt
        if v>vmax: vmax=v; tvmax=t
        if t>0.05 and v<=0: break
    return h,t,vmax,tvmax,amax,tr
lo,hi=2e-4,1e-3
for _ in range(45):
    mid=(lo+hi)/2
    if run(mid)[0]>2019: lo=mid
    else: hi=mid
CdA=(lo+hi)/2
h,tapo,vmax,tvmax,amax,tr=run(CdA)
A38=math.pi*0.019**2
print(f"RECONSTRUCTION  CdA={CdA:.3e} m^2 (Cd={CdA/A38:.2f} on 38mm)  [coast accel independently gives 4.6e-4]")
print(f"  apogee {h:.0f} m AGL @ t={tapo:.2f}s after ignition   (measured 2019 m @ 17.15 s)")
print(f"  MAX SPEED {vmax:.0f} m/s at t={tvmax:.2f}s (logT {tvmax+TIGN:+.2f})")
_,asnd=atm(380); print(f"  Mach {vmax/asnd:.2f}   burnout t=2.072s v={min(tr,key=lambda x:abs(x[0]-2.072))[1]:.0f} m/s")
print(f"  PEAK ACCELERATION {amax:.1f} g  (logged value railed at 16.4 g)")
sup=[p for p in tr if p[3]>=1.0]
print(f"  supersonic from t={sup[0][0]:.2f} to t={sup[-1][0]:.2f} s  ({sup[-1][0]-sup[0][0]:.2f} s above Mach 1)")
# baro error vs Mach using THIS trajectory
rows=list(csv.DictReader(open('flight.csv')))
def f(r,k):
    v=r.get(k); return float(v) if v not in (None,'') else None
baro=sorted([(f(r,'T'),f(r,'baro_m')-263.0) for r in rows if r['kind']=='baro'])
def bAt(t): return min(baro,key=lambda x:abs(x[0]-t))[1]
print("\nBARO ERROR vs MACH (static-port pressure coefficient Cp = 2*g*err/v^2):")
print("  logT  t_ign   v     Mach   true_alt   baro_reads   error    Cp")
for lt in [-0.3,-0.1,0.1,0.3,0.5,0.7,0.9,1.1,1.3,1.6,2.0,2.3,3.0,4.0,6.0,8.0]:
    t=lt-TIGN; p=min(tr,key=lambda x:abs(x[0]-t))
    if p[1]<20: continue
    err=bAt(lt)-p[2]; cp=2*G*err/p[1]**2
    print(f" {lt:5.1f} {t:6.2f} {p[1]:6.0f} {p[3]:6.2f} {p[2]:9.0f} {bAt(lt):11.0f} {err:8.0f} {cp:7.3f}")
json.dump({'CdA':CdA,'traj':[[round(x,3) for x in p] for p in tr[::10]]},open('recon.json','w'))
