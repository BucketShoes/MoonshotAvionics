import math
curve=[(0,0),(0.014,159.806),(0.045,110.931),(0.104,144.43),(0.131,135.643),(0.252,154.315),
(0.293,130.701),(0.387,149.921),(1.005,140.037),(1.261,119.168),(1.297,142.233),(1.324,109.833),
(1.387,120.267),(1.518,110.382),(1.595,80.727),(1.685,60.408),(1.797,33.499),(1.896,14.278),(2.072,0.0)]
ITOT=229.0; M0=0.432; MP=0.082; G=9.80665; SITE=303.0
A34=math.pi*0.017**2
def th(t,k):
    if t>curve[-1][0]: return 0.0
    for (t0,f0),(t1,f1) in zip(curve,curve[1:]):
        if t0<=t<=t1: return k*(f0+(f1-f0)*(t-t0)/(t1-t0))
    return 0.0
def imp(t,k):
    s=0
    for (t0,f0),(t1,f1) in zip(curve,curve[1:]):
        if t<=t0: break
        b=min(t,t1); fb=f0+(f1-f0)*(b-t0)/(t1-t0); s+=0.5*(f0+fb)*(b-t0)
        if t<t1: break
    return k*s
def atm(h):
    hm=h+SITE; T=293.15-0.0065*hm
    return 101325*(T/293.15)**5.2559/(287.05*T), 20.05*math.sqrt(T)
def mk_cdm(peak):
    def c(M):
        if M<0.8: return 1.0
        if M<1.05: return 1.0+(peak-1.0)*(M-0.8)/0.25
        if M<1.4:  return peak-(peak-1.0)*0.4*(M-1.05)/0.35
        return peak-(peak-1.0)*0.4
    return c
def run(CdA,k,cdm,dt=0.001):
    t=v=h=0.0; vmax=0; tv=0
    while t<40:
        F=th(t,k); m=M0-MP*imp(t,k)/(ITOT*k) if k else M0
        r,a_s=atm(max(h,0)); M=abs(v)/a_s
        D=0.5*r*v*abs(v)*CdA*cdm(M)
        v+=((F-D)/m-G)*dt; h+=v*dt; t+=dt
        if v>vmax: vmax=v; tv=t
        if t>0.05 and v<=0: break
    return h,t,vmax,tv
print("Subsonic drag area is MEASURED both ways (mass-independent ratio):")
print(f"  ascent coast, Mach 0.4 : CdA 4.5e-4  -> Cd {4.5e-4/A34:.2f} on 34 mm")
print(f"  descent, terminal      : CdA 6.4e-4  -> Cd {6.4e-4/A34:.2f} on 34 mm")
print(f"  ratio {6.4/4.5:.2f} : descent draggier\n")
print("Holding the MEASURED subsonic ascent CdA=4.5e-4, what transonic rise + impulse reaches 2019 m?")
print("  Cd peak  impulse needed   as % of 229 Ns   Vmax    Mach   t_apogee")
for peak in (1.5,2.0,2.5,3.0,3.5):
    cdm=mk_cdm(peak)
    lo,hi=0.5,2.5
    for _ in range(40):
        mid=(lo+hi)/2
        if run(4.5e-4,mid,cdm)[0]<2019: lo=mid
        else: hi=mid
    k=(lo+hi)/2; h,ta,vm,tv=run(4.5e-4,k,cdm)
    _,a_s=atm(400)
    print(f"   {peak:4.1f}x   {ITOT*k:8.0f} Ns      {100*k:6.0f}%      {vm:5.0f}  {vm/a_s:5.2f}   {ta:5.2f}s")
print("\nSame, but if ascent drag were the DESCENT value 6.4e-4 (i.e. no asymmetry):")
print("  Cd peak  impulse needed   as % of 229 Ns   Vmax    Mach   t_apogee")
for peak in (1.5,2.0,2.5,3.0):
    cdm=mk_cdm(peak)
    lo,hi=0.5,3.0
    for _ in range(40):
        mid=(lo+hi)/2
        if run(6.4e-4,mid,cdm)[0]<2019: lo=mid
        else: hi=mid
    k=(lo+hi)/2; h,ta,vm,tv=run(6.4e-4,k,cdm)
    _,a_s=atm(400)
    print(f"   {peak:4.1f}x   {ITOT*k:8.0f} Ns      {100*k:6.0f}%      {vm:5.0f}  {vm/a_s:5.2f}   {ta:5.2f}s")
print("\nMEASURED apogee time from first motion: 17.15 s")
