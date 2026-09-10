import math
exec(open('tradeoff.py').read().split('print("Subsonic')[0])
def run2(CdA,k,cdm,cut,dt=0.001):
    def T(t): return 0.0 if t>=cut else th(t,k)
    def I(t): return imp(min(t,cut),k)
    Ifull=I(1e9)
    t=v=h=0.0; vmax=0; tv=0
    while t<40:
        F=T(t); m=M0-MP*(I(t)/Ifull if Ifull else 0)
        r,a_s=atm(max(h,0)); M=abs(v)/a_s
        D=0.5*r*v*abs(v)*CdA*cdm(M)
        v+=((F-D)/m-G)*dt; h+=v*dt; t+=dt
        if v>vmax: vmax=v; tv=t
        if t>0.05 and v<=0: break
    return h,t,vmax,Ifull
cdm=mk_cdm(2.5)
print("Burn shape test: all scaled to reach the measured 2019 m apogee, CdA fixed at measured 4.5e-4")
print(" burn cut   impulse    burn dur   Vmax   Mach   t_apogee   vs measured 17.15 s")
for cut,label in ((2.072,"full"),(1.5,"cut 1.5s"),(1.0,"cut 1.0s"),(0.85,"cut 0.85s"),(0.6,"cut 0.6s")):
    lo,hi=0.5,6.0
    for _ in range(45):
        mid=(lo+hi)/2
        if run2(4.5e-4,mid,cdm,cut)[0]<2019: lo=mid
        else: hi=mid
    k=(lo+hi)/2; h,ta,vm,Ifull=run2(4.5e-4,k,cdm,cut)
    _,a_s=atm(400)
    print(f"  {label:9s} {Ifull:7.0f} Ns   {min(cut,2.072):5.2f} s   {vm:5.0f}  {vm/a_s:5.2f}   {ta:6.2f}s   {ta-17.15:+5.2f}")
print()
# last altitude corrected for the static-port error
Cp=0.125; v=101.0
err=Cp*v*v/(2*G)
print(f"Last logged raw baro: 86 m AGL indicated, descending {v:.0f} m/s")
print(f"Static-port error at that speed (Cp={Cp}): {err:.0f} m")
print(f"  -> true altitude at last record ~ {86-err:.0f} m AGL")
print(f"  -> time to impact ~ {(86-err)/v:.2f} s")
for cp in (0.10,0.125,0.15):
    e=cp*v*v/(2*G); print(f"     Cp={cp:.3f}: true {86-e:5.1f} m -> {(86-e)/v:.2f} s to impact")
