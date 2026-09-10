import csv, math
rows=list(csv.DictReader(open('flight.csv')))
def f(r,k):
    v=r.get(k); return float(v) if v not in (None,'') else None
baro=sorted([(f(r,'T')+0.65,f(r,'baro_m')) for r in rows if r['kind']=='baro'])
acc=[(f(r,'T')+0.65,f(r,'ax'),f(r,'ay'),f(r,'az')) for r in rows if r['kind']=='acc']
G=9.80665; M=0.350; SITE=303.0; SCALE=1.036
def rho(hm):
    T=293.15-0.0065*hm; return 101325*(T/293.15)**5.2559/(287.05*T)
def vel(t,w=0.9):
    s=[b for b in baro if abs(b[0]-t)<=w]
    n=len(s); sx=sum(b[0] for b in s); sy=sum(b[1] for b in s)
    sxx=sum(b[0]**2 for b in s); sxy=sum(b[0]*b[1] for b in s)
    return (n*sxy-sx*sy)/(n*sxx-sx*sx)
def bAt(t): return min(baro,key=lambda x:abs(x[0]-t))[1]
print("Descent drag area over time. If coning damps out, CdA should FALL as the descent settles.")
print("If it is a fixed configuration change (shifted cone, bent fin), CdA should be FLAT from the start.\n")
print("   T     alt(MSL)   v(m/s)   axial(g)   drag(N)    CdA(m^2)   Cd(34mm)")
A34=math.pi*0.017**2
for t in [20,22,24,26,28,30,32,34,36,38,40,42]:
    v=abs(vel(t)); a=min(acc,key=lambda x:abs(x[0]-t))
    ax=abs(a[1])/SCALE/1000.0     # axial only, scale corrected
    hm=bAt(t)+40-0.125*v*v/(2*G)  # QNH + airspeed correction -> true MSL
    # accelerometer measures drag/m directly (freefall); during descent it also
    # includes the fact we're not yet at terminal
    drag=ax*G*M
    cda=drag/(0.5*rho(hm)*v*v)
    print(f" {t:5.1f} {hm:9.0f} {v:8.1f} {ax:9.3f} {drag:9.2f} {cda:11.2e} {cda/A34:9.2f}")
print("\nAscent coast, same treatment (velocity from back-integration, not baro):")
# reuse anchored velocities
import json
asc={}
for T,v in [(4.65,204.5),(5.65,168.8),(6.65,141.7),(8.65,101.0),(10.65,72.0)]:
    a=min(acc,key=lambda x:abs(x[0]-T)); ax=abs(a[1])/SCALE/1000.0
    hm=bAt(T)+40-0.125*v*v/(2*G)
    drag=ax*G*M; cda=drag/(0.5*rho(hm)*v*v)
    print(f" {T:5.1f} {hm:9.0f} {v:8.1f} {ax:9.3f} {drag:9.2f} {cda:11.2e} {cda/A34:9.2f}")
