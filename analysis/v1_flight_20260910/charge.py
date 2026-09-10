import csv, math
rows=list(csv.DictReader(open('flight.csv')))
def f(r,k):
    v=r.get(k); return float(v) if v not in (None,'') else None
baro=[(f(r,'T')+0.65,f(r,'baro_m'),f(r,'vv')) for r in rows if r['kind']=='baro']
acc=[(f(r,'T')+0.65,f(r,'ax'),f(r,'ay'),f(r,'az')) for r in rows if r['kind']=='acc']
print("Looking for an ejection transient. Motor delay 14 s from burnout -> expect ~T+16.1 s")
print("  T     baro(MSL)  d(baro) resid   ax     ay     az    |a|")
prev=None
for T,b,vv in baro:
    if not (13.0<=T<=21.0): continue
    a=min(acc,key=lambda x:abs(x[0]-T))
    m=math.sqrt(a[1]**2+a[2]**2+a[3]**2)/1000
    d = (b-prev) if prev is not None else 0.0
    prev=b
    print(f" {T:6.2f} {b:9.2f} {d:+7.2f}        {a[1]:6.0f} {a[2]:6.0f} {a[3]:6.0f} {m:6.3f}")
# statistics of baro step size over quiet coast, to see what a spike would look like
import statistics
q=[b for b in baro if 8<=b[0]<=15]
st=[q[i+1][1]-q[i][1] for i in range(len(q)-1)]
print(f"\nbaro sample-to-sample step over T+8..15: mean {statistics.mean(st):+.2f} m, stdev {statistics.pstdev(st):.2f} m")
q2=[b for b in baro if 18<=b[0]<=24]
st2=[q2[i+1][1]-q2[i][1] for i in range(len(q2)-1)]
print(f"                          over T+18..24: mean {statistics.mean(st2):+.2f} m, stdev {statistics.pstdev(st2):.2f} m")
# accel noise floor in quiet coast
qa=[a for a in acc if 10<=a[0]<=16]
mags=[math.sqrt(a[1]**2+a[2]**2+a[3]**2)/1000 for a in qa]
print(f"accel |a| over T+10..16: mean {statistics.mean(mags):.3f} g, max {max(mags):.3f} g, stdev {statistics.pstdev(mags):.3f} g")
