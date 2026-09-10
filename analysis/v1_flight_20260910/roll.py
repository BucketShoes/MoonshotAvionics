import csv, math
rows=list(csv.DictReader(open('flight.csv')))
def f(r,k):
    v=r.get(k); return float(v) if v not in (None,'') else None
acc=[(f(r,'T'),f(r,'ax'),f(r,'ay'),f(r,'az')) for r in rows if r['kind']=='acc']
gyr=[(f(r,'T'),f(r,'gx')) for r in rows if r['kind']=='gyr']
G=9.80665; AYB=-4.0
print("Using transverse accel as a tachometer:  a_y = omega^2 * r  (IMU offset r from the roll axis)")
print("Calibrate r at two points where the gyro is NOT railed:\n")
cal=[]
for t,g in gyr:
    if abs(g)>=2270 or abs(g)<300: continue
    a=min(acc,key=lambda x:abs(x[0]-t))
    ay=abs(a[2]-AYB)/1000.0*G
    w=abs(g)*math.pi/180.0
    if w>1: cal.append((t,g,ay,ay/w**2*1000))
for t,g,ay,r in cal[:14]:
    print(f"  T={t:6.2f} gyro={g:8.1f} deg/s  |ay|={ay:6.2f} m/s^2  -> r={r:5.2f} mm")
rs=sorted(x[3] for x in cal); med=rs[len(rs)//2]
print(f"\n  median r = {med:.2f} mm")
r=med/1000.0
print("\nRoll rate reconstructed from a_y (valid while a_y is dominated by centripetal):")
print("   logT   t_ign   |ay|(g)   omega(deg/s)   rev/s   gyro logged")
for t in [2.4,2.6,3.0,3.5,4.0,5.0,6.0,8.0,10.0,12.0,14.0,16.0,20,25,30,35,40,42.5]:
    a=min(acc,key=lambda x:abs(x[0]-t))
    ay=abs(a[2]-AYB)/1000.0*G
    w=math.sqrt(ay/r) if ay>0 else 0
    gl=min(gyr,key=lambda x:abs(x[0]-t))
    print(f" {t:6.1f} {t+0.65:7.2f} {abs(a[2])/1000:9.3f} {math.degrees(w):12.0f} {math.degrees(w)/360:7.1f}   {gl[1]:9.1f}")
