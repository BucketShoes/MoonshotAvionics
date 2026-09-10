import csv
rows=list(csv.DictReader(open('flight.csv')))
def f(r,k):
    v=r.get(k); return float(v) if v not in (None,'') else None
baro=sorted([(f(r,'T')+0.65,f(r,'baro_m')) for r in rows if r['kind']=='baro'])
def bAt(t): return min(baro,key=lambda x:abs(x[0]-t))[1]
# GPS alt records (first-motion frame), with DOP
g=[(33.69,1474.3,0.9,0.7,21),(36.74,1.0,25.5,0.0,27),(37.74,967.3,1.6,0.9,26),
   (38.74,847.8,1.2,0.7,25),(39.74,748.0,1.1,0.8,25),(40.74,628.4,1.6,1.0,25)]
G=9.80665; Cp=0.125
print("GPS altitude vs barometer on descent.  Baro reads HIGH by Cp*v^2/2g (~65 m at 101 m/s).")
print("Site is 303 m MSL on the map; baro's ISA pressure altitude reads it as 263 m (QNH offset ~ +40 m).")
print("  T      GPS alt   baro raw   baro corrected(+40 QNH, -65 airspeed)   GPS-corrected   VDOP HDOP sats")
for t,alt,vdop,hdop,sats in g:
    b=bAt(t); bc=b+40-65
    print(f" {t:6.2f} {alt:9.1f} {b:10.1f} {bc:35.1f} {alt-bc:15.1f}   {vdop:4.1f} {hdop:4.1f} {sats:4d}")
print("\npad reference: GPS 330.5 m MSL, baro 263.9 m MSL, map 303 m")
print("  GPS - map = +27.5 m   (consistent offset; likely ellipsoid/geoid or receiver bias)")
