import math
# descent sanity: ballistic fall from apogee with terminal velocity vt
for vt in (95,100,101,105,110):
    g=9.80665; t=26.36
    d=(vt*vt/g)*math.log(math.cosh(g*t/vt))
    print(f" vt={vt:5.1f} m/s -> falls {d:6.0f} m in {t}s   (measured 1933 m)")
print()
# pad accel bias
print("pad: ax=1036 ay=-4 az=-92 mg  -> axial scale err +3.6%, az offset -92mg, ay offset -4mg")
print()
# GPS geometry
import math
def dist(a,b):
    dlat=(b[0]-a[0])*111320.0; dlon=(b[1]-a[1])*111320.0*math.cos(math.radians(a[0]))
    return math.hypot(dlat,dlon), dlat, dlon
pad=(-31.1577173,149.9282093); land=(-31.1631022,149.9317184)
last=(-31.1625933,149.9315946)
d,dla,dlo=dist(pad,land); print(f"pad -> true landing : {d:6.1f} m   (N{dla:+.0f} m, E{dlo:+.0f} m)  bearing {(math.degrees(math.atan2(dlo,dla))%360):.0f}deg")
d,dla,dlo=dist(last,land); print(f"last GPS fix -> landing: {d:6.1f} m  (N{dla:+.0f} m, E{dlo:+.0f} m)")
# descent horizontal speed from GPS
pts=[(33.69,-31.1617610,149.9305970),(36.73,-31.1621838,149.9311156),(37.73,-31.1622763,149.9312750),
     (38.73,-31.1623916,149.9313833),(39.73,-31.1625078,149.9314565),(40.73,-31.1625933,149.9315946)]
print("\ndescent horizontal speed from GPS:")
for (t0,la0,lo0),(t1,la1,lo1) in zip(pts,pts[1:]):
    dd,_,_=dist((la0,lo0),(la1,lo1)); print(f"  T {t0:.2f}->{t1:.2f}  {dd:5.1f} m  = {dd/(t1-t0):5.1f} m/s horizontal")
print(f"\nmean horizontal over whole flight = {686/43.5:.1f} m/s  (686 m in 43.5 s)")
