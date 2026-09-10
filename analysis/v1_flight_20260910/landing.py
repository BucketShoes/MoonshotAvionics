import math
def d(a,b):
    dla=(b[0]-a[0])*111320.0; dlo=(b[1]-a[1])*111320.0*math.cos(math.radians(a[0]))
    return math.hypot(dla,dlo), dla, dlo
pad=(-31.1577173,149.9282093); land=(-31.1631022,149.9317184)
p34=(-31.1617610,149.9305970); p41=(-31.1625933,149.9315946)
TAPO=17.15; TLAND=43.7
tot,dla,dlo=d(pad,land)
print(f"pad -> landing        : {tot:6.1f} m   bearing {(math.degrees(math.atan2(dlo,dla))%360):3.0f}deg")
a,_,_=d(p34,land);  print(f"T+33.69 fix -> landing: {a:6.1f} m  ({TLAND-33.69:.1f} s of flight left)")
b,_,_=d(p41,land);  print(f"T+40.74 fix -> landing: {b:6.1f} m  ({TLAND-40.74:.1f} s of flight left)")
# horizontal speed on descent, and back-extrapolation to apogee along the same track
vh,_,_=d(p34,p41); vh/= (40.74-33.69)
print(f"\ndescent horizontal speed {vh:.1f} m/s  (wind alone would be ~5.6 m/s at 20 km/h)")
back=vh*(33.69-TAPO)
print(f"back-extrapolating {vh:.1f} m/s from T+33.69 to apogee -> apogee was ~{back:.0f} m short of that fix")
# fraction of drift before/after apogee
f_asc=vh*TAPO; f_des=vh*(TLAND-TAPO)
print(f"\nif horizontal speed were constant all flight ({vh:.1f} m/s):")
print(f"   drift during ascent  {f_asc:5.0f} m   ({100*f_asc/tot:.0f}% of the {tot:.0f} m total)")
print(f"   drift during descent {f_des:5.0f} m   ({100*f_des/tot:.0f}%)")
print("\nWhat each predictor would have given (true landing = 0 m error):")
print(f"  'lands at the pad'                        error {tot:6.0f} m")
print(f"  'lands under apogee' (apogee ~{f_asc:.0f} m out)  error {tot-f_asc:6.0f} m")
print(f"  apogee position + {f_des:.0f} m descent drift   error   ~0 m  (if wind known)")
print(f"  last GPS fix, no extrapolation             error {b:6.0f} m")
print(f"  last GPS fix + {vh:.0f} m/s for {TLAND-40.74:.1f} s      error   ~{abs(b-vh*(TLAND-40.74)):4.0f} m")
