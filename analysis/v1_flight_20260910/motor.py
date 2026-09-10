import math
curve=[(0,0),(0.014,159.806),(0.045,110.931),(0.104,144.43),(0.131,135.643),(0.252,154.315),
(0.293,130.701),(0.387,149.921),(1.005,140.037),(1.261,119.168),(1.297,142.233),(1.324,109.833),
(1.387,120.267),(1.518,110.382),(1.595,80.727),(1.685,60.408),(1.797,33.499),(1.896,14.278),(2.072,0.0)]
I=0
for (t0,f0),(t1,f1) in zip(curve,curve[1:]): I+=0.5*(f0+f1)*(t1-t0)
print(f"total impulse = {I:.1f} Ns   burn time = {curve[-1][0]:.3f} s   avg thrust = {I/curve[-1][0]:.1f} N")
print(f"propellant 0.082 kg -> Isp = {I/(0.082*9.80665):.0f} s")
# impulse delivered by time t
def imp(t):
    s=0
    for (t0,f0),(t1,f1) in zip(curve,curve[1:]):
        if t<=t0: break
        a,b=t0,min(t,t1)
        fa=f0; fb=f0+(f1-f0)*(b-t0)/(t1-t0)
        s+=0.5*(fa+fb)*(b-a)
        if t<t1: break
    return s
for t in [0.4,0.6,0.8,0.85,1.0,1.2,1.42,1.6,2.072]:
    print(f"  I({t:5.3f}) = {imp(t):6.1f} Ns  ({100*imp(t)/I:4.1f}%)  ideal dv on 0.432->m = {imp(t)/ (0.432-0.082*imp(t)/I/2):6.0f} m/s")
