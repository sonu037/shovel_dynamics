"""Perceptual separation check (CIE Lab, dE76) for the figure palette."""
from matplotlib.colors import to_rgb
import itertools, numpy as np
def lab(c):
    r,g,b=to_rgb(c)
    f=lambda u: u/12.92 if u<=0.04045 else ((u+0.055)/1.055)**2.4
    r,g,b=f(r),f(g),f(b)
    X=r*0.4124+g*0.3576+b*0.1805; Y=r*0.2126+g*0.7152+b*0.0722; Z=r*0.0193+g*0.1192+b*0.9505
    Xn,Yn,Zn=0.95047,1.0,1.08883
    g_=lambda t: t**(1/3) if t>0.008856 else 7.787*t+16/116
    fx,fy,fz=g_(X/Xn),g_(Y/Yn),g_(Z/Zn)
    return np.array([116*fy-16, 500*(fx-fy), 200*(fy-fz)])
def dE(a,b): return float(np.linalg.norm(lab(a)-lab(b)))
def report(roles, thresh=24.0):
    bad=[(a,b,round(dE(roles[a],roles[b]),1)) for a,b in itertools.combinations(roles,2)
         if dE(roles[a],roles[b])<thresh]
    return bad
if __name__=="__main__":
    roles={"handle":"#C8A15A","dipper-edge":"#1A1A1A","boom":"#5B739B","kinematics":"#B03A2E",
           "ropes":"#1F3864","frame-o2":"#0F7B6C","frame-o3":"#7B3FA0","frame-o4":"#C2185B",
           "frame-W":"#555555","construction":"#8A8A8A"}
    bad=report(roles)
    print("pairs closer than dE 24:", bad if bad else "none")
    print("min pairwise dE:", round(min(dE(roles[a],roles[b]) for a,b in itertools.combinations(roles,2)),1))
