# MS02P08B pre-run prediction (2026-11-18, Revision 2).
# Evaluated on 65 s run, first 5 s excluded (60.0 s steady-state window, 1 full minute).
# Run: python ms02p08b_predict.py  -> E0 +21.9 kN, E1 +21.3 kN (see MS02P08B_SPEC.md section 5).
# Pre-run prediction for MS02P08B (ideal tracking of the reference trajectory assumed).

import numpy as np

g = 9.80665
eps = 1e-3
Ptrue = dict(Md=54300.0, c=-1.32, Izz=287900.0, fv3=3800.0, fc3=340.0, fv4=1473.0, fc4=480.0)

def tau_model(P, q, qd, qdd, d, dd, ddd):
    r = d + P['c']
    t3 = (P['Izz'] + P['Md']*r**2)*qdd + 2*P['Md']*r*dd*qd + P['Md']*g*r*np.cos(q) + P['fv3']*qd + P['fc3']*np.tanh(qd/eps)
    f4 = P['Md']*ddd - P['Md']*r*qd**2 + P['Md']*g*np.sin(q) + P['fv4']*dd + P['fc4']*np.tanh(dd/eps)
    return t3, f4

def case(name, q3B, q3A, d4B, d4A, w, F0=100e3, wF=2*np.pi/9, T=65.0, trim=5.0):
    t = np.arange(trim, T, 1/300.0)
    q = np.deg2rad(q3B + q3A*np.sin(w*t))
    qd = np.deg2rad(q3A)*w*np.cos(w*t)
    qdd = -np.deg2rad(q3A)*w*w*np.sin(w*t)
    d = d4B + d4A*np.sin(w*t)
    dd = d4A*w*np.cos(w*t)
    ddd = -d4A*w*w*np.sin(w*t)
    Fy = F0*np.sin(wF*t)
    
    Ph = dict(Ptrue)
    Ph['Md'] *= 1.05
    Ph['Izz'] *= 1.10
    Ph['c'] += 0.05
    
    t3t, f4t = tau_model(Ptrue, q, qd, qdd, d, dd, ddd)
    t3h, f4h = tau_model(Ph, q, qd, qdd, d, dd, ddd)
    
    dQ3 = t3h - t3t
    s = d + 1.5
    eFy = dQ3 / (s*np.cos(q))
    
    print(f"{name:10s} q3 {q3B:+.0f}+/-{q3A:.0f} deg, d4 {d4B}+/-{d4A} m, w={w:.3f}: "
          f"peak|Q3_true|={np.max(np.abs(s*Fy*np.cos(q)))/1e3:.0f} kNm; dQ3 mean={dQ3.mean()/1e3:.1f} kNm, rms={np.sqrt((dQ3**2).mean())/1e3:.1f} kNm; "
          f"Fy error: bias={eFy.mean()/1e3:.2f} kN, rms={np.sqrt((eFy**2).mean())/1e3:.2f} kN, peak={np.max(np.abs(eFy))/1e3:.1f} kN "
          f"(NRMSE vs 100 kN peak = {100*np.sqrt((eFy**2).mean())/F0:.1f} %)")

if __name__ == '__main__':
    print("MS02P08B Pre-Run Analytic Predictions (Revision 2: 65 s run, 5 s trim -> 60 s window):")
    case('E0', 45, 15, 8.75, 2.0, np.pi/10, T=65.0, trim=5.0)
    case('E1', -12, 9, 7.325, 0.275, 0.51, T=65.0, trim=5.0)
