# MS02P08B controller/tolerance sanity check on the ANALYTIC plant (2026-11-18, Revision 2).
# Evaluated on 65 s run, first 5 s excluded (60.0 s steady-state window, 1 full minute).
# Not a validation. Expected: tracking < 0.02 deg; mismatch bias 21.90 / 21.32 kN.
# Sanity check of the controller design and the pre-registered tolerances (analytic plant, NOT a validation).

import numpy as np
from scipy.integrate import solve_ivp

g = 9.80665
T = dict(Md=54300.0, c=-1.32, Izz=287900.0, fv3=3800.0, fc3=340.0, fv4=1473.0, fc4=480.0, eps=1e-3)

def model(P, q, qd, qdd, d, dd, ddd):
    r = d + P['c']
    t3 = (P['Izz'] + P['Md']*r**2)*qdd + 2*P['Md']*r*dd*qd + P['Md']*g*r*np.cos(q) + P['fv3']*qd + P['fc3']*np.tanh(qd/P['eps'])
    f4 = P['Md']*ddd - P['Md']*r*qd**2 + P['Md']*g*np.sin(q) + P['fv4']*dd + P['fc4']*np.tanh(dd/P['eps'])
    return t3, f4

def run(q3B_deg, q3A_deg, d4B, d4A, w, F0=100e3, wF=2*np.pi/9, Tend=65.0, trim=5.0):
    q3B, q3A = np.deg2rad(q3B_deg), np.deg2rad(q3A_deg)
    wc = 15.0
    J3 = T['Izz'] + T['Md']*(d4B + T['c'])**2
    Kp3, Kd3 = J3*wc**2, 2*J3*wc
    Ki3 = Kp3*wc/4.0
    Kp4, Kd4 = T['Md']*wc**2, 2*T['Md']*wc
    Ki4 = Kp4*wc/4.0
    
    I30 = T['Md']*g*(d4B + T['c'])*np.cos(q3B)/Ki3
    I40 = T['Md']*g*np.sin(q3B)/Ki4
    
    def ctrl(t, x):
        q, qd, d, dd, i3, i4 = x
        e3 = q3B + q3A*np.sin(w*t) - q
        ed3 = q3A*w*np.cos(w*t) - qd
        e4 = d4B + d4A*np.sin(w*t) - d
        ed4 = d4A*w*np.cos(w*t) - dd
        return Kp3*e3 + Ki3*i3 + Kd3*ed3, Kp4*e4 + Ki4*i4 + Kd4*ed4, e3, e4
        
    def f(t, x):
        q, qd, d, dd, i3, i4 = x
        r = d + T['c']
        Fy = F0*np.sin(wF*t)
        u3, u4, e3, e4 = ctrl(t, x)
        Q3 = (d + 1.5)*Fy*np.cos(q)
        Q4 = Fy*np.sin(q)
        M = T['Izz'] + T['Md']*r**2
        qdd = (u3 + Q3 - (2*T['Md']*r*dd*qd + T['Md']*g*r*np.cos(q) + T['fv3']*qd + T['fc3']*np.tanh(qd/T['eps']))) / M
        ddd = (u4 + Q4 - (-T['Md']*r*qd**2 + T['Md']*g*np.sin(q) + T['fv4']*dd + T['fc4']*np.tanh(dd/T['eps']))) / T['Md']
        return [qd, qdd, dd, ddd, e3, e4]
        
    x0 = [q3B, q3A*w, d4B, d4A*w, I30, I40]
    te = np.linspace(0, Tend, int(round(Tend*300)) + 1)
    sol = solve_ivp(f, (0, Tend), x0, method='Radau', t_eval=te, rtol=1e-8, atol=1e-9, max_step=5e-3)
    t = sol.t
    X = sol.y
    k = t >= trim
    q, qd, d, dd = X[0], X[1], X[2], X[3]
    acc = np.array([f(ti, X[:, i]) for i, ti in enumerate(t)]).T
    qdd, ddd = acc[1], acc[3]
    u3 = np.array([ctrl(ti, X[:, i])[0] for i, ti in enumerate(t)])
    Fy = F0*np.sin(wF*t)
    s = d + 1.5
    
    Pm = dict(T)
    Pm['Md'] *= 1.05
    Pm['Izz'] *= 1.10
    Pm['c'] += 0.05
    m3m, _ = model(Pm, q, qd, qdd, d, dd, ddd)
    Fym = (m3m - u3)/(s*np.cos(q))
    em = (Fym - Fy)[k]
    qm = np.rad2deg(q[k])
    
    print(f"q3 mid {(qm.max()+qm.min())/2:+.3f} half {(qm.max()-qm.min())/2:.3f} deg | d4 mid {(d[k].max()+d[k].min())/2:.4f} half {(d[k].max()-d[k].min())/2:.4f} m | "
          f"max track err q3 {np.rad2deg(np.max(np.abs(q-(q3B+q3A*np.sin(w*t)))[k])):.3f} deg | mismatch Fy bias {em.mean()/1e3:+.2f} kN rms {np.sqrt((em**2).mean())/1e3:.2f} kN")

if __name__ == '__main__':
    print("Closed-loop sanity check (Revision 2: Tend=65 s, trim=5 s -> 60 s window):")
    run(45, 15, 8.75, 2.0, np.pi/10, Tend=65.0, trim=5.0)
    run(-12, 9, 7.325, 0.275, 0.51, Tend=65.0, trim=5.0)
