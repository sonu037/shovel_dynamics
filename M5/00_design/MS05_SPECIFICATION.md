# Milestone 5 Technical Specification: Electromechanical Drive Dynamics & Inverter-Fed Telemetry

**Project:** AI-Enabled Platform for Diagnosis, Prognosis and Health Management of Mining Electrical Systems  
**Grant / Code:** CMPDI MT-189 | `IIT/SRIC/R/AEH/2026/104`  
**Host Institution:** Department of Mining Engineering & Department of Electrical Engineering, IIT Kharagpur  
**Target Equipment:** Taiyuan Heavy Industry (TZ) WK-20 Electric Rope Shovel (`Machine B`, NCL Amlohri Mine)  
**Authors:** Sonu Suhail, Prof. Dr. Aurobinda Routray, Prof. Dr. Snehamoy Chatterjee  
**Status:** Approved for Implementation (MS05P01 – MS05P04)  
**Date:** November 16, 2026  

---

## 1. Executive Summary & Physical Motivation

In mining operations at Northern Coalfields Limited (NCL Amlohri), **there are no direct torque or force sensors** installed on the dipper handle, saddle block pivot, or crowd rack. The only signals streamed to the central PLC (Siemens SIMATIC S7-400 / CPU 319-3 PN/DP) and SCADA historian are **drive-side electrical telemetry** streamed via PROFIBUS DP from the variable frequency drive inverters (Siemens SINAMICS S120 Powerblock).

Milestone 5 bridges the gap between pure rigid-body mechanics (Milestones 1–4) and actual mine telemetry:
1. **Motor Electromagnetics & Field-Oriented Control (FOC):** Models the CRRC Yongji 690 VAC induction motors (dual 600 kW hoist, single 350 kW crowd) in the synchronous $d\text{-}q$ rotor-flux frame with field-weakening above base frequency ($25.4\,\mathrm{Hz}$).
2. **Drivetrain Transmission & Directional Power Flow:** Incorporates the verified Table II gearing ($G_h = 32.4$, $G_c = 48.6$), four-quadrant smooth directional mesh efficiency ($\eta^{\operatorname{sgn}(P_m)}$), and length-dependent wire rope elasticity.
3. **Acceleration-Free Extended Generalized Momentum:** Embeds the reflected motor inertia tensor ($\mathbf{D}_{\mathrm{rotor}}$) directly into the generalized inertia matrix without computing noisy resolver velocity derivatives ($\ddot{\theta}_m$), preserving exact passivity via a closed-form rank-1 Coriolis derivative.
4. **Thermal Drift & Inverter Telemetry Pipeline:** Simulates 30 Hz PROFIBUS DP logging (`r0021`–`r0039`) with anti-aliasing filtering, incorporating copper resistance thermal drift ($\Delta T \le +95^\circ\mathrm{C}$) as the empirical bridge to Milestone 9 health monitoring.

---

## 2. Mathematical Architecture (Four Coupled Layers)

### Layer 1: Motor Electromagnetics & Field-Oriented Control (FOC)

The hoist and crowd drives utilize three-phase squirrel-cage induction motors modeled in the synchronously rotating $d\text{-}q$ reference frame aligned with the rotor flux vector $\bm{\psi}_r$ ($\psi_{rq} = 0, \psi_{rd} = \psi_r$):

$$\begin{aligned}
v_{sd} &= R_s i_{sd} + \sigma L_s \frac{di_{sd}}{dt} - \omega_e \sigma L_s i_{sq} + \frac{L_m}{L_r} \frac{d\psi_r}{dt} \\
v_{sq} &= R_s i_{sq} + \sigma L_s \frac{di_{sq}}{dt} + \omega_e \sigma L_s i_{sd} + \omega_e \frac{L_m}{L_r} \psi_r
\end{aligned}$$

where leakage coefficient $\sigma = 1 - \frac{L_m^2}{L_s L_r}$.

#### Rotor Flux Dynamics:
$$\tau_r \frac{d\psi_r}{dt} + \psi_r = L_m i_{sd}, \qquad \tau_r = \frac{L_r}{R_r(T)}$$

#### Slip Frequency & Synchronous Speed:
$$\omega_{\mathrm{sl}} = \omega_e - p \omega_m = \frac{L_m}{\tau_r \psi_r} i_{sq}, \qquad \omega_e = p \omega_m + \omega_{\mathrm{sl}}$$

#### Electromagnetic Shaft Torque:
$$\tau_e = \frac{3}{2} p \frac{L_m}{L_r} \psi_r i_{sq}$$
where $p = 2$ (4-pole machine).

#### Field Weakening Regime ($f > f_{\mathrm{base}} = 25.4\,\mathrm{Hz}$, $\omega_m > 79.8\,\mathrm{rad/s}$):
$$\psi_r^*(\omega_m) = \psi_{r,\mathrm{rated}} \cdot \min\left(1, \; \frac{\omega_{m,\mathrm{base}}}{\omega_m}\right)$$
permitting bucket lowering speeds up to $33.2\,\mathrm{Hz}$ ($1.58\,\mathrm{m/s}$) at constant rated stator voltage ($690\,\mathrm{V}$).

#### Thermal Drift Law:
$$R_r(T) = R_{r0} \left[1 + \alpha_{\mathrm{Cu}}(T - T_0)\right], \qquad \tau_r(T) = \frac{L_r}{R_r(T)}$$
where $\alpha_{\mathrm{Cu}} = 0.00393\,\mathrm{K^{-1}}$. For $\Delta T = +95\,\mathrm{K}$, $R_r$ increases by $37.3\%$.

---

### Layer 2: Drivetrain Transmission & Directional Efficiency

#### Kinematics:
$$\theta_{\mathrm{drum}} = \frac{\theta_{m,h}}{G_h}, \quad v_{\mathrm{rope}} = R_{\mathrm{drum}} \dot{\theta}_{\mathrm{drum}}, \quad v_{\mathrm{bail}} = \frac{v_{\mathrm{rope}}}{n_{\mathrm{falls}}}$$
$$\dot{d}_4 = \frac{R_{\mathrm{pinion}}}{G_c} \dot{\theta}_{m,c}$$

#### Smooth $C^\infty$ Directional Efficiency (Verstraten et al., 2016):
$$\gamma_\eta(P_m) = \left(\frac{1 + \tanh(P_m / P_\varepsilon)}{2}\right) \eta_{\mathrm{mot}} + \left(\frac{1 - \tanh(P_m / P_\varepsilon)}{2}\right) \frac{1}{\eta_{\mathrm{gen}}}$$
$$\tau_{\mathrm{trans}} = \gamma_\eta(P_m) \cdot (G \cdot \tau_m)$$
where $P_m = \tau_m \omega_m$, $\eta_{\mathrm{mot}} = 0.93$, $\eta_{\mathrm{gen}} = 0.90$, and transition deadband $P_\varepsilon = 5.0\,\mathrm{kW}$.

#### Wire Rope Elasticity:
$$K_{\mathrm{rope}}(q_3, d_4) = \frac{n_{\mathrm{falls}} E_{\mathrm{rope}} A_{\mathrm{metal}}}{L_{\mathrm{rope}}(q_3, d_4)}$$
$$\Delta L_{\mathrm{rope}} = \frac{F_{\mathrm{hoist}}}{K_{\mathrm{rope}}}$$
where $L_{\mathrm{rope}}(q_3, d_4) = \sqrt{D_c(q_3, d_4)^2 - R_{\mathrm{sheave}}^2}$ using exact tangent-circle geometry ($R_{\mathrm{sheave}} = 1.085\,\mathrm{m}$).

---

### Layer 3: Spong Extended Dynamics (Acceleration-Free Formulation)

#### Transmission Jacobian:
$$\dot{\bm{\theta}}_m = \mathbf{J}_T(\mathbf{q}) \dot{\mathbf{q}}, \qquad \mathbf{J}_T(\mathbf{q}) = \begin{bmatrix} \frac{n_{\mathrm{falls}} G_h}{R_{\mathrm{drum}}} s_{\mathrm{bail}}(d_4) \sin\alpha_3(q_3, d_4) & \frac{n_{\mathrm{falls}} G_h}{R_{\mathrm{drum}}} \cos\alpha_3(q_3, d_4) \\ 0 & \frac{G_c}{R_{\mathrm{pinion}}} \end{bmatrix}$$

#### Extended Generalized Inertia Matrix:
$$\mathbf{D}_{\mathrm{em}}(\mathbf{q}) = \mathbf{D}_{\mathrm{rigid}}(d_4) + \mathbf{J}_T^T(\mathbf{q}) \mathbf{M}_{m,\mathrm{rotor}} \mathbf{J}_T(\mathbf{q})$$

#### Rank-1 Coriolis Passivity Vector:
$$\mathbf{C}_{\mathrm{em}}(\mathbf{q}, \dot{\mathbf{q}})^T \dot{\mathbf{q}} = \begin{bmatrix} 0 \\ M_d(d_4 + c)\dot{q}_3^2 \end{bmatrix} + M_{\mathrm{refl},h} \left(\mathbf{j}_h^T \dot{\mathbf{q}}\right) \begin{bmatrix} \frac{\partial \mathbf{j}_h^T}{\partial q_3} \dot{\mathbf{q}} \\ \frac{\partial \mathbf{j}_h^T}{\partial d_4} \dot{\mathbf{q}} \end{bmatrix}$$
where $\mathbf{j}_h(q_3, d_4) = [s_{\mathrm{bail}}\sin\alpha_3; \, \cos\alpha_3]$ and $M_{\mathrm{refl},h} = \left(\frac{n_{\mathrm{falls}} G_h}{R_{\mathrm{drum}}}\right)^2 (2 J_{m,h})$.

---

### Layer 4: Multi-Rate Innovation-Gated Disturbance Observer

1. **Generalized Momentum**:
   $$\mathbf{p}_{\mathrm{em}} = \mathbf{D}_{\mathrm{em}}(\mathbf{q}) \dot{\mathbf{q}}$$
2. **Exact Discrete First-Order Hold (FOH) State Transition**:
   $$\mathbf{z}_{k+1} = \Phi_d \mathbf{z}_k + \mathbf{\beta}_0 \mathbf{u}_{k-1} + \mathbf{\beta}_1 \mathbf{u}_k$$
   $$\hat{\mathbf{\tau}}_L[k] = \mathbf{K}_O \left( \mathbf{p}_{\mathrm{em}}[k] - \mathbf{z}_k \right)$$
3. **Adaptive Innovation Gating (Boulders & Impact Stalls)**:
   $$\mathrm{NIS}_k = \bm{\nu}_k^T \mathbf{S}_k^{-1} \bm{\nu}_k$$
   $$\mathbf{Q}_k = \mathbf{Q}_0 \left[ 1 + \gamma \max\left(0, \; \mathrm{NIS}_k - \chi_{\mathrm{threshold}}^2\right) \right]$$

---

## 3. Protocol Verification Breakdown (MS05P01 – MS05P04)

| Protocol | Script Name | Verification Target | Acceptance Metric |
| :--- | :--- | :--- | :--- |
| **MS05P01** | `motor_foc_wk20.m` | CRRC Yongji YJ56B1 / YJ80A1 FOC, rotor flux dynamics, and field weakening. | Dual hoist torque $\tau_{e,\mathrm{rated}} = 15.28\,\mathrm{kN\cdot m}$, peak $1,500\,\mathrm{kW}$ at $25.4\,\mathrm{Hz}$, field-weakening envelope to $33.2\,\mathrm{Hz}$. |
| **MS05P02** | `drivetrain_transmission.m` | Gearbox transmission, smooth $\tanh$ 4-quadrant efficiency, rope elasticity. | Continuous torque through zero-crossings ($C^\infty$), cable stretch bounded $\le 54\,\mathrm{mm}$, rated bail pull $935\,\mathrm{kN}$, stall $1,542\,\mathrm{kN}$. |
| **MS05P03** | `electromechanical_gmo.m` | Spong extended momentum observer with rank-1 Coriolis compensation. | Zero acceleration differentiation, zero false torque spikes ($<1\,\mathrm{N\cdot m}$ on unloaded motion), 100% discrete stability. |
| **MS05P04** | `run_MS05_Verification_Suite.m` | End-to-end integration: power balance, boulder impact tracking, and $+95^\circ\mathrm{C}$ thermal drift. | Energy conservation error $<0.1\%$, boulder impact lag $<33\,\mathrm{ms}$, thermal drift detuning mapped and archived to `.mat`. |
