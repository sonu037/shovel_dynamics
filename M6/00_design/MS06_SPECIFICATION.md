# Milestone 6 Specification: 3D 4-DOF Swing-Coupled Dynamics & Dynamic Hoist Wire-Rope Compliance

**Equipment:** Taiyuan Heavy Industry (TZ) WK-20 Mining Cable Shovel (780 tonnes, $20\,\mathrm{m^3}$)  
**Mine Site:** Northern Coalfields Limited (NCL), Amlohri Open-Cast Project (`Machine B`, CMPDI MT-189)  
**Target Journals:** *IEEE Transactions on Mechatronics* / *IEEE Transactions on Robotics* / *Mechanical Systems and Signal Processing*  
**Date:** 2026-11-17  
**Status:** **LOCKED TECHNICAL SPECIFICATION**

---

## 1. Physical Motivation & Scope

In Milestones 1–5, the shovel was restricted to the 2-DOF planar vertical digging subspace $(q_3, d_4)$ under an inextensible kinematic hoist cable constraint. In actual open-pit production cycles (e.g. video `VID20261009112113.mp4`):
1. **Out-of-Plane Swing Coupling ($q_1$):**
   The revolving upper deck swings at speeds up to $\dot{q}_1 = 3.5\,\mathrm{rpm}$ ($0.3665\,\mathrm{rad/s}$). The dipper handle is located at a large radial lever arm ($\rho_{\mathrm{COM}} \in [8.1, 16.5]\,\mathrm{m}$), generating up to $150\,\mathrm{kN}$ of outward centrifugal pull on the crowd drive ($20\%$ of motor stall) and $480\,\mathrm{kN\cdot m}$ of Coriolis reaction torque on the swing turntable bull gear. A planar observer mistakes this centrifugal force for an external bank obstruction!
2. **Dynamic Wire-Rope Compliance ($x_{\mathrm{drum}} / \delta_{\mathrm{rope}}$):**
   The twin $52\,\mathrm{mm}$ hoist ropes have an axial stiffness of $K_{\mathrm{rope}} \approx 25\text{--}45\,\mathrm{MN/m}$. Coupled to the dipper mass ($M_d = 56,500\,\mathrm{kg}$ empty, $92,500\,\mathrm{kg}$ loaded) and winch rotor inertia ($M_{\mathrm{refl},h} = 229,380\,\mathrm{kg}$), the rope-bounce mode resonates at $f_{\mathrm{rope}} = 4.35\,\mathrm{Hz}$. Rigid-rope observers suffer from violent $4.35\,\mathrm{Hz}$ false oscillation spikes whenever the dipper breaks out of the bank.
3. **Lateral Side-Raking & Boom-Foot Torsion:**
   Non-collinear boulder collisions or operator slewing during bank engagement exert out-of-plane lateral forces $F_{\mathrm{lat}}$ at the bucket teeth. This induces severe torsional stress at the boom foot pins, identified in literature (Li & Frimpong 2008, Raza & Frimpong 2017) as the primary cause of boom weld cracking.

Milestone 6 expands the state space to **4 Degrees of Freedom**:
$$\mathbf{q} = \begin{bmatrix} q_1 \\ q_3 \\ d_4 \\ x_{\mathrm{drum}} \end{bmatrix} \in \mathbb{R}^4, \quad \dot{\mathbf{q}} = \begin{bmatrix} \dot{q}_1 \\ \dot{q}_3 \\ \dot{d}_4 \\ \dot{x}_{\mathrm{drum}} \end{bmatrix} \in \mathbb{R}^4$$

---

## 2. 3D Spatial Kinematics & Coordinate Frames

### 2.1 Coordinate Frame Hierarchy (Matching Simscape Multibody)
- **World Frame $\{0\}$:** Origin on the ground at the turntable kingpin centerline. $+Z_0$ vertically upward, $+X_0$ forward along the crawler tracks.
- **Revolving Deck Frame $\{1\}$:** Rotated by swing angle $q_1$ about $+Z_0$. $+X_1$ points forward along the boom centerline, $+Z_1 = +Z_0$.
- **Saddle Pivot Frame $\{3\}$:** Located at fixed horizontal offset $X_{O3} = 8.117\,\mathrm{m}$ and vertical height $Z_{O3} = 5.250\,\mathrm{m}$ relative to $\{1\}$. Rotated by saddle pitch angle $q_3$ about transverse horizontal axis $+Y_3$ (or $+Z_3$ in standard DH, CCW from horizontal).
- **Dipper Handle Frame $\{4\}$:** Translates by prismatic crowd extension $d_4$ along handle axis $+X_4$.
- **Bail Pin Frame $\{B\}$:** Located at radial distance $s_{\mathrm{bail}}(d_4) = d_4 + x_{\mathrm{ref}}$ ($x_{\mathrm{ref}} = 1.50\,\mathrm{m}$) along the handle.
- **Dipper Tooth Tip $\{E\}$:** Located at tooth tip offset $L_{\mathrm{tip}} = 2.85\,\mathrm{m}$ forward of bail pin.

### 2.2 Dipper Center of Mass Radius
The dipper-handle composite Center of Mass (COM) has handle coordinate $r(d_4) = d_4 + c$ where $c = -1.32\,\mathrm{m}$. Its instantaneous horizontal radial distance from the vertical swing axis is:
$$\rho_{\mathrm{COM}}(q_3, d_4) = X_{O3} + (d_4 + c)\cos q_3 \quad [\mathrm{m}]$$

Its spatial Cartesian position in the world frame is:
$$\mathbf{r}_{\mathrm{COM}}(q_1, q_3, d_4) = \begin{bmatrix} \rho_{\mathrm{COM}}(q_3, d_4) \cos q_1 \\ \rho_{\mathrm{COM}}(q_3, d_4) \sin q_1 \\ Z_{O3} + (d_4 + c)\sin q_3 \end{bmatrix} \quad [\mathrm{m}]$$

---

## 3. 4-DOF Variational Mechanics & Euler-Lagrange Formulation

### 3.1 Kinetic Energy & Extended 4x4 Mass Matrix $\mathbf{M}_{\mathrm{4DOF}}(\mathbf{q})$

Total system kinetic energy partitions by Spong & Tomei (1987, 1991):
$$\mathcal{K} = \frac{1}{2}\dot{\mathbf{q}}^T \mathbf{M}_{\mathrm{4DOF}}(\mathbf{q}) \dot{\mathbf{q}} = \frac{1}{2}\dot{\mathbf{q}}_L^T \mathbf{D}_L(\mathbf{q}_L) \dot{\mathbf{q}}_L + \frac{1}{2} M_{\mathrm{drum}} \dot{x}_{\mathrm{drum}}^2$$
where $\mathbf{q}_L = [q_1, q_3, d_4]^T \in \mathbb{R}^3$ are the link coordinates, and $x_{\mathrm{drum}}$ is the winch drum payout coordinate.

#### Exact Block-Diagonal Mass Matrix:
$$\mathbf{M}_{\mathrm{4DOF}}(\mathbf{q}) = \begin{bmatrix} D_{11}(q_3, d_4) & 0 & 0 & 0 \\ 0 & D_{22}(d_4) & 0 & 0 \\ 0 & 0 & D_{33} & 0 \\ 0 & 0 & 0 & M_{\mathrm{drum}} \end{bmatrix}$$

1. **3D Swing Inertia $D_{11}(q_3, d_4)$:**
   $$D_{11}(q_3, d_4) = I_{\mathrm{upper},zz} + G_{\mathrm{swing}}^2(2 J_{m,\mathrm{swing}}) + J_{\mathrm{rot},11}(q_3) + M_d \left[X_{O3} + (d_4 + c)\cos q_3\right]^2$$
   where $I_{\mathrm{upper},zz} = 2,500,000\,\mathrm{kg\cdot m^2}$ (upper-works + boom + counterweight), $G_{\mathrm{swing}} = 124.0$, $J_{m,\mathrm{swing}} = 1.955\,\mathrm{kg\cdot m^2}$ (dual YJ80A1 motors), and the **3D non-spherical attitude inertia** is:
   $$J_{\mathrm{rot},11}(q_3) = I_{xx}\sin^2 q_3 + I_{yy}\cos^2 q_3$$
2. **Saddle Pitch Inertia $D_{22}(d_4)$:**
   $$D_{22}(d_4) = I_{zz} + M_d (d_4 + c)^2 \quad [\mathrm{kg\cdot m^2}]$$
   where $I_{zz} = 305,000\,\mathrm{kg\cdot m^2}$.
3. **Crowd Mass $D_{33}$:**
   $$D_{33} = M_d + M_{\mathrm{refl},c} = 56,500 + 302,401.8 = \mathbf{358,901.8\,\mathrm{kg}}$$
   where $M_{\mathrm{refl},c} = (G_c / R_p)^2 J_{m,c} = (48.6 / 0.340)^2 \times 14.8 = 302,401.8\,\mathrm{kg}$.
4. **Winch Drum Inertia $M_{\mathrm{drum}}$:**
   $$M_{\mathrm{drum}} = M_{\mathrm{refl},h} + \frac{1}{3} n_{\mathrm{falls}} \rho_{\mathrm{rope}} L_{\mathrm{cat}} = 229,379.7 + 145.0 = \mathbf{229,524.7\,\mathrm{kg}}$$

### 3.2 Exact 4x4 Christoffel Matrix $\mathbf{C}_{\mathrm{4DOF}}(\mathbf{q}, \dot{\mathbf{q}})$

Using the Christoffel symbols $c_{ijk} = \frac{1}{2}\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)$, the matrix $\mathbf{C}$ satisfies the skew-symmetry property:
$$\mathbf{z}^T \left( \dot{\mathbf{M}}_{\mathrm{4DOF}} - 2 \mathbf{C}_{\mathrm{4DOF}} \right) \mathbf{z} = 0, \quad \forall \mathbf{z} \in \mathbb{R}^4$$

#### Explicit Matrix Entries:
$$\mathbf{C}_{\mathrm{4DOF}}(\mathbf{q}, \dot{\mathbf{q}}) = \begin{bmatrix}
C_{11} & C_{12} & C_{13} & 0 \\
C_{21} & C_{22} & C_{23} & 0 \\
C_{31} & C_{32} & 0 & 0 \\
0 & 0 & 0 & 0
\end{bmatrix}$$

where:
$$\begin{aligned}
C_{12} &= \frac{1}{2}\frac{\partial D_{11}}{\partial q_3} \dot{q}_1 = \left[ -M_d \rho_{\mathrm{COM}} (d_4 + c)\sin q_3 + \frac{1}{2}(I_{xx} - I_{yy})\sin(2q_3) \right] \dot{q}_1 \\
C_{13} &= \frac{1}{2}\frac{\partial D_{11}}{\partial d_4} \dot{q}_1 = \left[ M_d \rho_{\mathrm{COM}} \cos q_3 \right] \dot{q}_1 \\
C_{11} &= C_{12} \frac{\dot{q}_3}{\dot{q}_1} + C_{13} \frac{\dot{d}_4}{\dot{q}_1} = -M_d \rho_{\mathrm{COM}} (d_4 + c)\sin q_3 \dot{q}_3 + \frac{1}{2}(I_{xx} - I_{yy})\sin(2q_3)\dot{q}_3 + M_d \rho_{\mathrm{COM}}\cos q_3 \dot{d}_4 \\
C_{21} &= -C_{12} = \left[ M_d \rho_{\mathrm{COM}} (d_4 + c)\sin q_3 - \frac{1}{2}(I_{xx} - I_{yy})\sin(2q_3) \right] \dot{q}_1 \\
C_{22} &= M_d (d_4 + c) \dot{d}_4 \\
C_{23} &= M_d (d_4 + c) \dot{q}_3 \\
C_{31} &= -C_{13} = -M_d \rho_{\mathrm{COM}} \cos q_3 \dot{q}_1 \\
C_{32} &= -M_d (d_4 + c) \dot{q}_3
\end{aligned}$$

#### Physical Cross-Coupling Forces:
- **Outward Centrifugal Pull on Crowd ($F_{4,\mathrm{cent}}$):**
  $$F_{4,\mathrm{cent}} = C_{31} \dot{q}_1 = -M_d \rho_{\mathrm{COM}}(q_3, d_4) \cos q_3 \, \dot{q}_1^2 \quad [\mathrm{N}]$$
- **Pitch Nutation Moment on Saddle ($\tau_{3,\mathrm{cent}}$):**
  $$\tau_{3,\mathrm{cent}} = C_{21} \dot{q}_1 = \left[ M_d \rho_{\mathrm{COM}}(q_3, d_4)(d_4 + c)\sin q_3 - \frac{1}{2}(I_{xx} - I_{yy})\sin(2q_3) \right] \dot{q}_1^2 \quad [\mathrm{N\cdot m}]$$
- **Coriolis Reaction Torque on Swing ($\tau_{1,\mathrm{Coriolis}}$):**
  $$\tau_{1,\mathrm{Coriolis}} = 2 (C_{12}\dot{q}_3 + C_{13}\dot{d}_4) \dot{q}_1 \quad [\mathrm{N\cdot m}]$$

---

## 4. Dynamic Wire-Rope Subsystem & Elastic Potential

### 4.1 Catenary + Suspended Span Geometry
The instantaneous suspended rope length from boom sheave to bail pin is:
$$\ell_{\mathrm{bail}}(q_3, d_4) = \sqrt{D_{\mathrm{center}}^2 - R_{\mathrm{sheave}}^2} + R_{\mathrm{sheave}} \Delta\theta_{\mathrm{wrap}}(q_3, d_4) \quad [\mathrm{m}]$$
The effective total compliant length is $L_{\mathrm{eff}}(q_3, d_4) = L_{\mathrm{cat}} + \ell_{\mathrm{bail}}(q_3, d_4)$, where $L_{\mathrm{cat}} = 14.50\,\mathrm{m}$ is the inclined boom-catenary span.

The configuration-dependent axial rope stiffness is:
$$K_{\mathrm{rope}}(q_3, d_4) = \frac{n_{\mathrm{falls}} E_{\mathrm{rope}} A_{\mathrm{metal}}}{L_{\mathrm{eff}}(q_3, d_4)} = \frac{2.710 \times 10^8}{L_{\mathrm{eff}}(q_3, d_4)} \quad [\mathrm{N/m}]$$

### 4.2 Smooth $C^\infty$ Unilateral Slack-Snap Law
Let dynamic rope stretch be $\delta_{\mathrm{rope}} = x_{\mathrm{drum}} - \ell_{\mathrm{bail}}(q_3, d_4)$ and stretch rate $\dot{\delta}_{\mathrm{rope}} = \dot{x}_{\mathrm{drum}} - \mathbf{j}_h(q_3, d_4)^T [\dot{q}_3; \dot{d}_4]$.
To prevent tension under compression while avoiding discontinuous solver stalls:
$$\sigma_+(\delta) = \frac{1}{\beta_{\mathrm{slack}}} \ln\left(1 + e^{\beta_{\mathrm{slack}} \delta}\right), \quad \beta_{\mathrm{slack}} = 5,000\,\mathrm{m^{-1}}$$
$$T_{\mathrm{rope}}(\mathbf{q}, \dot{\mathbf{q}}) = \sigma_+(\delta_{\mathrm{rope}}) \max\left(0, \; K_{\mathrm{rope}}(q_3, d_4)\delta_{\mathrm{rope}} + C_{\mathrm{rope}}(q_3, d_4)\dot{\delta}_{\mathrm{rope}}\right)$$
where $C_{\mathrm{rope}} = 2\zeta_{\mathrm{rope}} \sqrt{K_{\mathrm{rope}} M_{\mathrm{red}}}$, $\zeta_{\mathrm{rope}} = 0.035$, and $M_{\mathrm{red}} = \frac{M_{\mathrm{drum}} M_d}{M_{\mathrm{drum}} + M_d} = 45,334\,\mathrm{kg}$.

---

## 5. Dual-Layer De Luca–Spong Momentum Observer

Because $\mathbf{M}_{\mathrm{4DOF}}$ is block-diagonal, the observer decouples into two passivity-guaranteed layers:

### 5.1 Layer A: Sensorless Winch Drum Tension Observer ($\hat{T}_{\mathrm{rope}}$)
$$p_{\mathrm{drum}} = M_{\mathrm{drum}} \dot{x}_{\mathrm{drum}}$$
$$\dot{z}_{\mathrm{drum}} = F_{\mathrm{hoist,drive}} - F_{\mathrm{fric,drum}}(\dot{x}_{\mathrm{drum}}) + k_{O,\mathrm{drum}} (p_{\mathrm{drum}} - z_{\mathrm{drum}})$$
$$\hat{T}_{\mathrm{rope}}[k] = k_{O,\mathrm{drum}} (z_{\mathrm{drum}}[k] - p_{\mathrm{drum}}[k])$$

### 5.2 Layer B: 3D Link-Side Generalized Momentum Observer ($\hat{\boldsymbol{\tau}}_{\mathrm{ext},L} \in \mathbb{R}^3$)
$$p_L = \mathbf{D}_L(\mathbf{q}_L) \dot{\mathbf{q}}_L$$
$$\dot{\mathbf{z}}_L = \boldsymbol{\tau}_{\mathrm{act},L}(\hat{T}_{\mathrm{rope}}) + \mathbf{C}_L^T(\mathbf{q}_L, \dot{\mathbf{q}}_L)\dot{\mathbf{q}}_L - \mathbf{G}_L(\mathbf{q}_L) - \mathbf{F}_{\mathrm{fric},L}(\dot{\mathbf{q}}_L) + \mathbf{K}_{O,L} (\mathbf{p}_L - \mathbf{z}_L)$$
$$\hat{\boldsymbol{\tau}}_{\mathrm{ext},L}[k] = \mathbf{K}_{O,L} (\mathbf{z}_L[k] - \mathbf{p}_L[k])$$

### 5.3 3D Tooth-Tip Force Reconstruction
$$\hat{\mathbf{F}}_{\mathrm{tip}} = \begin{bmatrix} F_{\mathrm{tangential}} \\ F_{\mathrm{normal}} \\ F_{\mathrm{lateral}} \end{bmatrix} = \mathbf{J}_{\mathrm{tip,3D}}^{-T}(\mathbf{q}_L) \hat{\boldsymbol{\tau}}_{\mathrm{ext},L}$$
Out-of-plane side-raking force isolating boom-foot torsion:
$$F_{\mathrm{lateral}}(t) = \frac{\hat{\tau}_{\mathrm{ext},1}(t)}{\rho_{\mathrm{tip}}(q_3, d_4)}$$

---

## 6. Verification Protocol Matrix (MS06P01 – MS06P04)

| Protocol | Script / Target | Benchmark Regime | Acceptance Metric |
| :--- | :--- | :--- | :--- |
| **MS06P01** | `kinematics_dynamics_4dof.m` | 3D 4-DOF Variational Mechanics & Gyroscopic Nutation | Skew-symmetry $|\mathbf{z}^T(\dot{\mathbf{M}}-2\mathbf{C})\mathbf{z}| < 10^{-12}$, centrifugal pull $F_{4,\mathrm{cent}} = 148.48\,\mathrm{kN}$ |
| **MS06P02** | `elastic_rope_subsystem.m` | Dynamic Wire-Rope Compliance & Slack Physics | Reduced mass $M_{\mathrm{red}} = 45,334\,\mathrm{kg}$, bounce frequency $f_{\mathrm{rope}} = 4.35\,\mathrm{Hz}$ ($< 1\%$ error) |
| **MS06P03** | `observer_4dof_gmo.m` | Dual-Layer Spong-De Luca 4-DOF Disturbance Observer | Centrifugal false disturbance $< 0.1\%$, $4.35\,\mathrm{Hz}$ whip rejection $> 99\%$, $F_{\mathrm{lat}}$ isolation |
| **MS06P04** | `run_MS06_Verification_Suite.m` | End-to-End Simscape Multibody Verification Suite | Inverse-dynamics error $< 10^{-5}$ on `.slx`, dual-channel payload ID error $< 1.5\%$ on $36\,\mathrm{t}$ load |
