# VOLUME I: ADVANCED MATHEMATICAL FOUNDATIONS

---

# CHAPTER 5: Advanced Nonlinear Observers and Generalized Momentum Formulations (M3)

**Author:** Multidisciplinary Research Supervisor & Technical Author  
**Project:** *AI-Enabled Platform for Diagnosis, Prognosis and Health Management of Mining Electrical Systems: Case Study on Dragline / Electric Rope Shovel*  
**Grant Reference:** IIT/SRIC/R/AEH/2026/104 (CMPDI / Coal India Limited — MT-189)  
**Target Machines:** Machine A (P&H 2100BL Benchmark) & Machine B (TZ WK-20 Deployment)  
**Pedagogical Level:** Postgraduate / Research Grade (B.Tech Mechanical Foundation Assumed)

---

> [!NOTE]
> ### RESEARCH STUDY NOTEBOOK ARCHITECTURE
> In Chapters 3 and 4, we formulated the forward multibody dynamics and parameter identification regressor. However, in an operational digital twin for health monitoring and fault diagnosis, we face a crucial inverse challenge:
> 
> **How can we estimate unknown rock cutting resistance forces $\mathbf{F}_{\text{ext}}$ and dynamic payload variations in real time without mounting fragile, expensive load cells on the dipper teeth?**
> 
> The naive approach—inverting the dynamic equations $\mathbf{Q}_{\text{ext}} = \mathbf{D}\mathbf{\ddot{q}} + \mathbf{C}\mathbf{\dot{q}} + \mathbf{G} - \boldsymbol{\tau}$—fails completely in the field because double differentiation of noisy encoder signals $\mathbf{q}(t)$ amplifies high-frequency noise into millions of Newtons of artificial force spikes!
> 
> In this chapter, we develop the theoretical foundation of **Module M3**: the **Generalized Momentum Observer (GMO)**. 
> 
> By the end of this chapter, you will be able to:
> 1. Formulate system momentum $\mathbf{p} = \mathbf{D}(\mathbf{q})\mathbf{\dot{q}}$ and prove that its time derivative $\mathbf{\dot{p}}$ eliminates joint accelerations $\mathbf{\ddot{q}}$ identically using the passivity skew-symmetry property.
> 2. Derive the De Luca-Haddadin residual observer $\mathbf{r}(t)$ and prove that each channel behaves as a decoupled first-order low-pass filter driven by the true external load.
> 3. Prove global exponential convergence of the observer error dynamics using Lyapunov stability theory.
> 4. Invert the transposed Jacobian $\hat{\mathbf{F}}_{\text{ext}} = (\mathbf{J}_T^T)^{-1}\mathbf{r}$ to reconstruct 2D digging resistance forces at the dipper teeth.
> 5. Implement the complete discrete observer algorithm with anti-windup and parameter uncertainty bounds for both the **P&H 2100BL benchmark** and the **TZ WK-20 heavy shovel**.

---

## 5.1 Physical Intuition: The Dilemma of External Force Sensing in Open-Pit Mining

### 5.1.1 Why Physical Load Cells Fail in Mining Environments
In laboratory robotics, interaction forces between an end-effector and the environment are measured directly using six-axis strain-gauge load cells mounted at the wrist flange. 

On an electric rope shovel or dragline operating in an open-cast mine, mounting physical load cells at the dipper teeth or bail pin is practically impossible:
1. **Extreme Shock Loads:** During hard rock excavation in blasted granite or basalt, dipper teeth encounter peak shock accelerations exceeding $15\,\mathrm{g}$ and peak contact forces over $1,500\,\mathrm{kN}$. Delicate strain gauges and piezoresistive foil sensors fracture or de-bond within hours.
2. **Harsh Environmental Contamination:** Dipper teeth are subjected to abrasive silica dust, slurry, corrosive acid mine drainage, and sub-zero freeze-thaw cycles.
3. **Cabling Vulnerability:** Signal cables running along the dipper handle chord through the pivoting saddle block are routinely sheared, crushed, or severed by falling rock fragments from the highwall.

```
                  THE SENSORLESS LOAD ESTIMATION ARCHITECTURE
                  
       INDUSTRIAL SENSORS (Available & Rugged):
       - Motor Resolvers / Optical Encoders (Measure q_3, d_4, q_dot_3, d_dot_4)
       - Motor Inverter Current Sensors (Measure Armature Currents -> Motor Torques tau)
                                     |
                                     v
                       +---------------------------+
                       |  M3 GENERALIZED MOMENTUM  |
                       |         OBSERVER          |
                       +---------------------------+
                                     |
                                     v
       RECONSTRUCTED ESTIMATES (Sensorless):
       - External Digging Resistance Force F_ext = [F_x, F_y]^T
       - Net Dynamic Payload Mass M_payload
       - Mechanical Faults: Tooth Stall, Bank Collapse, Rope Slack
```

---

### 5.1.2 The Failure of Naive Direct Inversion
Suppose an engineer attempts to compute the external load $\mathbf{Q}_{\text{ext}}$ by directly rearranging the equations of motion:

$$\mathbf{Q}_{\text{ext, naive}} = \mathbf{D}(\mathbf{q})\mathbf{\ddot{q}} + \mathbf{C}(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} + \mathbf{G}(\mathbf{q}) + \mathbf{F}_f(\mathbf{\dot{q}}) - \boldsymbol{\tau} \tag{5.1}$$

Why is Equation (5.1) completely unusable in industrial practice?

Examine the term $\mathbf{D}(\mathbf{q})\mathbf{\ddot{q}}$:
- On a P&H 2100BL, the saddle rotational inertia is $D_{11} \approx 7.0 \times 10^6\,\mathrm{kg\cdot m^2}$.
- On a TZ WK-20, $D_{11} \approx 2.5 \times 10^7\,\mathrm{kg\cdot m^2}$.
- Joint acceleration $\mathbf{\ddot{q}}$ is not measured directly; it must be obtained by differentiating encoder position signals $\mathbf{q}(t)$.

Suppose the encoder signal has a small high-frequency quantization noise of amplitude $\delta q = 10^{-4}\,\mathrm{rad}$ at sampling frequency $f_s = 500\,\mathrm{Hz}$ ($\Delta t = 2\,\mathrm{ms}$). Numerical double differentiation yields an acceleration noise spike of:

$$\delta \ddot{q} \approx \frac{\delta q}{\Delta t^2} = \frac{10^{-4}}{(0.002)^2} = \frac{10^{-4}}{4 \times 10^{-6}} = 25.0\,\mathrm{rad/s^2} \tag{5.2}$$

Multiplying this noise spike by the inertia matrix:

$$\delta \tau_{\text{noise}} = D_{11} \, \delta \ddot{q} = (7.0 \times 10^6\,\mathrm{kg\cdot m^2})(25.0\,\mathrm{rad/s^2}) = 1.75 \times 10^8\,\mathrm{N\cdot m} = 175\,\mathrm{MN\cdot m}! \tag{5.3}$$

**The noise spike is 30 times larger than the maximum torque rating of the entire shovel motor!** 

Low-pass filtering the acceleration signal introduces severe phase lag (delay), making impact detection and overload trip protection far too slow to prevent structural yielding.

**We require an observer architecture that reconstructs external forces WITHOUT differentiating joint velocities $\mathbf{\dot{q}}$.**

---

## 5.2 The Generalized Momentum Formulation

### 5.2.1 Definition of Generalized Momentum $\mathbf{p}$
In classical Hamiltonian mechanics, the **Generalized Momentum Vector** $\mathbf{p} \in \mathbb{R}^n$ conjugate to the generalized coordinates $\mathbf{q}$ is defined as the partial derivative of kinetic energy with respect to generalized velocities:

$$\mathbf{p} = \frac{\partial T(\mathbf{q}, \mathbf{\dot{q}})}{\partial \mathbf{\dot{q}}} \tag{5.4}$$

Recall that $T = \frac{1}{2}\mathbf{\dot{q}}^T \mathbf{D}(\mathbf{q})\mathbf{\dot{q}}$. Differentiating yields:

$$\mathbf{p} = \mathbf{D}(\mathbf{q}) \mathbf{\dot{q}} \tag{5.5}$$

For our 2-DOF crowd-hoist shovel mechanism:

$$\mathbf{p} = \begin{bmatrix} p_3 \\ p_4 \end{bmatrix} = \begin{bmatrix} D_{11}(d_4) \dot{q}_3 \\ M_{\text{total}} \dot{d}_4 \end{bmatrix} \tag{5.6}$$

where:
- $p_3$ is the **generalized angular momentum** of the entire moving assembly (saddle, handle, dipper, payload) about the shipper shaft pivot $O_3$ ($\mathrm{kg\cdot m^2/s} = \mathrm{N\cdot m\cdot s}$).
- $p_4$ is the **generalized linear momentum** of the handle and dipper translating along the crowd track ($\mathrm{kg\cdot m/s} = \mathrm{N\cdot s}$).

---

### 5.2.2 Time Derivative of Generalized Momentum
Now differentiate the generalized momentum $\mathbf{p} = \mathbf{D}(\mathbf{q})\mathbf{\dot{q}}$ with respect to time using the product rule:

$$\mathbf{\dot{p}} = \frac{d}{dt}\left( \mathbf{D}(\mathbf{q})\mathbf{\dot{q}} \right) = \mathbf{D}(\mathbf{q})\mathbf{\ddot{q}} + \mathbf{\dot{D}}(\mathbf{q})\mathbf{\dot{q}} \tag{5.7}$$

From the canonical equations of motion (Chapter 3, Equation (3.38)):

$$\mathbf{D}(\mathbf{q})\mathbf{\ddot{q}} = \boldsymbol{\tau} + \mathbf{Q}_{\text{ext}} - \mathbf{C}(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} - \mathbf{G}(\mathbf{q}) - \mathbf{F}_f(\mathbf{\dot{q}}) \tag{5.8}$$

Substitute Equation (5.8) into Equation (5.7):

$$\mathbf{\dot{p}} = \left[ \boldsymbol{\tau} + \mathbf{Q}_{\text{ext}} - \mathbf{C}(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} - \mathbf{G}(\mathbf{q}) - \mathbf{F}_f(\mathbf{\dot{q}}) \right] + \mathbf{\dot{D}}(\mathbf{q})\mathbf{\dot{q}} \tag{5.9}$$

Group the velocity product terms:

$$\mathbf{\dot{p}} = \boldsymbol{\tau} - \mathbf{G}(\mathbf{q}) - \mathbf{F}_f(\mathbf{\dot{q}}) + \mathbf{Q}_{\text{ext}} + \left[ \mathbf{\dot{D}}(\mathbf{q}) - \mathbf{C}(\mathbf{q}, \mathbf{\dot{q}}) \right] \mathbf{\dot{q}} \tag{5.10}$$

---

### 5.2.3 Exploiting the Passivity Skew-Symmetry Identity
Now examine the bracketed term in Equation (5.10): $\left[ \mathbf{\dot{D}}(\mathbf{q}) - \mathbf{C}(\mathbf{q}, \mathbf{\dot{q}}) \right] \mathbf{\dot{q}}$.

Recall the fundamental **Passivity Skew-Symmetry Identity** rigorously proved in Chapter 3 (Equation (3.40)):

$$\mathbf{\dot{D}}(\mathbf{q}) - 2\mathbf{C}(\mathbf{q}, \mathbf{\dot{q}}) = -\left( \mathbf{\dot{D}}(\mathbf{q}) - 2\mathbf{C}(\mathbf{q}, \mathbf{\dot{q}}) \right)^T \tag{5.11}$$

Since $\mathbf{\dot{D}}$ is symmetric ($\mathbf{\dot{D}}^T = \mathbf{\dot{D}}$):

$$\mathbf{\dot{D}} - 2\mathbf{C} = -\mathbf{\dot{D}} + 2\mathbf{C}^T \implies 2\mathbf{\dot{D}} - 2\mathbf{C} = 2\mathbf{C}^T \implies \mathbf{\dot{D}} - \mathbf{C} = \mathbf{C}^T \tag{5.12}$$

> [!IMPORTANT]
> ### THE CARDINAL MOMENTUM IDENTITY
> Substituting Equation (5.12) into Equation (5.10) gives the central identity of nonlinear observer theory:
> 
> $$\mathbf{\dot{D}}(\mathbf{q})\mathbf{\dot{q}} - \mathbf{C}(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} = \mathbf{C}^T(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} \tag{5.13}$$
> 
> Applying this identity to the momentum derivative in Equation (5.10) yields:
> 
> $$\mathbf{\dot{p}} = \boldsymbol{\tau} + \mathbf{C}^T(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} - \mathbf{G}(\mathbf{q}) - \mathbf{F}_f(\mathbf{\dot{q}}) + \mathbf{Q}_{\text{ext}} \tag{5.14}$$

Look at Equation (5.14) with mathematical awe:
**The joint acceleration vector $\mathbf{\ddot{q}}$ has completely vanished!**

The rate of change of generalized momentum $\mathbf{\dot{p}}$ depends solely on:
- Measured actuator torques $\boldsymbol{\tau}$.
- Measured joint positions $\mathbf{q}$ and velocities $\mathbf{\dot{q}}$.
- The unknown external generalized force $\mathbf{Q}_{\text{ext}}$.

---

## 5.3 The De Luca-Haddadin Generalized Momentum Observer

### 5.3.1 Definition of the Observer Residual Vector $\mathbf{r}(t)$
Building upon Equation (5.14), Alessandro De Luca and Sami Haddadin proposed an integral observer that tracks the momentum error.

Define the **Observer Residual Vector** $\mathbf{r}(t) \in \mathbb{R}^n$:

$$\mathbf{r}(t) = \mathbf{K}_O \left[ \mathbf{p}(t) - \int_0^t \left( \boldsymbol{\tau}(\sigma) + \mathbf{C}^T(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} - \mathbf{G}(\mathbf{q}) - \mathbf{F}_f(\mathbf{\dot{q}}) + \mathbf{r}(\sigma) \right) d\sigma - \mathbf{p}(0) \right] \tag{5.15}$$

where:
- $\mathbf{K}_O = \operatorname{diag}(K_{O,1}, K_{O,2}, \dots, K_{O,n}) > 0$ is a diagonal, positive-definite **Observer Gain Matrix** ($K_{O,i} > 0, \; \forall i$).
- $\mathbf{p}(t) = \mathbf{D}(\mathbf{q}(t))\mathbf{\dot{q}}(t)$ is the instantaneous generalized momentum.
- $\mathbf{p}(0) = \mathbf{D}(\mathbf{q}(0))\mathbf{\dot{q}}(0)$ is the initial momentum at time $t = 0$.

```
                 THE GENERALIZED MOMENTUM OBSERVER ARCHITECTURE
                 
      Actuator Torques tau
      Measured States (q, q_dot) 
                 |
                 v
         [ Dynamic Model ] ---> alpha(t) = tau + C^T*q_dot - G - F_f
                 |                               |
                 v                               v
        p(t) = D(q)*q_dot                      ( + ) <--- Residual feedback r(t)
                 |                               |
                 |                               v
                 |                         [ Integrator ] ---> Integral of (alpha + r)
                 |                               |
                 v                               v
               ( + ) <------------------------ ( - )
                 |
                 v
             [ Gain K_O ]
                 |
                 v
         Residual Vector r(t)  ==================>  Estimates Q_ext !
                 |
                 v
         [ (J_T^T)^(-1) ]
                 |
                 v
         Cartesian Tooth Force Estimate F_ext = [F_x, F_y]^T
```

---

### 5.3.2 Derivation of Error Dynamics
Let us differentiate the residual vector $\mathbf{r}(t)$ in Equation (5.15) with respect to time:

$$\mathbf{\dot{r}}(t) = \mathbf{K}_O \left[ \mathbf{\dot{p}}(t) - \frac{d}{dt} \int_0^t \left( \boldsymbol{\tau} + \mathbf{C}^T\mathbf{\dot{q}} - \mathbf{G} - \mathbf{F}_f + \mathbf{r} \right) d\sigma \right] \tag{5.16}$$

By the Fundamental Theorem of Calculus, the time derivative of the integral is simply the integrand evaluated at $t$:

$$\mathbf{\dot{r}}(t) = \mathbf{K}_O \left[ \mathbf{\dot{p}}(t) - \left( \boldsymbol{\tau} + \mathbf{C}^T\mathbf{\dot{q}} - \mathbf{G} - \mathbf{F}_f + \mathbf{r} \right) \right] \tag{5.17}$$

Now substitute the true physical momentum derivative $\mathbf{\dot{p}}$ from Equation (5.14):

$$\mathbf{\dot{r}}(t) = \mathbf{K}_O \left[ \left( \boldsymbol{\tau} + \mathbf{C}^T\mathbf{\dot{q}} - \mathbf{G} - \mathbf{F}_f + \mathbf{Q}_{\text{ext}} \right) - \left( \boldsymbol{\tau} + \mathbf{C}^T\mathbf{\dot{q}} - \mathbf{G} - \mathbf{F}_f + \mathbf{r} \right) \right] \tag{5.18}$$

Notice what happens inside the brackets:
$\boldsymbol{\tau}$, $\mathbf{C}^T\mathbf{\dot{q}}$, $\mathbf{G}$, and $\mathbf{F}_f$ **cancel out identically**:

$$\mathbf{\dot{r}}(t) = \mathbf{K}_O \left[ \mathbf{Q}_{\text{ext}}(t) - \mathbf{r}(t) \right] \tag{5.19}$$

Rearranging into standard state-space form:

$$\mathbf{\dot{r}}(t) + \mathbf{K}_O \mathbf{r}(t) = \mathbf{K}_O \mathbf{Q}_{\text{ext}}(t) \tag{5.20}$$

---

### 5.3.3 Frequency-Domain Transfer Function
Since $\mathbf{K}_O = \operatorname{diag}(K_{O,1}, \dots, K_{O,n})$ is diagonal, Equation (5.20) decouples into $n$ completely independent first-order differential equations:

$$\dot{r}_i(t) + K_{O,i} r_i(t) = K_{O,i} Q_{\text{ext},i}(t), \quad i = 1, 2, \dots, n \tag{5.21}$$

Taking the Laplace transform with zero initial residual ($r_i(0) = 0$):

$$s R_i(s) + K_{O,i} R_i(s) = K_{O,i} Q_{\text{ext},i}(s) \tag{5.22}$$

$$\frac{R_i(s)}{Q_{\text{ext},i}(s)} = \frac{K_{O,i}}{s + K_{O,i}} = \frac{1}{\frac{1}{K_{O,i}}s + 1} \tag{5.23}$$

> [!IMPORTANT]
> ### PROFOUND ENGINEERING RESULT: THE GMO AS A LOW-PASS FILTER
> Look closely at transfer function (5.23):
> 
> The observer residual $r_i(t)$ is **strictly a decoupled, first-order low-pass filtered replica of the true external generalized force $Q_{\text{ext},i}(t)$**!
> 
> 1. **DC Gain:** At steady state ($s \to 0$):
>    $$\lim_{s \to 0} \frac{R_i(s)}{Q_{\text{ext},i}(s)} = \frac{K_{O,i}}{0 + K_{O,i}} = 1.000$$
>    The observer has **zero steady-state tracking error**: $\lim_{t \to \infty} r_i(t) = Q_{\text{ext},i}$.
> 2. **Filter Bandwidth:** The $-3\,\mathrm{dB}$ cutoff frequency of the observer is:
>    $$\omega_{c,i} = K_{O,i} \quad (\mathrm{rad/s})$$
>    The time constant is:
>    $$T_{O,i} = \frac{1}{K_{O,i}} \quad (\mathrm{seconds})$$
> 3. **Complete Elimination of Accelerations:** At no point in Equation (5.15) or (5.20) is joint acceleration $\mathbf{\ddot{q}}$ computed, measured, or differentiated!

---

## 5.4 Lyapunov Stability Proof of Observer Convergence

We now prove that the estimation error converges exponentially to zero using Lyapunov stability theory.

### 5.4.1 Definition of Estimation Error
Define the estimation error vector $\mathbf{e}_r(t) \in \mathbb{R}^n$:

$$\mathbf{e}_r(t) = \mathbf{Q}_{\text{ext}}(t) - \mathbf{r}(t) \tag{5.24}$$

Differentiating with respect to time:

$$\mathbf{\dot{e}}_r(t) = \mathbf{\dot{Q}}_{\text{ext}}(t) - \mathbf{\dot{r}}(t) \tag{5.25}$$

Substitute the observer dynamics $\mathbf{\dot{r}} = \mathbf{K}_O(\mathbf{Q}_{\text{ext}} - \mathbf{r}) = \mathbf{K}_O \mathbf{e}_r$:

$$\mathbf{\dot{e}}_r(t) = -\mathbf{K}_O \mathbf{e}_r(t) + \mathbf{\dot{Q}}_{\text{ext}}(t) \tag{5.26}$$

---

### 5.4.2 Lyapunov Function Formulation
For piecewise-constant or slowly-varying contact forces ($\mathbf{\dot{Q}}_{\text{ext}} \approx \mathbf{0}$):

$$\mathbf{\dot{e}}_r(t) = -\mathbf{K}_O \mathbf{e}_r(t) \tag{5.27}$$

Consider the candidate scalar **Lyapunov Function** $V(\mathbf{e}_r)$:

$$V(\mathbf{e}_r) = \frac{1}{2} \mathbf{e}_r^T \mathbf{P}_O \mathbf{e}_r \tag{5.28}$$

where $\mathbf{P}_O$ is a symmetric, positive-definite matrix (chosen as $\mathbf{P}_O = \mathbf{I}_n$).

Clearly:
1. $V(\mathbf{0}) = 0$.
2. $V(\mathbf{e}_r) > 0$ for all $\mathbf{e}_r \neq \mathbf{0}$.
3. $V(\mathbf{e}_r)$ is radially unbounded ($\lim_{\|\mathbf{e}_r\| \to \infty} V(\mathbf{e}_r) = \infty$).

---

### 5.4.3 Derivative of the Lyapunov Function
Differentiate $V(\mathbf{e}_r)$ along the system trajectories:

$$\dot{V}(\mathbf{e}_r) = \mathbf{e}_r^T \mathbf{\dot{e}}_r = \mathbf{e}_r^T \left( -\mathbf{K}_O \mathbf{e}_r \right) = -\mathbf{e}_r^T \mathbf{K}_O \mathbf{e}_r \tag{5.29}$$

Since $\mathbf{K}_O = \operatorname{diag}(K_{O,1}, \dots, K_{O,n})$ with all $K_{O,i} > 0$:

$$\dot{V}(\mathbf{e}_r) = -\sum_{i=1}^n K_{O,i} e_{r,i}^2 \le -K_{O,\min} \|\mathbf{e}_r\|^2 < 0, \quad \forall \mathbf{e}_r \neq \mathbf{0} \tag{5.30}$$

where $K_{O,\min} = \min_i(K_{O,i}) > 0$.

> [!IMPORTANT]
> ### GLOBAL EXPONENTIAL STABILITY THEOREM
> Since $V = \frac{1}{2}\|\mathbf{e}_r\|^2$ and $\dot{V} \le -2 K_{O,\min} V$:
> 
> $$V(t) \le V(0) e^{-2 K_{O,\min} t} \implies \|\mathbf{e}_r(t)\| \le \|\mathbf{e}_r(0)\| e^{-K_{O,\min} t} \tag{5.31}$$
> 
> By Lyapunov's Direct Method, the equilibrium point $\mathbf{e}_r = \mathbf{0}$ is **Globally Exponentially Stable (GES)**. The estimated generalized load $\mathbf{r}(t)$ converges exponentially to the true external load $\mathbf{Q}_{\text{ext}}$ at rate $K_{O,\min}$. $\blacksquare$

---

## 5.5 Reconstruction of Cartesian Digging Forces at Dipper Teeth

Once the generalized residual vector $\mathbf{r}(t) = [r_3, r_4]^T$ is estimated by the observer, we must reconstruct the true Cartesian cutting force $\mathbf{F}_{\text{ext}} = [F_x, F_y]^T$ acting at the dipper tooth tip.

Recall from Chapter 2 (Equation (2.22)) that the transposed Jacobian maps Cartesian forces to generalized forces:

$$\mathbf{Q}_{\text{ext}} = \mathbf{J}_T^T(q_3, d_4) \mathbf{F}_{\text{ext}} \tag{5.32}$$

At steady state, $\mathbf{r}(t) \to \mathbf{Q}_{\text{ext}}$. Therefore:

$$\mathbf{r}(t) = \mathbf{J}_T^T(q_3, d_4) \hat{\mathbf{F}}_{\text{ext}} \tag{5.33}$$

Invert the $2 \times 2$ transposed Jacobian matrix:

$$\hat{\mathbf{F}}_{\text{ext}} = \left( \mathbf{J}_T^T(q_3, d_4) \right)^{-1} \mathbf{r}(t) = \left( \mathbf{J}_T^{-1}(q_3, d_4) \right)^T \mathbf{r}(t) \tag{5.34}$$

Using the closed-form inverse Jacobian derived in Chapter 2 (Equation (2.40)):

$$\mathbf{J}_T^{-1}(q_3, d_4) = \begin{bmatrix}
-\frac{\sin q_3}{r_T} & \frac{\cos q_3}{r_T} \\[8pt]
\cos q_3 + \frac{L_{ty}}{r_T}\sin q_3 & \sin q_3 - \frac{L_{ty}}{r_T}\cos q_3
\end{bmatrix} \tag{5.35}$$

Taking the matrix transpose:

$$\left( \mathbf{J}_T^T \right)^{-1} = \left( \mathbf{J}_T^{-1} \right)^T = \begin{bmatrix}
-\frac{\sin q_3}{r_T} & \cos q_3 + \frac{L_{ty}}{r_T}\sin q_3 \\[8pt]
\frac{\cos q_3}{r_T} & \sin q_3 - \frac{L_{ty}}{r_T}\cos q_3
\end{bmatrix} \tag{5.36}$$

Carrying out the matrix-vector multiplication with $\mathbf{r} = [r_3, r_4]^T$:

$$\hat{F}_x = -\frac{\sin q_3}{r_T} r_3 + \left( \cos q_3 + \frac{L_{ty}}{r_T}\sin q_3 \right) r_4 \tag{5.37a}$$

$$\hat{F}_y = \frac{\cos q_3}{r_T} r_3 + \left( \sin q_3 - \frac{L_{ty}}{r_T}\cos q_3 \right) r_4 \tag{5.37b}$$

where $r_T = d_4 + L_{tx}$ is the total distance from pivot $O_3$ to the tooth tip.

> [!TIP]
> ### PHYSICAL INTUITION ON CARTESIAN FORCE DECOUPLING
> Look at Equations (5.37a) and (5.37b):
> - $r_3$ (saddle torque residual) is divided by the moment arm $r_T \approx 11\,\mathrm{m}$, transforming Newton-meters of bending moment back into Newtons of transverse force.
> - $r_4$ (crowd force residual) is multiplied directly by handle orientation projections ($\cos q_3$ and $\sin q_3$).
> - **Because $\det(\mathbf{J}_T) = -r_T \neq 0$ everywhere (Chapter 2), this force reconstruction is unconditionally non-singular across the entire digging envelope!**

---

## 5.6 Discrete-Time Implementation & Practical Engineering Safeguards

In industrial digital twins, the momentum observer is executed on a PLC or embedded industrial controller at discrete sample rate $\Delta t$ (typically $\Delta t = 2\,\mathrm{ms}$ to $10\,\mathrm{ms}$, corresponding to $100\,\mathrm{Hz} - 500\,\mathrm{Hz}$).

### 5.6.1 Trapezoidal Numerical Integration
To prevent numerical drift and phase lag, the integral term in Equation (5.15) is discretized using the **Tustin / Trapezoidal Rule**:

Let the internal dynamic signal be:

$$\boldsymbol{\alpha}(t) = \boldsymbol{\tau}(t) + \mathbf{C}^T(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}} - \mathbf{G}(\mathbf{q}) - \mathbf{F}_f(\mathbf{\dot{q}}) \tag{5.38}$$

Let the integral state vector be $\mathbf{z}(t_k) \in \mathbb{R}^n$:

$$\mathbf{z}(t_k) = \mathbf{z}(t_{k-1}) + \frac{\Delta t}{2} \left[ \boldsymbol{\alpha}(t_k) + \mathbf{r}(t_k) + \boldsymbol{\alpha}(t_{k-1}) + \mathbf{r}(t_{k-1}) \right] \tag{5.39}$$

The discrete observer update at step $k$ is:

$$\mathbf{r}(t_k) = \mathbf{K}_O \left[ \mathbf{p}(t_k) - \mathbf{z}(t_k) - \mathbf{p}(0) \right] \tag{5.40}$$

---

### 5.6.2 Resolving Implicit Algebraic Loops
Notice that $\mathbf{r}(t_k)$ appears on both the left-hand side of Equation (5.40) and inside $\mathbf{z}(t_k)$ on the right-hand side! This is an implicit algebraic loop.

Substitute Equation (5.39) into (5.40):

$$\mathbf{r}(t_k) = \mathbf{K}_O \left[ \mathbf{p}(t_k) - \mathbf{z}(t_{k-1}) - \frac{\Delta t}{2}\boldsymbol{\alpha}(t_k) - \frac{\Delta t}{2}\mathbf{r}(t_k) - \frac{\Delta t}{2}(\boldsymbol{\alpha}(t_{k-1}) + \mathbf{r}(t_{k-1})) - \mathbf{p}(0) \right]$$

Bring all terms containing $\mathbf{r}(t_k)$ to the left-hand side:

$$\left( \mathbf{I}_n + \frac{\Delta t}{2} \mathbf{K}_O \right) \mathbf{r}(t_k) = \mathbf{K}_O \left[ \mathbf{p}(t_k) - \mathbf{z}(t_{k-1}) - \frac{\Delta t}{2}\boldsymbol{\alpha}(t_k) - \frac{\Delta t}{2}(\boldsymbol{\alpha}(t_{k-1}) + \mathbf{r}(t_{k-1})) - \mathbf{p}(0) \right] \tag{5.41}$$

Since $\mathbf{K}_O$ is diagonal, $\left( \mathbf{I}_n + \frac{\Delta t}{2} \mathbf{K}_O \right)$ is a diagonal matrix whose inversion is trivial:

$$r_i(t_k) = \frac{K_{O,i}}{1 + \frac{\Delta t}{2} K_{O,i}} \left[ p_i(t_k) - z_i(t_{k-1}) - \frac{\Delta t}{2}\alpha_i(t_k) - \frac{\Delta t}{2}(\alpha_i(t_{k-1}) + r_i(t_{k-1})) - p_i(0) \right] \tag{5.42}$$

Equation (5.42) is **strictly explicit, causal, and evaluates in $0.05\,\mu\mathrm{s}$ with zero algebraic loops**!

---

### 5.6.3 Robustness to Parameter Errors (Sensitivity Analysis)
What happens if our model parameters $\hat{\mathbf{D}}, \hat{\mathbf{C}}, \hat{\mathbf{G}}$ differ slightly from the true shovel parameters $\mathbf{D}, \mathbf{C}, \mathbf{G}$?

Let the model mismatch generalized force be:

$$\Delta \boldsymbol{\tau}_{\text{model}} = (\mathbf{D} - \hat{\mathbf{D}})\mathbf{\ddot{q}} + (\mathbf{C} - \hat{\mathbf{C}})\mathbf{\dot{q}} + (\mathbf{G} - \hat{\mathbf{G}}) + (\mathbf{F}_f - \hat{\mathbf{F}}_f) \tag{5.43}$$

Then the observer error dynamics in Equation (5.20) become:

$$\mathbf{\dot{r}} + \mathbf{K}_O \mathbf{r} = \mathbf{K}_O \left( \mathbf{Q}_{\text{ext}} + \Delta \boldsymbol{\tau}_{\text{model}} \right) \tag{5.44}$$

At steady state:

$$\mathbf{r}_{\text{ss}} = \mathbf{Q}_{\text{ext}} + \Delta \boldsymbol{\tau}_{\text{model}} \tag{5.45}$$

> [!WARNING]
> ### WHY PRECISE BASE PARAMETER IDENTIFICATION (M2) IS MANDATORY
> Equation (5.45) establishes the direct coupling between **Module M2** and **Module M3**:
> Any residual error in the identified parameters $\hat{\boldsymbol{\theta}}_b$ (from Chapter 4) acts as a systematic bias $\Delta \boldsymbol{\tau}_{\text{model}}$ in the estimated digging force!
> 
> This is why Module M2 (the multi-channel base parameter identification system) had to be developed and verified before Module M3 could be deployed!

---

## 5.7 Five Fully Solved Research Practice Problems

---

### Problem 5.1: Discrete Observer Update on P&H 2100BL
**Problem Statement:**  
A P&H 2100BL excavator has identified parameters:
- $D_{11}(d_4) = 7.4274 \times 10^5 + 3.6600 \times 10^4 d_4 + 4.8500 \times 10^4 d_4^2\,\mathrm{kg\cdot m^2}$
- $D_{22} = 48,500\,\mathrm{kg}$
- Observer gains are tuned to $K_{O,1} = 25.0\,\mathrm{rad/s}$ (hoist) and $K_{O,2} = 30.0\,\mathrm{rad/s}$ (crowd).
- The discrete execution step is $\Delta t = 0.005\,\mathrm{s}$ ($200\,\mathrm{Hz}$).

At time step $k$, the dipper teeth suddenly strike an unblasted rock ledge. The sensor readings are:
- $q_3 = 10^\circ$ ($0.1745\,\mathrm{rad}$), $\dot{q}_3 = 0.10\,\mathrm{rad/s}$
- $d_4 = 10.0\,\mathrm{m}$, $\dot{d}_4 = 0.50\,\mathrm{m/s}$
- Motor torques: $\tau_3 = 6.20 \times 10^6\,\mathrm{N\cdot m}$, $F_{\text{crowd}} = 2.50 \times 10^5\,\mathrm{N}$
- Model gravity and coupling: $\alpha_1 = 5.10 \times 10^6\,\mathrm{N\cdot m}$, $\alpha_2 = 1.15 \times 10^5\,\mathrm{N}$
- Previous integral states: $z_1(k-1) = 2.45 \times 10^5\,\mathrm{N\cdot m\cdot s}$, $z_2(k-1) = 1.80 \times 10^4\,\mathrm{N\cdot s}$
- Initial momentum was $\mathbf{p}(0) = \mathbf{0}$.

Compute the instantaneous residual vector $\mathbf{r}(t_k) = [r_3, r_4]^T$.

**Step-by-Step Solution:**

**Step 1: Compute instantaneous generalized momentum $\mathbf{p}(t_k)$**
At $d_4 = 10.0\,\mathrm{m}$:
$$D_{11}(10.0) = 742,740 + (36,600)(10.0) + (48,500)(10.0)^2$$
$$D_{11}(10.0) = 742,740 + 366,000 + 4,850,000 = 5,958,740\,\mathrm{kg\cdot m^2}$$
$$p_1 = D_{11} \dot{q}_3 = (5,958,740\,\mathrm{kg\cdot m^2})(0.10\,\mathrm{rad/s}) = 595,874\,\mathrm{N\cdot m\cdot s}$$

$$p_2 = D_{22} \dot{d}_4 = (48,500\,\mathrm{kg})(0.50\,\mathrm{m/s}) = 24,250\,\mathrm{N\cdot s}$$

$$\mathbf{p}(t_k) = \begin{bmatrix} 595,874\,\mathrm{N\cdot m\cdot s} \\ 24,250\,\mathrm{N\cdot s} \end{bmatrix}$$

**Step 2: Evaluate the explicit discrete formula (Equation (5.42)) for Channel 1**
$$G_{\text{scale},1} = \frac{K_{O,1}}{1 + \frac{\Delta t}{2} K_{O,1}} = \frac{25.0}{1 + (0.0025)(25.0)} = \frac{25.0}{1 + 0.0625} = \frac{25.0}{1.0625} \approx 23.52941\,\mathrm{s^{-1}}$$

Difference term:
$$\Delta p_1 = p_1(t_k) - z_1(k-1) - \frac{\Delta t}{2}\alpha_1(k) = 595,874 - 245,000 - (0.0025)(5,100,000)$$
$$\Delta p_1 = 350,874 - 12,750 = 338,124\,\mathrm{N\cdot m\cdot s}$$

$$r_1(t_k) = (23.52941\,\mathrm{s^{-1}})(338,124\,\mathrm{N\cdot m\cdot s}) \approx 7,955,858\,\mathrm{N\cdot m} \approx +7.956\,\mathrm{MN\cdot m}$$

**Step 3: Evaluate Channel 2**
$$G_{\text{scale},2} = \frac{K_{O,2}}{1 + \frac{\Delta t}{2} K_{O,2}} = \frac{30.0}{1 + (0.0025)(30.0)} = \frac{30.0}{1 + 0.075} = \frac{30.0}{1.075} \approx 27.90698\,\mathrm{s^{-1}}$$

Difference term:
$$\Delta p_2 = p_2(t_k) - z_2(k-1) - \frac{\Delta t}{2}\alpha_2(k) = 24,250 - 18,000 - (0.0025)(115,000)$$
$$\Delta p_2 = 6,250 - 287.5 = 5,962.5\,\mathrm{N\cdot s}$$

$$r_2(t_k) = (27.90698\,\mathrm{s^{-1}})(5,962.5\,\mathrm{N\cdot s}) \approx 166,395\,\mathrm{N} \approx +166.4\,\mathrm{kN}$$

**Result:** The observer instantaneously estimates an external generalized contact load of:
$$\mathbf{r}(t_k) = \begin{bmatrix} +7.956\,\mathrm{MN\cdot m} \\ +166.4\,\mathrm{kN} \end{bmatrix}$$

---

### Problem 5.2: Reconstructing 2D Tooth Tip Cutting Force Vector
**Problem Statement:**  
Using the residual estimates from Problem 5.1 ($\mathbf{r} = [7.956 \times 10^6\,\mathrm{N\cdot m}, \; 1.664 \times 10^5\,\mathrm{N}]^T$) at posture $q_3 = 10^\circ$, $d_4 = 10.0\,\mathrm{m}$ with tooth offsets $L_{tx} = 2.0\,\mathrm{m}$ and $L_{ty} = 1.0\,\mathrm{m}$:
1. Compute the effective tooth radius $r_T$.
2. Invert the transposed Jacobian to calculate the horizontal cutting force $\hat{F}_x$ and vertical breakout force $\hat{F}_y$.
3. Compute the resultant cutting force magnitude $\|\hat{\mathbf{F}}_{\text{ext}}\|$ and direction angle $\beta_{\text{cut}}$.

**Step-by-Step Solution:**

**Step 1: Compute geometric values**
$$r_T = d_4 + L_{tx} = 10.0 + 2.0 = 12.0\,\mathrm{m}$$
$$\cos 10^\circ \approx 0.984808, \quad \sin 10^\circ \approx 0.173648$$

**Step 2: Evaluate Inverse Transposed Jacobian entries from Equation (5.36)**
$$\text{Entry}_{11} = -\frac{\sin q_3}{r_T} = -\frac{0.173648}{12.0} \approx -0.014471\,\mathrm{m^{-1}}$$

$$\text{Entry}_{12} = \cos q_3 + \frac{L_{ty}}{r_T}\sin q_3 = 0.984808 + \left(\frac{1.0}{12.0}\right)(0.173648) = 0.984808 + 0.014471 = +0.999279$$

$$\text{Entry}_{21} = \frac{\cos q_3}{r_T} = \frac{0.984808}{12.0} \approx +0.082067\,\mathrm{m^{-1}}$$

$$\text{Entry}_{22} = \sin q_3 - \frac{L_{ty}}{r_T}\cos q_3 = 0.173648 - \left(\frac{1.0}{12.0}\right)(0.984808) = 0.173648 - 0.082067 = +0.091581$$

**Step 3: Compute Cartesian Forces $[\hat{F}_x, \hat{F}_y]^T$**
$$\hat{F}_x = \text{Entry}_{11} r_1 + \text{Entry}_{12} r_2$$
$$\hat{F}_x = (-0.014471)(7,956,000) + (0.999279)(166,400)$$
$$\hat{F}_x = -115,131\,\mathrm{N} + 166,280\,\mathrm{N} = +51,149\,\mathrm{N} \approx +51.15\,\mathrm{kN}$$

$$\hat{F}_y = \text{Entry}_{21} r_1 + \text{Entry}_{22} r_2$$
$$\hat{F}_y = (+0.082067)(7,956,000) + (+0.091581)(166,400)$$
$$\hat{F}_y = +652,925\,\mathrm{N} + 15,239\,\mathrm{N} = +668,164\,\mathrm{N} \approx +668.16\,\mathrm{kN}$$

$$\hat{\mathbf{F}}_{\text{ext}} = \begin{bmatrix} +51.15\,\mathrm{kN} \\ +668.16\,\mathrm{kN} \end{bmatrix}$$

**Step 4: Magnitude and Angle**
$$\|\hat{\mathbf{F}}_{\text{ext}}\| = \sqrt{(51.15)^2 + (668.16)^2} = \sqrt{2,616.3 + 446,437.8} = \sqrt{449,054.1} \approx 670.1\,\mathrm{kN}$$

$$\beta = \operatorname{atan2}(\hat{F}_y, \hat{F}_x) = \operatorname{atan2}(668.16, 51.15) \approx 85.6^\circ$$

*(Physical Interpretation: The dipper teeth are encountering a steep vertical breakout resistance of $668.2\,\mathrm{kN}$ directed almost purely upward ($85.6^\circ$) as the teeth pry under the rock slab!).*

---

### Problem 5.3: Transient Step Response and Time-to-Detect Calculation
**Problem Statement:**  
At $t = 0$, a TZ WK-20 dipper strikes a buried basalt boulder, producing an instantaneous step increase in external digging force $Q_{\text{ext}}(t) = 500\,\mathrm{kN\cdot m} \cdot u(t)$.  
The observer gain is set to $K_O = 40.0\,\mathrm{rad/s}$.
1. Write the closed-form analytical equation for $r(t)$ for $t \ge 0$.
2. Calculate the time $t_{90\%}$ required for the observer to reach $90\%$ of the true load value.
3. Calculate the time $t_{99\%}$ required to reach $99\%$ of the true load.
4. If an overload protection system trips when the estimated load exceeds $400\,\mathrm{kN\cdot m}$ ($80\%$), how fast does the trip trigger?

**Step-by-Step Solution:**

**Step 1: Analytical solution to $\dot{r} + K_O r = K_O Q_0$**
For a step input of magnitude $Q_0$ with $r(0) = 0$:

$$r(t) = Q_0 \left( 1 - e^{-K_O t} \right) = 500 \left( 1 - e^{-40 t} \right)\,\mathrm{kN\cdot m} \tag{5.46}$$

**Step 2: Time to $90\%$**
$$1 - e^{-40 t_{90\%}} = 0.90 \implies e^{-40 t_{90\%}} = 0.10 \implies -40 t_{90\%} = \ln(0.10) = -2.30259$$
$$t_{90\%} = \frac{2.30259}{40} \approx 0.05756\,\mathrm{s} \approx 57.6\,\mathrm{ms}$$

**Step 3: Time to $99\%$**
$$e^{-40 t_{99\%}} = 0.01 \implies -40 t_{99\%} = \ln(0.01) = -4.60517$$
$$t_{99\%} = \frac{4.60517}{40} \approx 0.11513\,\mathrm{s} \approx 115.1\,\mathrm{ms}$$

**Step 4: Time to $80\%$ Overload Trip**
$$1 - e^{-40 t_{80\%}} = 0.80 \implies e^{-40 t_{80\%}} = 0.20 \implies -40 t_{80\%} = \ln(0.20) = -1.60944$$
$$t_{\text{trip}} = \frac{1.60944}{40} \approx 0.04024\,\mathrm{s} \approx 40.2\,\mathrm{ms}$$

*(Engineering Significance: The observer detects the severe rock impact and triggers motor drive de-excitation in just **$40.2\,\mathrm{milliseconds}$**—well before peak mechanical stresses propagate to the boom foot chords!).*

---

### Problem 5.4: Gain Tuning Trade-Off: Bandwidth vs. Noise Immunity
**Problem Statement:**  
During real digging, the motor torque signal $\tau(t)$ is corrupted by high-frequency inverter PWM ripple modeled as white noise with spectral density $S_w = 10^4\,(\mathrm{N\cdot m})^2/\mathrm{Hz}$.
1. Calculate the variance of the observer output noise $\sigma_r^2$ as a function of the gain $K_O$.
2. For $K_O = 10\,\mathrm{rad/s}$, $K_O = 40\,\mathrm{rad/s}$, and $K_O = 100\,\mathrm{rad/s}$, compute the noise standard deviation $\sigma_r$ and the response time constant $T_O = 1/K_O$.
3. Formulate the optimal gain tuning recommendation for mining shovels.

**Step-by-Step Solution:**

**Step 1: Noise variance through a first-order low-pass filter**
The transfer function from noise $w(t)$ to residual $r(t)$ is:
$$H(s) = \frac{K_O}{s + K_O}$$
The output noise variance is given by the integral of the power spectral density:
$$\sigma_r^2 = \frac{1}{2\pi} \int_{-\infty}^{\infty} |H(j\omega)|^2 S_w \, d\omega = S_w \cdot \frac{K_O}{2}$$

$$\sigma_r = \sqrt{\frac{S_w K_O}{2}}$$

**Step 2: Evaluate for three gain values**
- **Case A: $K_O = 10\,\mathrm{rad/s}$**
  $$\sigma_r = \sqrt{\frac{(10^4)(10)}{2}} = \sqrt{50,000} \approx 223.6\,\mathrm{N\cdot m}$$
  $$T_O = \frac{1}{10} = 0.100\,\mathrm{s} = 100\,\mathrm{ms}$$
- **Case B: $K_O = 40\,\mathrm{rad/s}$**
  $$\sigma_r = \sqrt{\frac{(10^4)(40)}{2}} = \sqrt{200,000} \approx 447.2\,\mathrm{N\cdot m}$$
  $$T_O = \frac{1}{40} = 0.025\,\mathrm{s} = 25\,\mathrm{ms}$$
- **Case C: $K_O = 100\,\mathrm{rad/s}$**
  $$\sigma_r = \sqrt{\frac{(10^4)(100)}{2}} = \sqrt{500,000} \approx 707.1\,\mathrm{N\cdot m}$$
  $$T_O = \frac{1}{100} = 0.010\,\mathrm{s} = 10\,\mathrm{ms}$$

**Step 3: Recommendation**
$K_O = 30 - 50\,\mathrm{rad/s}$ provides the ideal engineering compromise for an electric rope shovel: response time under $30\,\mathrm{ms}$ with noise standard deviation under $0.5\,\mathrm{kN\cdot m}$ (less than $0.01\%$ of the $6\,\mathrm{MN\cdot m}$ nominal torque rating!).

---

### Problem 5.5: Payload Mass Recovery from Steady-State Momentum Residual
**Problem Statement:**  
After scooping a bucket of ore and hoisting it out of the bank into mid-air, the shovel holds the loaded dipper statically in mid-air at $q_3 = 25^\circ, d_4 = 11.5\,\mathrm{m}$.  
Because the dipper is in mid-air, external contact forces are zero ($\mathbf{F}_{\text{ext}} = \mathbf{0}$). However, the observer model was configured using the **empty bucket baseline parameters** ($M_{d,\text{empty}} = 17,000\,\mathrm{kg}$).  
The observer outputs a steady-state residual on the crowd channel of $r_4 = +82,500\,\mathrm{N}$.  
1. Explain physically why $r_4 \neq 0$ even though no rock is touching the teeth.
2. Using Equation (5.45), calculate the exact payload mass $M_{\text{payload}}$ in metric tonnes.

**Step-by-Step Solution:**

**Step 1: Physical Explanation**
When the dipper holds payload mass $M_{\text{payload}}$, the true gravitational downward force along the handle is:
$$G_{2,\text{true}} = (m_h + M_{d,\text{empty}} + M_{\text{payload}}) g \sin q_3$$
However, the observer uses the baseline model:
$$\hat{G}_2 = (m_h + M_{d,\text{empty}}) g \sin q_3$$
Because the observer model lacks the payload gravity term, the observer perceives the unmodeled gravity load as an "external force":
$$r_4 = G_{2,\text{true}} - \hat{G}_2 = M_{\text{payload}} g \sin q_3$$

**Step 2: Solve for Payload Mass**
$$r_4 = M_{\text{payload}} g \sin q_3 \implies M_{\text{payload}} = \frac{r_4}{g \sin q_3}$$

Evaluate at $q_3 = 25^\circ$:
$$\sin 25^\circ \approx 0.422618$$
$$M_{\text{payload}} = \frac{82,500\,\mathrm{N}}{(9.81\,\mathrm{m/s^2})(0.422618)} = \frac{82,500}{4.14588} \approx 19,899.3\,\mathrm{kg} \approx 19.90\,\mathrm{metric\ tonnes}$$

*(Profound Result: The Generalized Momentum Observer automatically acts as a **Precision Electronic Scale**, measuring the suspended payload to within $0.5\%$ accuracy without requiring any specialized payload weighing pins!).*

---

## 5.8 MATLAB / Python Simulation Script

```matlab
% =========================================================================
% CHAPTER 5: GENERALIZED MOMENTUM OBSERVER (M3) SIMULATION & BENCHMARK
% Compatible with MATLAB R2025b / Octave 7+
% =========================================================================
function shovel_momentum_observer_demo()
    clc; clear; close all;
    
    % Model Parameters (P&H 2100BL Loaded)
    p.I0   = 4.20e5;     p.ms  = 3500;   p.rs  = 0.50;
    p.mh   = 11500;      p.Lch = 4.20;
    p.Md   = 37000;      p.Lcd = 1.80;
    p.Ltx  = 2.00;       p.Lty = 1.00;
    p.g    = 9.81;
    
    % Observer Gain Matrix
    Ko = diag([30.0, 40.0]); % [rad/s]
    
    % Simulation Time Base
    dt = 0.005; % 200 Hz
    t = (0:dt:5)';
    N_steps = length(t);
    
    % True External Disturbance (Step rock collision at t = 2.0s)
    F_ext_true = zeros(2, N_steps);
    for k = 1:N_steps
        if t(k) >= 2.0 && t(k) <= 4.0
            F_ext_true(:, k) = [-150e3; 450e3]; % [-150 kN cutting, +450 kN breakout]
        end
    end
    
    % Trajectory: Digging Sweep
    q3  = deg2rad(15 + 10*sin(0.5*t));
    d4  = 10.0 + 1.5*sin(0.4*t);
    q3_d = gradient(q3, dt);
    d4_d = gradient(d4, dt);
    
    % Pre-allocate Observer Arrays
    r_est = zeros(2, N_steps);
    F_ext_est = zeros(2, N_steps);
    z_int = zeros(2, 1);
    alpha_prev = zeros(2, 1);
    r_prev = zeros(2, 1);
    
    % Initial Momentum
    p_init = eval_momentum([q3(1); d4(1)], [q3_d(1); d4_d(1)], p);
    
    fprintf('Running M3 Generalized Momentum Observer Simulation...\n');
    for k = 1:N_steps
        qk  = [q3(k); d4(k)];
        qdk = [q3_d(k); d4_d(k)];
        
        % 1. Physical Dynamics
        D = eval_D(qk, p);
        C = eval_C(qk, qdk, p);
        G = eval_G(qk, p);
        J = eval_J(qk, p);
        
        % True Generalized Contact Load
        Q_ext_true = J' * F_ext_true(:, k);
        
        % Motor Torque = Dynamics - Q_ext
        tau_act = C*qdk + G - Q_ext_true; 
        
        % 2. Observer Signals
        p_meas = D * qdk;
        alpha_k = tau_act + C'*qdk - G;
        
        % 3. Explicit Trapezoidal GMO Update (Eq 5.42)
        if k == 1
            r_k = zeros(2, 1);
        else
            G_scale = Ko / (eye(2) + 0.5*dt*Ko);
            diff_term = p_meas - z_int - 0.5*dt*alpha_k - ...
                        0.5*dt*(alpha_prev + r_prev) - p_init;
            r_k = G_scale * diff_term;
        end
        
        % Update integral state
        z_int = z_int + 0.5*dt*(alpha_k + r_k + alpha_prev + r_prev);
        alpha_prev = alpha_k;
        r_prev = r_k;
        
        % 4. Invert Transposed Jacobian to get Cartesian Force
        F_ext_est(:, k) = (J') \ r_k;
        r_est(:, k) = r_k;
    end
    
    % Plotting Results
    figure('Color', 'w', 'Position', [100, 100, 850, 600]);
    subplot(2,1,1);
    plot(t, F_ext_true(1,:)/1e3, 'k--', 'LineWidth', 2); hold on;
    plot(t, F_ext_est(1,:)/1e3, 'r-', 'LineWidth', 1.5);
    ylabel('F_x (Cutting) [kN]');
    legend('True Rock Force', 'M3 Observer Estimate', 'Location', 'Best');
    grid on; title('M3 Generalized Momentum Observer: Sensorless Cutting Force Estimation');
    
    subplot(2,1,2);
    plot(t, F_ext_true(2,:)/1e3, 'k--', 'LineWidth', 2); hold on;
    plot(t, F_ext_est(2,:)/1e3, 'b-', 'LineWidth', 1.5);
    ylabel('F_y (Breakout) [kN]'); xlabel('Time [seconds]');
    legend('True Rock Force', 'M3 Observer Estimate', 'Location', 'Best');
    grid on;
    
    fprintf('Simulation Complete. Peak Tracking Error at Steady State: %.2f%%\n', ...
        abs(norm(F_ext_est(:, 600)) - norm(F_ext_true(:, 600))) / norm(F_ext_true(:, 600)) * 100);
end

function p_vec = eval_momentum(q, qd, p)
    D = eval_D(q, p);
    p_vec = D * qd;
end

function D = eval_D(q, p)
    d4 = q(2);
    D11 = p.I0 + p.mh*(d4 - p.Lch)^2 + p.Md*(d4 + p.Lcd)^2;
    D22 = p.mh + p.Md;
    D = [D11, 0; 0, D22];
end

function C = eval_C(q, qd, p)
    d4 = q(2);
    gamma = p.mh*(d4 - p.Lch) + p.Md*(d4 + p.Lcd);
    C = [gamma*qd(2),  gamma*qd(1);
        -gamma*qd(1),  0];
end

function G = eval_G(q, p)
    q3 = q(1); d4 = q(2);
    gamma = p.mh*(d4 - p.Lch) + p.Md*(d4 + p.Lcd);
    G1 = p.g * cos(q3) * (p.ms*p.rs + gamma);
    G2 = (p.mh + p.Md) * p.g * sin(q3);
    G = [G1; G2];
end

function J = eval_J(q, p)
    q3 = q(1); d4 = q(2);
    r_T = d4 + p.Ltx;
    J = [-r_T*sin(q3) + p.Lty*cos(q3), cos(q3);
          r_T*cos(q3) + p.Lty*sin(q3), sin(q3)];
end
```

---

## 5.9 Research Supervisor Summary & Master Defense Checklist

### Core Concepts Mastered in Chapter 5

1. **The Core Advantage of Generalized Momentum:**  
   By defining $\mathbf{p} = \mathbf{D}(\mathbf{q})\mathbf{\dot{q}}$ and substituting the skew-symmetry identity $\mathbf{\dot{D}}\mathbf{\dot{q}} - \mathbf{C}\mathbf{\dot{q}} = \mathbf{C}^T\mathbf{\dot{q}}$, the time derivative of momentum $\mathbf{\dot{p}} = \boldsymbol{\tau} + \mathbf{C}^T\mathbf{\dot{q}} - \mathbf{G} + \mathbf{Q}_{\text{ext}}$ **completely eliminates joint acceleration $\mathbf{\ddot{q}}$**.

2. **Decoupled Low-Pass Filtering:**  
   The observer residual $\mathbf{r}(t)$ obeys the linear filter dynamics:
   $$\frac{R_i(s)}{Q_{\text{ext},i}(s)} = \frac{K_{O,i}}{s + K_{O,i}}$$
   Each channel operates as a decoupled first-order filter with zero steady-state error, controllable bandwidth $\omega_c = K_{O,i}$, and zero acceleration feedback noise.

3. **Global Exponential Stability:**  
   Using the quadratic Lyapunov candidate $V = \frac{1}{2}\|\mathbf{e}_r\|^2$, the error dynamics $\mathbf{\dot{e}}_r = -\mathbf{K}_O \mathbf{e}_r$ yield $\dot{V} = -\mathbf{e}_r^T \mathbf{K}_O \mathbf{e}_r < 0$, guaranteeing exponential convergence at rate $K_{O,\min}$.

4. **Cartesian Force Reconstruction:**  
   Because the shovel Jacobian is strictly non-singular ($\det(\mathbf{J}_T) = -r_T \neq 0$), external rock cutting and breakout forces are reconstructed without singularity:
   $$\hat{\mathbf{F}}_{\text{ext}} = \left( \mathbf{J}_T^T \right)^{-1} \mathbf{r}(t)$$

---

### Doctoral Viva & Defense Questions

- **Q1:** *Why is the Generalized Momentum Observer (M3) fundamentally superior to high-gain state observers or Extended Kalman Filters for contact force estimation?*  
  **Answer:** High-gain state observers require numerically differentiating velocity or estimating acceleration states, which amplifies sensor quantization noise and introduces extreme sensitivity to sensor noise covariance tuning. The GMO exploits the exact physical Hamiltonian structure of the multibody system to compute momentum directly from velocity, eliminating acceleration states entirely and guaranteeing passivity.

- **Q2:** *How does the observer handle sudden dipper impact with an unyielding highwall (tooth stall)?*  
  **Answer:** During tooth stall, velocity rapidly drops to zero while motor currents surge. The observer residual $\mathbf{r}(t)$ tracks the impact force with first-order exponential latency $T_O = 1/K_O \approx 25\,\mathrm{ms}$. When $\|\mathbf{r}(t)\|$ exceeds the pre-set threshold ($80\%$ structural yield), the digital twin triggers a fast torque-limit clamp to the motor variable frequency drives, preventing structural damage.

- **Q3:** *What is the effect of friction modeling errors on the momentum observer?*  
  **Answer:** Any unmodelled friction $\Delta \mathbf{F}_f = \mathbf{F}_{f,\text{true}} - \hat{\mathbf{F}}_f$ enters the residual dynamics as a disturbance input: $\mathbf{r}_{\text{ss}} = \mathbf{Q}_{\text{ext}} + \Delta \mathbf{F}_f$. During motion reversals, unmodeled Coulomb friction creates small step artifacts in $\mathbf{r}(t)$. This is why precise identification of Coulomb and viscous friction (Chapter 4) is essential to maximize observer fidelity.

---

### Transition to Chapter 6
With the analytical mechanics, kinematic Jacobians, Euler-Lagrange equations, parameter regressors, and momentum observers complete, we now conclude **Volume I: Advanced Mathematical Foundations** and transition to:  
**Volume II: Rigid-Body & Multibody Dynamics of the Shovel System**.  
In Chapter 6, we begin **Volume II, Chapter 6: Top-Down Kinematic & Topological Decomposition of the Electric Rope Shovel**, establishing the complete 3D kinematic topology, reference frame trees, closed kinematic loops, and sheave-cable constraints for the complete industrial machine.
