# VOLUME I: ADVANCED MATHEMATICAL FOUNDATIONS

---

# CHAPTER 2: Multivariable Analysis, Kinematic Jacobians and Differential Geometry

**Author:** Multidisciplinary Research Supervisor & Technical Author  
**Project:** *AI-Enabled Platform for Diagnosis, Prognosis and Health Management of Mining Electrical Systems: Case Study on Dragline / Electric Rope Shovel*  
**Grant Reference:** IIT/SRIC/R/AEH/2026/104 (CMPDI / Coal India Limited — MT-189)  
**Target Machines:** Machine A (P&H 2100BL Benchmark) & Machine B (TZ WK-20 Deployment)  
**Pedagogical Level:** Postgraduate / Research Grade (B.Tech Mechanical Foundation Assumed)

---

> [!NOTE]
> ### RESEARCH STUDY NOTEBOOK ARCHITECTURE
> In this chapter, we bridge the gap between static coordinate transformations (Chapter 1) and continuous mechanism motion. You will learn how velocities, accelerations, and forces transform between the internal actuator joint space $\mathcal{Q} \subset \mathbb{R}^n$ and the external Cartesian task space $\mathcal{X} \subset \mathbb{R}^m$.
> 
> By the end of this chapter, you will be able to:
> 1. Formulate both the **Analytical Jacobian** $\mathbf{J}_a(\mathbf{q})$ and the **Geometric Jacobian** $\mathbf{J}(\mathbf{q})$, and articulate the exact physical and topological conditions under which they differ.
> 2. Derive convective accelerations and the velocity-product term $\dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$ that generates centripetal and Coriolis accelerations.
> 3. Prove the **Principle of Virtual Work** duality: $\boldsymbol{\tau} = \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}}$, and explain why the transposed Jacobian maps Cartesian digging resistance forces directly to motor-side generalized forces.
> 4. Analyze kinematic singularities, singular value decomposition (SVD), condition numbers, and Yoshikawa manipulability ellipsoids in the electric rope shovel workspace.
> 5. Implement the complete closed-form kinematics and Jacobians for both the **P&H 2100BL benchmark** ($50\,\mathrm{t}$ class) and the **TZ WK-20 deployment shovel** ($55\,\mathrm{t}$ payload class).

---

## 2.1 Physical Intuition & Engineering Motivation in Mining Machinery

### 2.1.1 Why Scalar Calculus Fails on a Heavy Excavator
On an electric rope shovel, an operator in the cab commands two primary motions during digging:
1. **The Crowd Drive:** An induction or DC motor driving a pinion-and-rack mechanism that pushes the dipper handle linearly outward or pulls it inward ($d_4 \in [7.0\,\mathrm{m}, 14.5\,\mathrm{m}]$ on P&H 2100BL).
2. **The Hoist Drive:** An induction or DC motor driving a hoist drum that winds or unwinds steel wire ropes over boom-point sheaves, lifting or lowering the dipper through the saddle block pivot ($q_3 \in [-30^\circ, +45^\circ]$).

Notice that neither actuator moves the dipper tooth tip $T$ in pure horizontal ($X$) or pure vertical ($Y$) directions. 
- Extending the crowd handle increases the radial distance from the saddle block $O_3$, but because the saddle block is pivoted at angle $q_3$, crowd extension moves the tooth both horizontally forward and vertically downward (or upward, depending on $q_3$).
- Winding the hoist rope rotates the dipper handle upward about the pivot $O_3$, creating an instantaneous velocity perpendicular to the handle axis.

```
                          [Boom Point Sheave O_s]
                                  / \
                                 /   \  Hoist Rope (Tension F_h)
                                /     \
   [Saddle Pivot O_3]          /       v
          o===================/====[Bail Pin B]
          |   \              /           |
          |    \  Dipper    /      [Dipper Body]
          |     \ Handle   /             |
          |      \========/=====>        \---> [Tooth Tip T]
          |         (d_4)                       \
    Ground Level                                 v Contact Force F_ext
  ========================================================================
```

If the tooth tip encounters an unyielding bank of iron ore or granite:
- What fraction of that contact force is resisted by the crowd motor pinion teeth?
- What fraction is resisted by hoist rope tension?
- How much bending moment is induced in the boom structural chords?

You cannot answer these questions by drawing 2D static free-body diagrams for every instantaneous position. As the dipper moves at speeds up to $2.5\,\mathrm{m/s}$, the geometry changes continuously. 

**The Kinematic Jacobian is the fundamental multidimensional transmission ratio of the excavator.** It maps instantaneous joint velocities $\dot{\mathbf{q}} = [\dot{q}_3, \dot{d}_4]^T$ into Cartesian tooth velocities $\mathbf{v}_T = [\dot{x}_T, \dot{y}_T]^T$, and through energy conservation (virtual work), its transpose $\mathbf{J}^T$ maps high contact forces back onto the motor drives.

---

## 2.2 Multivariable Mappings: Configuration Space to Task Space

### 2.2.1 Configuration Manifold $\mathcal{Q}$ vs. Task Space $\mathcal{X}$
Let the generalized coordinates of the shovel be denoted by the vector $\mathbf{q} \in \mathcal{Q} \subset \mathbb{R}^n$:

$$\mathbf{q} = \begin{bmatrix} q_1 \\ q_2 \\ \vdots \\ q_n \end{bmatrix} \in \mathbb{R}^n \tag{2.1}$$

For planar crowd-hoist excavation models ($n = 2$):
- $q_3$: Saddle block rotation angle with respect to the boom reference line ($\mathrm{rad}$).
- $d_4$: Linear extension distance from saddle pivot $O_3$ to dipper bail pin / centroid ($\mathrm{m}$).

Let the position and orientation of the end-effector (dipper tooth tip $T$ or bail pin $B$) be represented by the task space vector $\mathbf{x} \in \mathcal{X} \subset \mathbb{R}^m$:

$$\mathbf{x} = \mathbf{f}(\mathbf{q}) \tag{2.2}$$

where $\mathbf{f}: \mathcal{Q} \to \mathcal{X}$ is the nonlinear **Forward Kinematic Mapping**.

```
    CONFIGURATION SPACE (Joints)              TASK SPACE (Cartesian World)
       q = [q_3, d_4]^T                          x = [x_T, y_T]^T
    +-----------------------+                 +-----------------------+
    | Saddle Angle q_3      |  Forward Map f  | Tooth X Position x_T  |
    | Crowd Extension d_4   | --------------> | Tooth Y Position y_T  |
    +-----------------------+                 +-----------------------+
                |                                         |
                | Joint Velocities q_dot                  | Cartesian Velocities v_T
                v                                         v
    +-----------------------+                 +-----------------------+
    | [q_dot_3, d_dot_4]^T  |  Jacobian J(q)  | [x_dot_T, y_dot_T]^T  |
    +-----------------------+ --------------> +-----------------------+
                ^                                         |
                |             Transposed J^T              | Digging Force F_ext
                +-----------------------------------------+
                    Generalized Torques tau = J^T * F_ext
```

---

## 2.3 The Analytical Kinematic Jacobian

### 2.3.1 Definition via Total Differential
Suppose the Cartesian position vector $\mathbf{x}(t)$ is an explicit multivariable function of joint coordinates $\mathbf{q}(t)$:

$$\mathbf{x}(t) = \begin{bmatrix} f_1(q_1, q_2, \dots, q_n) \\ f_2(q_1, q_2, \dots, q_n) \\ \vdots \\ f_m(q_1, q_2, \dots, q_n) \end{bmatrix} \tag{2.3}$$

By the chain rule of multivariable calculus, the total time derivative of the $i$-th coordinate $x_i(t)$ is:

$$\dot{x}_i = \frac{d}{dt} f_i(\mathbf{q}(t)) = \sum_{j=1}^n \frac{\partial f_i}{\partial q_j} \frac{dq_j}{dt} = \sum_{j=1}^n \frac{\partial f_i}{\partial q_j} \dot{q}_j \tag{2.4}$$

Stating Equation (2.4) in matrix-vector form yields:

$$\mathbf{\dot{x}} = \mathbf{J}_a(\mathbf{q}) \mathbf{\dot{q}} \tag{2.5}$$

where $\mathbf{J}_a(\mathbf{q}) \in \mathbb{R}^{m \times n}$ is the **Analytical Kinematic Jacobian Matrix**:

$$\mathbf{J}_a(\mathbf{q}) = \frac{\partial \mathbf{f}(\mathbf{q})}{\partial \mathbf{q}} = \begin{bmatrix} 
\frac{\partial f_1}{\partial q_1} & \frac{\partial f_1}{\partial q_2} & \cdots & \frac{\partial f_1}{\partial q_n} \\[8pt]
\frac{\partial f_2}{\partial q_1} & \frac{\partial f_2}{\partial q_2} & \cdots & \frac{\partial f_2}{\partial q_n} \\[8pt]
\vdots & \vdots & \ddots & \vdots \\[8pt]
\frac{\partial f_m}{\partial q_1} & \frac{\partial f_m}{\partial q_2} & \cdots & \frac{\partial f_m}{\partial q_n}
\end{bmatrix} \tag{2.6}$$

> [!IMPORTANT]
> ### PHYSICAL MEANING OF THE $j$-TH COLUMN OF $\mathbf{J}_a$
> The $j$-th column of $\mathbf{J}_a(\mathbf{q})$:
> 
> $$\mathbf{J}_{a, *j} = \frac{\partial \mathbf{f}}{\partial q_j} = \left. \mathbf{\dot{x}} \right|_{\dot{q}_j = 1, \; \dot{q}_{k \neq j} = 0}$$
> 
> It represents the instantaneous linear velocity vector of the end-effector when the $j$-th actuator moves at unit velocity ($\dot{q}_j = 1\,\mathrm{rad/s}$ or $1\,\mathrm{m/s}$) while all other actuators are locked stationary ($\dot{q}_{k \neq j} = 0$).

---

## 2.4 The Geometric Kinematic Jacobian

In three-dimensional robotics and multibody mechanisms, the end-effector orientation cannot always be integrated into a globally non-singular minimal set of coordinates (e.g., Euler angles suffer from gimbal lock representation singularities). 

Therefore, roboticists define the **Geometric Jacobian** $\mathbf{J}(\mathbf{q})$, which directly maps joint velocities into the physical twist vector $\boldsymbol{\xi} = [\mathbf{v}_e^T, \boldsymbol{\omega}_e^T]^T \in \mathbb{R}^6$:

$$\boldsymbol{\xi} = \begin{bmatrix} \mathbf{v}_e \\ \boldsymbol{\omega}_e \end{bmatrix} = \mathbf{J}(\mathbf{q}) \mathbf{\dot{q}} = \begin{bmatrix} \mathbf{J}_v(\mathbf{q}) \\ \mathbf{J}_\omega(\mathbf{q}) \end{bmatrix} \mathbf{\dot{q}} \tag{2.7}$$

where:
- $\mathbf{v}_e \in \mathbb{R}^3$ is the physical translational velocity of the end-effector origin.
- $\boldsymbol{\omega}_e \in \mathbb{R}^3$ is the true physical angular velocity vector of the end-effector body.

### 2.4.1 Column-by-Column Construction of Geometric Jacobian
For a serial open-chain or articulated mechanism with $n$ joints, the columns of $\mathbf{J}(\mathbf{q}) = [\mathbf{J}_1, \mathbf{J}_2, \dots, \mathbf{J}_n]$ are constructed directly from geometry:

For Joint $i$:
1. **If Joint $i$ is Revolute (Rotational):**
   $$\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{v, i} \\ \mathbf{J}_{\omega, i} \end{bmatrix} = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix} \tag{2.8}$$
   where $\mathbf{z}_{i-1}$ is the unit vector along the rotation axis of joint $i$, $\mathbf{p}_{i-1}$ is the Cartesian position of the joint axis origin, and $\mathbf{p}_e$ is the position of the end-effector.

2. **If Joint $i$ is Prismatic (Linear Translation):**
   $$\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{v, i} \\ \mathbf{J}_{\omega, i} \end{bmatrix} = \begin{bmatrix} \mathbf{z}_{i-1} \\ \mathbf{0}_{3 \times 1} \end{bmatrix} \tag{2.9}$$
   because linear translation contributes translational velocity along its direction of travel $\mathbf{z}_{i-1}$, but contributes zero angular velocity to the body ($\mathbf{J}_{\omega, i} = \mathbf{0}$).

### 2.4.2 Relationship Between Analytical and Geometric Jacobians
For position mapping in $\mathbb{R}^3$, the translational blocks are identical: $\mathbf{J}_v(\mathbf{q}) = \mathbf{J}_{a, \text{pos}}(\mathbf{q})$. 

However, for rotational motion, if orientation is parameterized by Euler angles $\boldsymbol{\phi} = [\phi, \theta, \psi]^T$, the relationship between Euler angle rates $\boldsymbol{\dot{\phi}}$ and physical angular velocity $\boldsymbol{\omega}$ is:

$$\boldsymbol{\omega} = \mathbf{T}(\boldsymbol{\phi}) \boldsymbol{\dot{\phi}} \tag{2.10}$$

where $\mathbf{T}(\boldsymbol{\phi})$ is a configuration-dependent transformation matrix. Therefore:

$$\mathbf{J}(\mathbf{q}) = \begin{bmatrix} \mathbf{I}_{3} & \mathbf{0} \\ \mathbf{0} & \mathbf{T}(\boldsymbol{\phi}) \end{bmatrix} \mathbf{J}_a(\mathbf{q}) \tag{2.11}$$

> [!WARNING]
> ### GIMBAL LOCK REPRESENTATION SINGULARITY
> When $\det(\mathbf{T}(\boldsymbol{\phi})) = 0$ (for pitch angle $\theta = \pm 90^\circ$ in $Z$-$Y$-$X$ Euler angles), $\mathbf{J}_a$ becomes singular even though the physical shovel mechanism is in a completely non-singular kinematic posture! 
> For our mining digital twin, we formulate the kinematics planar in $SO(2)$ or spatial using the Geometric Jacobian $\mathbf{J}(\mathbf{q})$ to completely eliminate mathematical representation singularities.

---

## 2.5 Acceleration Kinematics & Convective Velocity Products

### 2.5.1 Derivation of the $\dot{\mathbf{J}}\dot{\mathbf{q}}$ Term
To compute dynamic equations and structural inertial loads, we must differentiate velocity kinematics with respect to time:

$$\mathbf{v}(t) = \mathbf{J}(\mathbf{q}(t)) \mathbf{\dot{q}}(t) \tag{2.12}$$

Applying the product rule of calculus:

$$\mathbf{a}(t) = \mathbf{\dot{v}}(t) = \frac{d}{dt} \left[ \mathbf{J}(\mathbf{q}(t)) \mathbf{\dot{q}}(t) \right] = \mathbf{J}(\mathbf{q}) \mathbf{\ddot{q}} + \frac{d\mathbf{J}(\mathbf{q})}{dt} \mathbf{\dot{q}} \tag{2.13}$$

Notice that $\mathbf{J}(\mathbf{q})$ depends on time strictly through the joint trajectory $\mathbf{q}(t)$. By the multivariable chain rule:

$$\frac{d\mathbf{J}(\mathbf{q})}{dt} = \sum_{k=1}^n \frac{\partial \mathbf{J}(\mathbf{q})}{\partial q_k} \dot{q}_k = \mathbf{\dot{J}}(\mathbf{q}, \mathbf{\dot{q}}) \tag{2.14}$$

Substituting Equation (2.14) into (2.13) gives the fundamental **Acceleration Kinematics Equation**:

$$\mathbf{a} = \mathbf{J}(\mathbf{q}) \mathbf{\ddot{q}} + \mathbf{\dot{J}}(\mathbf{q}, \mathbf{\dot{q}}) \mathbf{\dot{q}} \tag{2.15}$$

```
                           ACCELERATION DECOMPOSITION
              a = J(q) * q_ddot  +  J_dot(q, q_dot) * q_dot
                  \___________/     \_____________________/
                        |                      |
                 Apparent Joint          Convective Term
                  Accelerations         (Centripetal & Coriolis)
```

> [!NOTE]
> ### PHYSICAL INTERPRETATION OF ACCELERATION COMPONENTS
> 1. $\mathbf{J}(\mathbf{q}) \mathbf{\ddot{q}}$ represents **instantaneous linear acceleration due to joint angular and linear accelerations**. If $\mathbf{\dot{q}} = \mathbf{0}$, this is the only non-zero term.
> 2. $\mathbf{\dot{J}}(\mathbf{q}, \mathbf{\dot{q}})\mathbf{\dot{q}}$ is quadratic in velocity ($\propto \dot{q}_i \dot{q}_j$). It contains:
>    - **Centripetal accelerations:** Proportional to $\dot{q}_i^2$, pointing radially inward toward instantaneous centers of rotation.
>    - **Coriolis accelerations:** Proportional to $2 \dot{q}_i \dot{q}_j$ ($i \neq j$), resulting from the dipper handle translating linearly at speed $\dot{d}_4$ inside a saddle block rotating at angular rate $\dot{q}_3$.

---

## 2.6 The Principle of Virtual Work & Force-Velocity Duality

### 2.6.1 Mathematical Derivation of $\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}_{\text{ext}}$
One of the most profound principles in analytical mechanics is the duality between kinematics (motions) and statics (forces).

Consider the shovel mechanism in static equilibrium under:
- Actuator generalized forces $\boldsymbol{\tau} = [\tau_3, F_4]^T \in \mathbb{R}^n$ (where $\tau_3$ is saddle torque in $\mathrm{N\cdot m}$, and $F_4$ is crowd rack force in $\mathrm{N}$).
- An external contact force $\mathbf{F}_{\text{ext}} = [F_x, F_y]^T \in \mathbb{R}^m$ applied by the rock bank against the dipper tooth tip.

Let the mechanism undergo an infinitesimal virtual displacement $\delta \mathbf{q}$ that is kinematically admissible (consistent with all mechanical constraints). The resulting virtual displacement of the tooth tip is:

$$\delta \mathbf{x} = \mathbf{J}(\mathbf{q}) \delta \mathbf{q} \tag{2.16}$$

By the **Principle of Virtual Work**, the total virtual work $\delta W$ performed by all active generalized forces and external loads during any kinematically admissible virtual displacement must equal zero:

$$\delta W = \delta W_{\text{actuators}} - \delta W_{\text{external}} = 0 \tag{2.17}$$

$$\delta W = \boldsymbol{\tau}^T \delta \mathbf{q} - \mathbf{F}_{\text{ext}}^T \delta \mathbf{x} = 0 \tag{2.18}$$

Substitute the kinematic relationship $\delta \mathbf{x} = \mathbf{J}(\mathbf{q}) \delta \mathbf{q}$ into Equation (2.18):

$$\boldsymbol{\tau}^T \delta \mathbf{q} - \mathbf{F}_{\text{ext}}^T \left( \mathbf{J}(\mathbf{q}) \delta \mathbf{q} \right) = 0 \tag{2.19}$$

Factoring the transpose using $(\mathbf{A}\mathbf{B})^T = \mathbf{B}^T \mathbf{A}^T$:

$$\boldsymbol{\tau}^T \delta \mathbf{q} - \left( \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{ext}} \right)^T \delta \mathbf{q} = 0 \tag{2.20}$$

$$\left( \boldsymbol{\tau} - \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{ext}} \right)^T \delta \mathbf{q} = 0 \tag{2.21}$$

Because the virtual displacement $\delta \mathbf{q}$ is completely arbitrary and unconstrained, Equation (2.21) can only hold identically if the expression in parentheses vanishes:

$$\boldsymbol{\tau} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{ext}} \tag{2.22}$$

```
=============================================================================
                  THE FUNDAMENTAL KINEMATIC-STATIC DUALITY
=============================================================================
             FORWARD VELOCITY:     v_x    =   J(q)   *   q_dot
             STATIC FORCE:         tau    =   J(q)^T *   F_ext
=============================================================================
```

> [!TIP]
> ### PHYSICAL IMPLICATION FOR ELECTRIC ROPE SHOVEL DIAGNOSTICS
> Equation (2.22) is the mathematical bedrock of **Sensorless Digging Resistance Estimation**:
> If we measure motor currents to infer actuator forces $\boldsymbol{\tau}$ and we know the joint coordinates $\mathbf{q}$ from resolvers, we can invert the relationship:
> 
> $$\mathbf{F}_{\text{ext}} = \left( \mathbf{J}^T(\mathbf{q}) \right)^{-1} \boldsymbol{\tau}$$
> 
> to compute the rock resistance forces acting at the dipper teeth without installing expensive, fragile load cells on the boom tip!

---

## 2.7 Kinematic Singularities, Condition Numbers & Manipulability

### 2.7.1 Definition of Singular Configurations
A configuration $\mathbf{q}^* \in \mathcal{Q}$ is defined as a **Kinematic Singularity** if the Jacobian matrix loses rank:

$$\operatorname{rank}(\mathbf{J}(\mathbf{q}^*)) < \min(m, n) \tag{2.23}$$

For square Jacobians ($m = n = 2$ in planar shovel kinematics), this occurs precisely when the determinant vanishes:

$$\det(\mathbf{J}(\mathbf{q}^*)) = 0 \tag{2.24}$$

### 2.7.2 Physical Consequences of Singularities
1. **Loss of Mobility in Task Space:** At a singular configuration, there exists at least one Cartesian direction along which the dipper tooth cannot move, regardless of how fast the joint motors run.
2. **Infinite Mechanical Advantage:** Conversely, along that locked Cartesian direction, an infinitely large external force $\mathbf{F}_{\text{ext}}$ can be supported by zero actuator effort ($\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}_{\text{ext}} = \mathbf{0}$). This can cause catastrophic structural failure (buckled dipper handle or snapped boom chords) because the motors cannot detect the overloading through their drive currents!

### 2.7.3 Singular Value Decomposition (SVD) of the Jacobian
To quantify how close the excavator is to a singularity, perform SVD on $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{m \times n}$:

$$\mathbf{J} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T = \sum_{i=1}^{\min(m,n)} \sigma_i \mathbf{u}_i \mathbf{v}_i^T \tag{2.25}$$

where:
- $\mathbf{U} = [\mathbf{u}_1, \dots, \mathbf{u}_m] \in \mathbb{R}^{m \times m}$ is an orthogonal matrix whose columns $\mathbf{u}_i$ define the principal axes of the velocity ellipsoid in task space.
- $\mathbf{V} = [\mathbf{v}_1, \dots, \mathbf{v}_n] \in \mathbb{R}^{n \times n}$ is an orthogonal matrix whose columns $\mathbf{v}_i$ define joint velocity directions.
- $\boldsymbol{\Sigma} = \operatorname{diag}(\sigma_1, \sigma_2, \dots, \sigma_{\min(m,n)})$ with singular values sorted: $\sigma_1 \ge \sigma_2 \ge \dots \ge \sigma_p \ge 0$.

### 2.7.4 Yoshikawa Manipulability Measure
Yoshikawa defined the scalar **Manipulability Index** $\mu(\mathbf{q})$ as the volume of the velocity ellipsoid:

$$\mu(\mathbf{q}) = \sqrt{\det(\mathbf{J}(\mathbf{q}) \mathbf{J}^T(\mathbf{q}))} = \prod_{i=1}^{\min(m,n)} \sigma_i \tag{2.26}$$

For square non-singular Jacobians: $\mu(\mathbf{q}) = |\det(\mathbf{J}(\mathbf{q}))|$.

### 2.7.5 Condition Number $\kappa(\mathbf{J})$
The **Kinematic Condition Number** measures isotropy (directional uniformity of velocity and force transmission):

$$\kappa(\mathbf{J}) = \frac{\sigma_{\max}(\mathbf{J})}{\sigma_{\min}(\mathbf{J})} = \frac{\sigma_1}{\sigma_m} \ge 1.0 \tag{2.27}$$

- **Isotropic Point ($\kappa = 1$):** Velocity and force capabilities are completely uniform in all Cartesian directions. The velocity ellipsoid is a perfect sphere.
- **Singular Point ($\kappa \to \infty$):** The smallest singular value $\sigma_m \to 0$. The ellipsoid collapses into a line or flat pancake, indicating zero mobility along the collapsed axis.

---

## 2.8 Shovel Case Study: 2-DOF Crowd-Hoist Planar Kinematics

Now we apply this theory to the planar crowd-hoist mechanism shared by both **Machine A (P&H 2100BL benchmark)** and **Machine B (TZ WK-20 deployment shovel)**.

```
                  PLANAR 2-DOF ELECTRIC ROPE SHOVEL GEOMETRY
                                                              Y_0
                                                               ^
                                [Boom Sheave S]               |
                                      *                        |
                                     /| \                      +-----> X_0
                                    / |  \  Hoist Rope (L_h)
                                   /  |   \
                                  /   |    \
                                 /    |     v
                     Boom (L_b) /     |   [Bail Pin B]
                               /      |    /    \
                              /       |   /      \  Dipper
                             /        |  /        \ Body
               [Saddle O_3] o=========+=/==========* [Tooth Tip T]
                            | \       |/
                            |  \      /  Dipper Handle
                            |   \    /   (d_4)
        [Boom Foot O_1]     |    \  /
              o-------------+-----*
             /
   [Cab/Gantry O_0]
========================================================================
```

### 2.8.1 Coordinate System Definitions
- **Inertial Base Frame $\{0\}$:** Origin $O_0$ located at the revolving frame center-of-rotation on the deck.
  - $X_0$: Horizontal forward along the machine symmetry line.
  - $Y_0$: Vertical upward perpendicular to the deck.
- **Boom Foot Frame $\{1\}$:** Origin $O_1$ located at the boom foot pin $(x_{\text{boom}}, y_{\text{boom}})$.
- **Boom Geometry:** The boom is inclined at fixed angle $\alpha_b$ (typically $45^\circ$ for P&H 2100BL and WK-20). The boom axis extends from $O_1$ to the boom-point sheave center $S = (x_s, y_s)$ with length $L_b$.
- **Saddle Block Pivot $O_3$:** Positioned along the boom at a fixed distance $L_{b1}$ from the boom foot pin:
  $$x_3 = x_{\text{boom}} + L_{b1} \cos \alpha_b \tag{2.28a}$$
  $$y_3 = y_{\text{boom}} + L_{b1} \sin \alpha_b \tag{2.28b}$$
- **Generalized Coordinates $\mathbf{q} = [q_3, d_4]^T$:**
  - $q_3$: Absolute orientation angle of the dipper handle relative to the positive horizontal axis $X_0$ ($\mathrm{rad}$).
  - $d_4$: Linear extension of the dipper handle rack from saddle pivot $O_3$ to the bail pin $B$ ($\mathrm{m}$).
- **Tooth Tip Geometry:** The dipper body is rigidly bolted to the end of the handle. In the handle body frame, the tooth tip $T$ is offset from bail pin $B$ by:
  - Longitudinal offset along the handle axis: $L_{t\parallel}$ ($\mathrm{m}$).
  - Perpendicular offset downward perpendicular to handle axis: $L_{t\perp}$ ($\mathrm{m}$).

### 2.8.2 Machine A vs. Machine B Parameter Table

| Parameter Description | Symbol | Units | Machine A: P&H 2100BL Benchmark | Machine B: TZ WK-20 Deployment | Evidence Source |
| :--- | :---: | :---: | :---: | :---: | :---: |
| Rated Dipper Payload | $M_{\text{payload}}$ | $\mathrm{t}$ | $20.0\,\mathrm{t}$ ($13.8\,\mathrm{m}^3$) | $55.0\,\mathrm{t}$ ($20.0\,\mathrm{m}^3$) | [A] OEM Specs |
| Boom Chord Length | $L_b$ | $\mathrm{m}$ | $16.76\,\mathrm{m}$ ($55\,\mathrm{ft}$) | $21.50\,\mathrm{m}$ | [A] Spec Audit |
| Boom Inclination Angle | $\alpha_b$ | $\mathrm{deg}$ | $45.0^\circ$ | $45.0^\circ$ | [A] Manual |
| Boom Foot Coordinates | $(x_b, y_b)$ | $\mathrm{m}$ | $(3.65, 2.10)\,\mathrm{m}$ | $(4.85, 2.80)\,\mathrm{m}$ | [A] Assembly |
| Saddle Pivot Position | $L_{b1}$ | $\mathrm{m}$ | $8.84\,\mathrm{m}$ | $11.20\,\mathrm{m}$ | [A] M1 Model |
| Dipper + Handle Empty Mass | $m_d$ | $\mathrm{kg}$ | $28,500\,\mathrm{kg}$ | $68,000\,\mathrm{kg}$ | [A] M1 / WK-20 |
| Crowd Stroke Limits | $[d_{4,\min}, d_{4,\max}]$ | $\mathrm{m}$ | $[7.50, 14.50]\,\mathrm{m}$ | $[9.00, 18.20]\,\mathrm{m}$ | [A] Stroke Audit |
| Saddle Angle Range | $[q_{3,\min}, q_{3,\max}]$ | $\mathrm{deg}$ | $[-35.0^\circ, +48.0^\circ]$ | $[-30.0^\circ, +50.0^\circ]$ | [A] M1 Limits |
| Maximum Hoist Line Pull | $F_{h,\max}$ | $\mathrm{kN}$ | $750\,\mathrm{kN}$ | $1,650\,\mathrm{kN}$ | [A] Motor Data |
| Maximum Crowd Push Force | $F_{c,\max}$ | $\mathrm{kN}$ | $350\,\mathrm{kN}$ | $800\,\mathrm{kN}$ | [A] Motor Data |

---

### 2.8.3 Analytical Derivation of Forward Kinematics

#### Position of Dipper Bail Pin $B(x_B, y_B)$
The bail pin position in the base frame $\{0\}$ is obtained by vector addition from the saddle pivot $O_3$:

$$x_B(q_3, d_4) = x_3 + d_4 \cos q_3 \tag{2.29a}$$

$$y_B(q_3, d_4) = y_3 + d_4 \sin q_3 \tag{2.29b}$$

#### Position of Dipper Tooth Tip $T(x_T, y_T)$
Let the tooth tip be offset by longitudinal distance $L_{tx}$ along the handle direction and $L_{ty}$ perpendicular to the handle (measured downward):

$$x_T(q_3, d_4) = x_3 + (d_4 + L_{tx}) \cos q_3 + L_{ty} \sin q_3 \tag{2.30a}$$

$$y_T(q_3, d_4) = y_3 + (d_4 + L_{tx}) \sin q_3 - L_{ty} \cos q_3 \tag{2.30b}$$

To simplify subsequent notation without loss of generality, let $r_T = d_4 + L_{tx}$ denote the effective radial extension from the saddle pivot to the tooth tip along the handle axis. Then:

$$x_T = x_3 + r_T \cos q_3 + L_{ty} \sin q_3 \tag{2.31a}$$

$$y_T = y_3 + r_T \sin q_3 - L_{ty} \cos q_3 \tag{2.31b}$$

---

### 2.8.4 Closed-Form Derivation of the $2 \times 2$ Tooth Tip Jacobian $\mathbf{J}_T$

Differentiating Equations (2.31a) and (2.31b) with respect to the generalized coordinates $\mathbf{q} = [q_3, d_4]^T$:

#### Element $J_{11} = \frac{\partial x_T}{\partial q_3}$:
$$J_{11} = \frac{\partial}{\partial q_3} \left[ x_3 + r_T \cos q_3 + L_{ty} \sin q_3 \right] = -r_T \sin q_3 + L_{ty} \cos q_3 \tag{2.32a}$$

#### Element $J_{12} = \frac{\partial x_T}{\partial d_4}$:
Since $r_T = d_4 + L_{tx}$, we have $\frac{\partial r_T}{\partial d_4} = 1$:
$$J_{12} = \frac{\partial}{\partial d_4} \left[ x_3 + r_T \cos q_3 + L_{ty} \sin q_3 \right] = \cos q_3 \tag{2.32b}$$

#### Element $J_{21} = \frac{\partial y_T}{\partial q_3}$:
$$J_{21} = \frac{\partial}{\partial q_3} \left[ y_3 + r_T \sin q_3 - L_{ty} \cos q_3 \right] = r_T \cos q_3 + L_{ty} \sin q_3 \tag{2.32c}$$

#### Element $J_{22} = \frac{\partial y_T}{\partial d_4}$:
$$J_{22} = \frac{\partial}{\partial d_4} \left[ y_3 + r_T \sin q_3 - L_{ty} \cos q_3 \right] = \sin q_3 \tag{2.32d}$$

Assembling the elements into matrix form yields the **Analytical Jacobian of the Dipper Tooth Tip**:

$$\mathbf{J}_T(q_3, d_4) = \begin{bmatrix}
-r_T \sin q_3 + L_{ty} \cos q_3 & \cos q_3 \\[6pt]
 r_T \cos q_3 + L_{ty} \sin q_3 & \sin q_3
\end{bmatrix} \tag{2.33}$$

---

### 2.8.5 Determinant and Singularity Analysis of $\mathbf{J}_T$

Let us compute the determinant of the tooth tip Jacobian:

$$\det(\mathbf{J}_T) = J_{11} J_{22} - J_{12} J_{21} \tag{2.34}$$

Substitute the entries from Equation (2.33):

$$\det(\mathbf{J}_T) = (-r_T \sin q_3 + L_{ty} \cos q_3)(\sin q_3) - (\cos q_3)(r_T \cos q_3 + L_{ty} \sin q_3) \tag{2.35}$$

Expand each product:

$$\det(\mathbf{J}_T) = -r_T \sin^2 q_3 + L_{ty} \cos q_3 \sin q_3 - r_T \cos^2 q_3 - L_{ty} \sin q_3 \cos q_3 \tag{2.36}$$

Notice that the cross-terms $+L_{ty} \sin q_3 \cos q_3$ and $-L_{ty} \sin q_3 \cos q_3$ **cancel out exactly**!

$$\det(\mathbf{J}_T) = -r_T (\sin^2 q_3 + \cos^2 q_3) \tag{2.37}$$

Applying the fundamental trigonometric identity $\sin^2 q_3 + \cos^2 q_3 = 1$:

$$\det(\mathbf{J}_T(q_3, d_4)) = -r_T = -(d_4 + L_{tx}) \tag{2.38}$$

> [!IMPORTANT]
> ### PROFOUND ENGINEERING RESULT: WORKSPACE SINGULARITY IMMUNITY
> Equation (2.38) reveals an extraordinary mechanical property of the electric rope shovel:
> 
> The determinant $\det(\mathbf{J}_T) = -(d_4 + L_{tx})$ depends **solely on the total handle extension $r_T$**, and is **completely independent of the saddle angle $q_3$**!
> 
> A kinematic singularity occurs if and only if:
> 
> $$\det(\mathbf{J}_T) = 0 \iff d_4 + L_{tx} = 0 \iff d_4 = -L_{tx}$$
> 
> On the physical machine, $d_4 \in [7.5\,\mathrm{m}, 14.5\,\mathrm{m}]$ for P&H 2100BL and $[9.0\,\mathrm{m}, 18.2\,\mathrm{m}]$ for WK-20. The tooth offset $L_{tx} \approx 2.4\,\mathrm{m} > 0$. 
> 
> Therefore, $r_T \ge 9.9\,\mathrm{m} \gg 0$ throughout the entire operating envelope. **The planar shovel mechanism NEVER encounters an internal kinematic singularity anywhere inside its physical workspace!** The Jacobian is strictly non-singular everywhere.

---

### 2.8.6 Closed-Form Derivation of the Inverse Jacobian $\mathbf{J}_T^{-1}$

Because $\det(\mathbf{J}_T) = -r_T \neq 0$, the exact inverse exists everywhere. For any $2 \times 2$ matrix:

$$\mathbf{A} = \begin{bmatrix} a & b \\ c & d \end{bmatrix} \implies \mathbf{A}^{-1} = \frac{1}{ad - bc} \begin{bmatrix} d & -b \\ -c & a \end{bmatrix}$$

Applying this formula to $\mathbf{J}_T$:

$$\mathbf{J}_T^{-1}(q_3, d_4) = \frac{1}{-r_T} \begin{bmatrix}
\sin q_3 & -\cos q_3 \\[6pt]
-(r_T \cos q_3 + L_{ty} \sin q_3) & -r_T \sin q_3 + L_{ty} \cos q_3
\end{bmatrix} \tag{2.39}$$

Distributing the minus sign into the matrix:

$$\mathbf{J}_T^{-1}(q_3, d_4) = \begin{bmatrix}
-\frac{\sin q_3}{r_T} & \frac{\cos q_3}{r_T} \\[8pt]
\cos q_3 + \frac{L_{ty}}{r_T}\sin q_3 & \sin q_3 - \frac{L_{ty}}{r_T}\cos q_3
\end{bmatrix} \tag{2.40}$$

This explicit formula allows instantaneous calculation of required motor joint speeds $[\dot{q}_3, \dot{d}_4]^T = \mathbf{J}_T^{-1} [\dot{x}_T, \dot{y}_T]^T$ on a low-cost PLC without numerical matrix inversion!

---

### 2.8.7 Closed-Form Derivation of the Convective Term $\mathbf{\dot{J}}_T \mathbf{\dot{q}}$

To calculate accelerations and inertial forces, we take the time derivative of each element of $\mathbf{J}_T$:

$$\mathbf{\dot{J}}_T = \begin{bmatrix} \dot{J}_{11} & \dot{J}_{12} \\ \dot{J}_{21} & \dot{J}_{22} \end{bmatrix} \tag{2.41}$$

Recall:
- $\dot{r}_T = \dot{d}_4$
- $\frac{d}{dt}(\cos q_3) = -\dot{q}_3 \sin q_3$
- $\frac{d}{dt}(\sin q_3) = \dot{q}_3 \cos q_3$

#### Derivative of Column 1:
$$\dot{J}_{11} = \frac{d}{dt} (-r_T \sin q_3 + L_{ty} \cos q_3) = -\dot{d}_4 \sin q_3 - r_T \dot{q}_3 \cos q_3 - L_{ty} \dot{q}_3 \sin q_3 \tag{2.42a}$$

$$\dot{J}_{21} = \frac{d}{dt} (r_T \cos q_3 + L_{ty} \sin q_3) = \dot{d}_4 \cos q_3 - r_T \dot{q}_3 \sin q_3 + L_{ty} \dot{q}_3 \cos q_3 \tag{2.42b}$$

#### Derivative of Column 2:
$$\dot{J}_{12} = \frac{d}{dt} (\cos q_3) = -\dot{q}_3 \sin q_3 \tag{2.42c}$$

$$\dot{J}_{22} = \frac{d}{dt} (\sin q_3) = \dot{q}_3 \cos q_3 \tag{2.42d}$$

Now multiply $\mathbf{\dot{J}}_T$ by the joint velocity vector $\mathbf{\dot{q}} = [\dot{q}_3, \dot{d}_4]^T$:

$$\mathbf{a}_{\text{conv}} = \mathbf{\dot{J}}_T \mathbf{\dot{q}} = \begin{bmatrix} \dot{J}_{11} \dot{q}_3 + \dot{J}_{12} \dot{d}_4 \\ \dot{J}_{21} \dot{q}_3 + \dot{J}_{22} \dot{d}_4 \end{bmatrix} \tag{2.43}$$

Evaluating Row 1:
$$a_{\text{conv}, x} = \left(-\dot{d}_4 \sin q_3 - r_T \dot{q}_3 \cos q_3 - L_{ty} \dot{q}_3 \sin q_3\right) \dot{q}_3 + \left(-\dot{q}_3 \sin q_3\right) \dot{d}_4$$

$$a_{\text{conv}, x} = -2 \dot{q}_3 \dot{d}_4 \sin q_3 - \dot{q}_3^2 (r_T \cos q_3 + L_{ty} \sin q_3) \tag{2.44a}$$

Evaluating Row 2:
$$a_{\text{conv}, y} = \left(\dot{d}_4 \cos q_3 - r_T \dot{q}_3 \sin q_3 + L_{ty} \dot{q}_3 \cos q_3\right) \dot{q}_3 + \left(\dot{q}_3 \cos q_3\right) \dot{d}_4$$

$$a_{\text{conv}, y} = 2 \dot{q}_3 \dot{d}_4 \cos q_3 - \dot{q}_3^2 (r_T \sin q_3 - L_{ty} \cos q_3) \tag{2.44b}$$

> [!IMPORTANT]
> ### PHYSICAL IDENTIFICATION OF THE ACCELERATION FORCES
> Look closely at Equations (2.44a) and (2.44b):
> 1. The term containing $2 \dot{q}_3 \dot{d}_4$ is the **Coriolis Acceleration Vector**:
>    $$\mathbf{a}_{\text{Coriolis}} = 2 \dot{q}_3 \dot{d}_4 \begin{bmatrix} -\sin q_3 \\ \cos q_3 \end{bmatrix}$$
>    It acts perpendicular to the handle axis, pushing against the saddle block side liners whenever the crowd extends while the saddle is rotating!
> 2. The term containing $\dot{q}_3^2$ is the **Centripetal Acceleration Vector**:
>    $$\mathbf{a}_{\text{Centripetal}} = -\dot{q}_3^2 \begin{bmatrix} r_T \cos q_3 + L_{ty} \sin q_3 \\ r_T \sin q_3 - L_{ty} \cos q_3 \end{bmatrix}$$
>    It points directly inward toward the saddle pivot center $O_3$!

---

### 2.8.8 Transposed Jacobian & Digging Force Mapping

When the dipper teeth cut through hard rock, a large contact resistance force $\mathbf{F}_{\text{ext}} = [F_x, F_y]^T$ acts on tooth tip $T$. By Equation (2.22), the resulting generalized forces on the saddle and crowd actuators are:

$$\boldsymbol{\tau} = \begin{bmatrix} \tau_3 \\ F_{\text{crowd}} \end{bmatrix} = \mathbf{J}_T^T(q_3, d_4) \begin{bmatrix} F_x \\ F_y \end{bmatrix} \tag{2.45}$$

Substitute the transposed matrix:

$$\begin{bmatrix} \tau_3 \\ F_{\text{crowd}} \end{bmatrix} = \begin{bmatrix}
-r_T \sin q_3 + L_{ty} \cos q_3 & r_T \cos q_3 + L_{ty} \sin q_3 \\[6pt]
\cos q_3 & \sin q_3
\end{bmatrix} \begin{bmatrix} F_x \\ F_y \end{bmatrix} \tag{2.46}$$

Carrying out matrix multiplication:

$$\tau_3 = F_x (-r_T \sin q_3 + L_{ty} \cos q_3) + F_y (r_T \cos q_3 + L_{ty} \sin q_3) \tag{2.47a}$$

$$F_{\text{crowd}} = F_x \cos q_3 + F_y \sin q_3 \tag{2.47b}$$

> [!TIP]
> ### PHYSICAL INSIGHT ON CROWD FORCE EQUATION (2.47b)
> Look at the crowd force equation:
> 
> $$F_{\text{crowd}} = \begin{bmatrix} \cos q_3 & \sin q_3 \end{bmatrix} \begin{bmatrix} F_x \\ F_y \end{bmatrix} = \mathbf{F}_{\text{ext}} \cdot \mathbf{u}_{\text{handle}}$$
> 
> The crowd motor resists **precisely the component of the digging force that acts parallel to the dipper handle axis**! 
> The perpendicular component creates bending torque $\tau_3$ about the saddle block pivot, which is resisted by the hoist rope tension acting through the bail pin moment arm!

---

## 2.9 Five Fully Solved Research Practice Problems

---

### Problem 2.1: Numerical Jacobian & Tooth Velocity on P&H 2100BL
**Problem Statement:**  
On a P&H 2100BL excavator (Machine A), the saddle angle is $q_3 = 30^\circ$ ($\frac{\pi}{6}\,\mathrm{rad}$) and the crowd extension is $d_4 = 9.0\,\mathrm{m}$. The tooth offsets are $L_{tx} = 2.0\,\mathrm{m}$ and $L_{ty} = 1.0\,\mathrm{m}$.  
At this instant, the saddle block rotates upward at $\dot{q}_3 = 0.15\,\mathrm{rad/s}$ and the crowd extends outward at $\dot{d}_4 = 0.80\,\mathrm{m/s}$.
1. Calculate the effective radial tooth distance $r_T$.
2. Compute the numerical components of the Jacobian matrix $\mathbf{J}_T$.
3. Compute the instantaneous Cartesian velocity vector $[\dot{x}_T, \dot{y}_T]^T$ of the dipper tooth tip.
4. Calculate the speed magnitude $\|\mathbf{v}_T\|$.

**Step-by-Step Solution:**

**Step 1: Compute effective radial distance $r_T$**
$$r_T = d_4 + L_{tx} = 9.0\,\mathrm{m} + 2.0\,\mathrm{m} = 11.0\,\mathrm{m}$$

**Step 2: Evaluate trigonometric functions at $q_3 = 30^\circ$**
$$\cos 30^\circ = \frac{\sqrt{3}}{2} \approx 0.866025, \quad \sin 30^\circ = 0.500000$$

**Step 3: Calculate elements of $\mathbf{J}_T$ using Equation (2.33)**
$$J_{11} = -r_T \sin q_3 + L_{ty} \cos q_3 = -(11.0)(0.5) + (1.0)(0.866025) = -5.5 + 0.866025 = -4.633975\,\mathrm{m/rad}$$

$$J_{12} = \cos q_3 = 0.866025\,\mathrm{m/m}$$

$$J_{21} = r_T \cos q_3 + L_{ty} \sin q_3 = (11.0)(0.866025) + (1.0)(0.5) = 9.526279 + 0.5 = 10.026279\,\mathrm{m/rad}$$

$$J_{22} = \sin q_3 = 0.500000\,\mathrm{m/m}$$

Therefore:
$$\mathbf{J}_T = \begin{bmatrix} -4.6340 & 0.8660 \\ 10.0263 & 0.5000 \end{bmatrix}$$

**Step 4: Compute tooth velocity $\mathbf{v}_T = \mathbf{J}_T \mathbf{\dot{q}}$**
$$\begin{bmatrix} \dot{x}_T \\ \dot{y}_T \end{bmatrix} = \begin{bmatrix} -4.633975 & 0.866025 \\ 10.026279 & 0.500000 \end{bmatrix} \begin{bmatrix} 0.15 \\ 0.80 \end{bmatrix}$$

$$\dot{x}_T = (-4.633975)(0.15) + (0.866025)(0.80) = -0.695096 + 0.692820 = -0.002276\,\mathrm{m/s}$$

$$\dot{y}_T = (10.026279)(0.15) + (0.500000)(0.80) = 1.503942 + 0.400000 = +1.903942\,\mathrm{m/s}$$

$$\mathbf{v}_T \approx \begin{bmatrix} -0.0023 \\ +1.9039 \end{bmatrix}\,\mathrm{m/s}$$

**Step 5: Compute total tooth speed**
$$\|\mathbf{v}_T\| = \sqrt{(-0.002276)^2 + (1.903942)^2} \approx 1.9039\,\mathrm{m/s}$$

*(Physical Note: Notice that $\dot{x}_T \approx 0$. At this exact posture, the forward extension of the crowd rack is almost perfectly cancelled by the backward arc motion of the saddle rotation, causing the dipper teeth to cut almost purely vertically upward through the rock face!)*

---

### Problem 2.2: Digging Force Mapping on TZ WK-20 Heavy Shovel
**Problem Statement:**  
On a TZ WK-20 excavator (Machine B), the dipper is actively cutting blasted dolomite. Joint sensors record $q_3 = 15^\circ$ ($0.2618\,\mathrm{rad}$), $d_4 = 12.5\,\mathrm{m}$, with tooth geometry $L_{tx} = 2.5\,\mathrm{m}$, $L_{ty} = 1.2\,\mathrm{m}$.  
A 2D load cell array detects that the dolomite bank exerts an external digging resistance force on the teeth of:
$$\mathbf{F}_{\text{ext}} = \begin{bmatrix} F_x \\ F_y \end{bmatrix} = \begin{bmatrix} -450.0\,\mathrm{kN} \\ -680.0\,\mathrm{kN} \end{bmatrix}$$
*(where negative signs denote forces opposing forward and upward motion).*
1. Determine the effective radius $r_T$.
2. Compute the mechanical crowd push/pull force $F_{\text{crowd}}$ experienced by the crowd rack pinions.
3. Compute the torque $\tau_3$ that must be counterbalanced by hoist rope tension about pivot $O_3$.

**Step-by-Step Solution:**

**Step 1: Compute geometric values**
$$r_T = d_4 + L_{tx} = 12.5 + 2.5 = 15.0\,\mathrm{m}$$
$$\cos 15^\circ \approx 0.965926, \quad \sin 15^\circ \approx 0.258819$$

**Step 2: Compute crowd rack force using Equation (2.47b)**
$$F_{\text{crowd}} = F_x \cos q_3 + F_y \sin q_3$$
$$F_{\text{crowd}} = (-450\,\mathrm{kN})(0.965926) + (-680\,\mathrm{kN})(0.258819)$$
$$F_{\text{crowd}} = -434.667\,\mathrm{kN} - 175.997\,\mathrm{kN} = -610.664\,\mathrm{kN}$$

**Result:** The crowd motor rack is subjected to a massive compressive retarding force of **$610.7\,\mathrm{kN}$** pushing the dipper handle back into the machinery deck!

**Step 3: Compute saddle pivot torque $\tau_3$ using Equation (2.47a)**
First calculate the moment arm coefficients:
$$\text{Arm}_x = -r_T \sin q_3 + L_{ty} \cos q_3 = -(15.0)(0.258819) + (1.2)(0.965926) = -3.882285 + 1.159111 = -2.723174\,\mathrm{m}$$

$$\text{Arm}_y = r_T \cos q_3 + L_{ty} \sin q_3 = (15.0)(0.965926) + (1.2)(0.258819) = 14.488890 + 0.310583 = +14.799473\,\mathrm{m}$$

Now compute torque:
$$\tau_3 = F_x (\text{Arm}_x) + F_y (\text{Arm}_y)$$
$$\tau_3 = (-450\,\mathrm{kN})(-2.723174\,\mathrm{m}) + (-680\,\mathrm{kN})(+14.799473\,\mathrm{m})$$
$$\tau_3 = +1,225.428\,\mathrm{kN\cdot m} - 10,063.642\,\mathrm{kN\cdot m} = -8,838.214\,\mathrm{kN\cdot m}$$

**Result:** The external rock resistance induces a counter-clockwise bending moment of **$-8.838\,\mathrm{MN\cdot m}$** about the saddle pivot $O_3$, which must be held by the tension in the dual hoist ropes!

---

### Problem 2.3: Verification of Convective Acceleration Vectors
**Problem Statement:**  
Using the parameters of the P&H 2100BL from Problem 2.1 ($q_3 = 30^\circ$, $d_4 = 9.0\,\mathrm{m}$, $r_T = 11.0\,\mathrm{m}$, $L_{ty} = 1.0\,\mathrm{m}$, $\dot{q}_3 = 0.15\,\mathrm{rad/s}$, $\dot{d}_4 = 0.80\,\mathrm{m/s}$):
1. Calculate the numerical Coriolis acceleration vector $\mathbf{a}_{\text{Coriolis}}$.
2. Calculate the numerical Centripetal acceleration vector $\mathbf{a}_{\text{Centripetal}}$.
3. Compute the total convective acceleration vector $\mathbf{a}_{\text{conv}} = \mathbf{\dot{J}}_T \mathbf{\dot{q}}$.

**Step-by-Step Solution:**

**Step 1: Compute Coriolis acceleration**
From Section 2.8.7:
$$\mathbf{a}_{\text{Coriolis}} = 2 \dot{q}_3 \dot{d}_4 \begin{bmatrix} -\sin q_3 \\ \cos q_3 \end{bmatrix}$$
$$2 \dot{q}_3 \dot{d}_4 = 2 (0.15) (0.80) = 0.2400\,\mathrm{m/s^2}$$

$$\mathbf{a}_{\text{Coriolis}} = 0.2400 \begin{bmatrix} -0.5000 \\ 0.8660 \end{bmatrix} = \begin{bmatrix} -0.1200 \\ +0.2078 \end{bmatrix}\,\mathrm{m/s^2}$$

**Step 2: Compute Centripetal acceleration**
From Section 2.8.7:
$$\dot{q}_3^2 = (0.15)^2 = 0.0225\,\mathrm{rad^2/s^2}$$
$$r_T \cos q_3 + L_{ty} \sin q_3 = (11.0)(0.866025) + (1.0)(0.5) = 10.0263\,\mathrm{m}$$
$$r_T \sin q_3 - L_{ty} \cos q_3 = (11.0)(0.5) - (1.0)(0.866025) = 5.5 - 0.8660 = 4.6340\,\mathrm{m}$$

$$\mathbf{a}_{\text{Centripetal}} = -0.0225 \begin{bmatrix} 10.0263 \\ 4.6340 \end{bmatrix} = \begin{bmatrix} -0.2256 \\ -0.1043 \end{bmatrix}\,\mathrm{m/s^2}$$

**Step 3: Total Convective Acceleration $\mathbf{a}_{\text{conv}}$**
$$\mathbf{a}_{\text{conv}} = \mathbf{a}_{\text{Coriolis}} + \mathbf{a}_{\text{Centripetal}}$$
$$\mathbf{a}_{\text{conv}} = \begin{bmatrix} -0.1200 - 0.2256 \\ +0.2078 - 0.1043 \end{bmatrix} = \begin{bmatrix} -0.3456 \\ +0.1035 \end{bmatrix}\,\mathrm{m/s^2}$$

*(Physical Significance: If the dipper body has an effective mass of $30,000\,\mathrm{kg}$, this convective acceleration creates an unmodelled dynamic inertial force of $\mathbf{F} = m \mathbf{a}_{\text{conv}} = [-10.37\,\mathrm{kN}, +3.11\,\mathrm{kN}]^T$. Neglecting $\mathbf{\dot{J}}\mathbf{\dot{q}}$ during dynamic payload identification causes a systematic bias in payload estimation!)*

---

### Problem 2.4: Singular Value Decomposition & Condition Number
**Problem Statement:**  
Consider a simplified planar excavator Jacobian where tooth offsets are neglected ($L_{ty} = 0$), so:
$$\mathbf{J}_T = \begin{bmatrix} -r_T \sin q_3 & \cos q_3 \\ r_T \cos q_3 & \sin q_3 \end{bmatrix}$$
1. Compute the analytical matrix product $\mathbf{A} = \mathbf{J}_T \mathbf{J}_T^T$.
2. Determine the eigenvalues $\lambda_1, \lambda_2$ of $\mathbf{A}$.
3. Deduce the exact singular values $\sigma_1, \sigma_2$ of $\mathbf{J}_T$.
4. Express the condition number $\kappa(\mathbf{J}_T)$ as a function of radius $r_T$.
5. Evaluate $\kappa$ for $r_T = 11.0\,\mathrm{m}$.

**Step-by-Step Solution:**

**Step 1: Compute $\mathbf{A} = \mathbf{J}_T \mathbf{J}_T^T$**
$$\mathbf{J}_T \mathbf{J}_T^T = \begin{bmatrix} -r_T \sin q_3 & \cos q_3 \\ r_T \cos q_3 & \sin q_3 \end{bmatrix} \begin{bmatrix} -r_T \sin q_3 & r_T \cos q_3 \\ \cos q_3 & \sin q_3 \end{bmatrix}$$

$$A_{11} = (-r_T \sin q_3)^2 + (\cos q_3)^2 = r_T^2 \sin^2 q_3 + \cos^2 q_3$$

$$A_{12} = (-r_T \sin q_3)(r_T \cos q_3) + (\cos q_3)(\sin q_3) = (1 - r_T^2) \sin q_3 \cos q_3$$

$$A_{21} = A_{12} = (1 - r_T^2) \sin q_3 \cos q_3$$

$$A_{22} = (r_T \cos q_3)^2 + (\sin q_3)^2 = r_T^2 \cos^2 q_3 + \sin^2 q_3$$

**Step 2: Compute eigenvalues of $\mathbf{A}$**
The trace of $\mathbf{A}$ is:
$$\operatorname{tr}(\mathbf{A}) = A_{11} + A_{22} = r_T^2(\sin^2 q_3 + \cos^2 q_3) + (\sin^2 q_3 + \cos^2 q_3) = r_T^2 + 1$$

The determinant of $\mathbf{A}$ is:
$$\det(\mathbf{A}) = (\det(\mathbf{J}_T))^2 = (-r_T)^2 = r_T^2$$

The characteristic polynomial is:
$$\lambda^2 - \operatorname{tr}(\mathbf{A})\lambda + \det(\mathbf{A}) = \lambda^2 - (r_T^2 + 1)\lambda + r_T^2 = 0$$

Factoring directly:
$$(\lambda - r_T^2)(\lambda - 1) = 0 \implies \lambda_1 = r_T^2, \quad \lambda_2 = 1$$

**Step 3: Deduce singular values $\sigma_i = \sqrt{\lambda_i}$**
Since $r_T \ge 1.0\,\mathrm{m}$ on any physical mining shovel:
$$\sigma_1 = \sqrt{r_T^2} = r_T, \quad \sigma_2 = \sqrt{1} = 1.0$$

**Step 4: Compute Condition Number $\kappa$**
$$\kappa(\mathbf{J}_T) = \frac{\sigma_1}{\sigma_2} = \frac{r_T}{1.0} = r_T$$

**Step 5: Numerical evaluation at $r_T = 11.0\,\mathrm{m}$**
$$\kappa(\mathbf{J}_T) = 11.0$$

*(Pedagogical Insight: Because the joint coordinates have mixed physical units—$q_3$ in radians, $d_4$ in meters—the condition number equals the dimensioned radius $r_T$. To make the condition number dimensionless and meaningful for numerical algorithms, one must normalize the Jacobian by characteristic length scales, as we established in the Chapter 1 audit!)*

---

### Problem 2.5: Inverse Kinematic Velocity Tracking
**Problem Statement:**  
To smoothly clean up the pit floor, an autonomous dipper trajectory controller commands the dipper teeth to travel horizontally forward at constant speed:
$$\mathbf{v}_T^* = \begin{bmatrix} \dot{x}_T^* \\ \dot{y}_T^* \end{bmatrix} = \begin{bmatrix} +1.20\,\mathrm{m/s} \\ 0.00\,\mathrm{m/s} \end{bmatrix}$$
At the current instant, the shovel posture is $q_3 = -15^\circ$ (tilted downward into the trench), $d_4 = 10.0\,\mathrm{m}$, with tooth geometry $r_T = 12.0\,\mathrm{m}$, $L_{ty} = 1.0\,\mathrm{m}$.  
Using the closed-form inverse Jacobian from Equation (2.40), calculate the exact joint velocity commands $[\dot{q}_3^*, \dot{d}_4^*]^T$ that the hoist and crowd motor variable frequency drives (VFDs) must deliver.

**Step-by-Step Solution:**

**Step 1: Evaluate trigonometric terms at $q_3 = -15^\circ$**
$$\cos(-15^\circ) = \cos(15^\circ) \approx 0.965926$$
$$\sin(-15^\circ) = -\sin(15^\circ) \approx -0.258819$$

**Step 2: Evaluate Inverse Jacobian matrix from Equation (2.40)**
$$J_{11}^{-1} = -\frac{\sin q_3}{r_T} = -\frac{-0.258819}{12.0} = +0.021568\,\mathrm{rad/m}$$

$$J_{12}^{-1} = \frac{\cos q_3}{r_T} = \frac{0.965926}{12.0} = +0.080494\,\mathrm{rad/m}$$

$$J_{21}^{-1} = \cos q_3 + \frac{L_{ty}}{r_T}\sin q_3 = 0.965926 + \left(\frac{1.0}{12.0}\right)(-0.258819) = 0.965926 - 0.021568 = +0.944358\,\mathrm{m/m}$$

$$J_{22}^{-1} = \sin q_3 - \frac{L_{ty}}{r_T}\cos q_3 = -0.258819 - \left(\frac{1.0}{12.0}\right)(0.965926) = -0.258819 - 0.080494 = -0.339313\,\mathrm{m/m}$$

Therefore:
$$\mathbf{J}_T^{-1} = \begin{bmatrix} +0.021568 & +0.080494 \\ +0.944358 & -0.339313 \end{bmatrix}$$

**Step 3: Multiply by desired Cartesian velocity $\mathbf{v}_T^* = [1.20, 0]^T$**
$$\dot{q}_3^* = J_{11}^{-1} \dot{x}_T^* + J_{12}^{-1} \dot{y}_T^* = (0.021568)(1.20) + (0.080494)(0) = +0.025882\,\mathrm{rad/s}$$

$$\dot{d}_4^* = J_{21}^{-1} \dot{x}_T^* + J_{22}^{-1} \dot{y}_T^* = (0.944358)(1.20) + (-0.339313)(0) = +1.133230\,\mathrm{m/s}$$

**Final Result:**
- The hoist drive must rotate the saddle upward at **$\dot{q}_3^* = +0.0259\,\mathrm{rad/s}$** ($+1.48^\circ/\mathrm{s}$).
- The crowd drive must push the handle outward at **$\dot{d}_4^* = +1.133\,\mathrm{m/s}$**.

---

## 2.10 MATLAB / Python Implementation Code

```matlab
% =========================================================================
% CHAPTER 2: ELECTRIC ROPE SHOVEL KINEMATIC JACOBIAN & FORCE MAPPING
% Compatible with MATLAB R2025b / Octave 7+
% =========================================================================
function shovel_kinematics_demo()
    clc; clear; close all;
    
    % Machine A (P&H 2100BL) Benchmark Parameters
    params.L_tx = 2.0;    % Longitudinal tooth offset [m]
    params.L_ty = 1.0;    % Perpendicular tooth offset [m]
    
    % Posture state
    q3  = deg2rad(30);   % Saddle angle [rad]
    d4  = 9.0;           % Crowd extension [m]
    
    % Velocities
    q3_dot = 0.15;       % Saddle angular velocity [rad/s]
    d4_dot = 0.80;       % Crowd linear speed [m/s]
    
    q     = [q3; d4];
    q_dot = [q3_dot; d4_dot];
    
    % 1. Compute Jacobian
    J = compute_jacobian(q, params);
    fprintf('=== TOOTH TIP JACOBIAN J_T ===\n');
    disp(J);
    
    % 2. Compute Tooth Velocity
    v_T = J * q_dot;
    fprintf('=== TOOTH VELOCITY [m/s] ===\n');
    fprintf('v_x = %+.4f m/s\n', v_T(1));
    fprintf('v_y = %+.4f m/s\n', v_T(2));
    fprintf('Speed ||v_T|| = %.4f m/s\n\n', norm(v_T));
    
    % 3. Determinant & Singularity Check
    det_J = det(J);
    fprintf('Determinant det(J) = %.4f (Expected: -(d4+L_tx) = %.4f)\n\n', ...
            det_J, -(d4 + params.L_tx));
        
    % 4. Convective Acceleration J_dot * q_dot
    a_conv = compute_convective_acc(q, q_dot, params);
    fprintf('=== CONVECTIVE ACCELERATION [m/s^2] ===\n');
    fprintf('a_conv_x = %+.4f m/s^2\n', a_conv(1));
    fprintf('a_conv_y = %+.4f m/s^2\n\n', a_conv(2));
    
    % 5. External Digging Force Mapping
    F_ext = [-450e3; -680e3]; % [N] (Problem 2.2 style)
    tau = J' * F_ext;
    fprintf('=== GENERALIZED MOTOR FORCES ===\n');
    fprintf('Saddle Torque tau_3 = %+.2f kN*m\n', tau(1)/1e3);
    fprintf('Crowd Rack Force F_4 = %+.2f kN\n', tau(2)/1e3);
end

function J = compute_jacobian(q, p)
    q3 = q(1);
    d4 = q(2);
    r_T = d4 + p.L_tx;
    
    J = [-r_T*sin(q3) + p.L_ty*cos(q3),  cos(q3);
          r_T*cos(q3) + p.L_ty*sin(q3),  sin(q3)];
end

function a_conv = compute_convective_acc(q, q_dot, p)
    q3 = q(1);
    d4 = q(2);
    q3_dot = q_dot(1);
    d4_dot = q_dot(2);
    r_T = d4 + p.L_tx;
    
    % Coriolis term
    a_coriolis = 2 * q3_dot * d4_dot * [-sin(q3); cos(q3)];
    
    % Centripetal term
    a_centripetal = -(q3_dot^2) * [r_T*cos(q3) + p.L_ty*sin(q3);
                                   r_T*sin(q3) - p.L_ty*cos(q3)];
                               
    a_conv = a_coriolis + a_centripetal;
end
```

---

## 2.11 Research Supervisor Summary & Master Defense Checklist

### Core Concepts Mastered in Chapter 2

1. **Analytical vs. Geometric Jacobians:**  
   The Analytical Jacobian $\mathbf{J}_a = \frac{\partial \mathbf{x}}{\partial \mathbf{q}}$ directly differentiates coordinates, whereas the Geometric Jacobian $\mathbf{J}$ maps joint speeds to the physical twist vector $[\mathbf{v}_e^T, \boldsymbol{\omega}_e^T]^T$. In planar shovel kinematics, both yield identical translational results.

2. **Acceleration Dynamics & Convective Product:**  
   End-effector acceleration decomposes into $\mathbf{a} = \mathbf{J}\mathbf{\ddot{q}} + \mathbf{\dot{J}}\mathbf{\dot{q}}$. The convective term $\mathbf{\dot{J}}\mathbf{\dot{q}}$ accounts for centripetal and Coriolis accelerations. In dynamic load identification, omitting this term biases payload estimates by up to $15\%$.

3. **Virtual Work Force Duality:**  
   The transpose of the kinematic Jacobian maps external Cartesian forces directly onto the generalized actuator coordinates: $\boldsymbol{\tau} = \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}}$. The crowd motor directly resists the longitudinal axis component: $F_{\text{crowd}} = \mathbf{F}_{\text{ext}} \cdot \mathbf{u}_{\text{handle}}$.

4. **Singularity Immunity of the Shovel:**  
   The determinant $\det(\mathbf{J}_T) = -(d_4 + L_{tx})$ depends solely on the extension radius $r_T$ and never vanishes inside the physical crowd stroke range $[7.5\,\mathrm{m}, 14.5\,\mathrm{m}]$. The shovel mechanism is unconditionally singular-free inside its admissible workspace.

---

### Doctoral Viva & Defense Questions

- **Q1:** *Why does the transpose of the Jacobian appear in static force mapping instead of the inverse?*  
  **Answer:** By the Principle of Virtual Work, work is an invariant scalar: $\delta W = \boldsymbol{\tau}^T \delta \mathbf{q} = \mathbf{F}_{\text{ext}}^T \delta \mathbf{x}$. Substituting the kinematic relationship $\delta \mathbf{x} = \mathbf{J}\delta \mathbf{q}$ yields $\boldsymbol{\tau}^T \delta \mathbf{q} = \mathbf{F}_{\text{ext}}^T \mathbf{J}\delta \mathbf{q} = (\mathbf{J}^T \mathbf{F}_{\text{ext}})^T \delta \mathbf{q}$. Since $\delta \mathbf{q}$ is independent, $\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}_{\text{ext}}$.

- **Q2:** *Under high crowd speed, which convective component dominates: Coriolis or centripetal?*  
  **Answer:** Coriolis acceleration scales linearly with both crowd linear speed and saddle angular speed ($\propto 2\dot{q}_3 \dot{d}_4$), whereas centripetal scales with the square of angular speed ($\propto \dot{q}_3^2$). During aggressive crowd extension into the rock bank ($\dot{d}_4 \approx 1.0\,\mathrm{m/s}, \dot{q}_3 \approx 0.1\,\mathrm{rad/s}$), Coriolis acceleration dominates and induces heavy lateral contact forces on the saddle block side bronze bushings.

- **Q3:** *Why does the condition number of the shovel Jacobian vary with extension radius $r_T$?*  
  **Answer:** The columns of $\mathbf{J}_T$ have mixed units: column 1 has units of $\mathrm{m/rad}$ (moment arm) while column 2 has units of $\mathrm{m/m}$ (dimensionless projection). The singular values are $\sigma_1 = r_T$ and $\sigma_2 = 1.0$, giving condition number $\kappa = r_T$. To obtain an objective, dimensionally homogeneous condition number, column 1 must be normalized by a characteristic radius $R_0$.

---

### Transition to Chapter 3
With multivariable kinematics, Jacobians, and static force transmission firmly established, we next enter:  
**Volume I, Chapter 3: Variational Principles, Generalized Coordinates, and Euler-Lagrange Formulations**.  
In Chapter 3, we formulate the system kinetic energy $T(\mathbf{q}, \mathbf{\dot{q}})$ and potential energy $V(\mathbf{q})$, and derive the continuous nonlinear equations of motion for the electric rope shovel from Hamilton's Principle.
