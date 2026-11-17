# Research Integrity and Physical Modeling Rules (Shovel Dynamics)

## 1. Primary Source Authority (The OEM Anchor Rule)
- The file `reference/3. AMLOHRI WK-20 TZ ROPE SHOVEL - Technical Specifications (1).docx` is the sole ground truth for the Taiyuan WK-20 rope shovel.
- Authoritative OEM values:
  - Working Weight: 779 t (1321 mm shoe) / 792 t (1778 mm shoe)
  - Nominal Payload: 36.0 tonnes (20 m³ dipper)
  - Boom Length / Angle: 15.50 m at 45°
  - Effective Handle Length: 9.70 m
  - Max Hoisting Force: 1540 kN
  - Max Crowding Force: 745 kN (never 755 kN)
  - Max Hoist Speed: 1.58 m/s; Max Crowd Speed: 0.54 m/s
- All unlisted parameters (dipper mass $M_d$, inertia $I_{zz}$, gear ratios $G_h, G_c$, rope diameter) MUST be explicitly labeled `[EST]` or `[Lit: Citation]`. Never label computed or estimated values as `[V]` or `[OEM]`.

## 2. Anti-Inverse-Crime Invariant
- An observer or parameter identification algorithm must NEVER be validated solely against its own generating equations.
- Validation plants must belong to a different model class or higher fidelity (e.g., Simscape Multibody with transmission compliance, actuator current loop delay, 14-bit resolver quantization, and $\pm 5\text{--}10\%$ parameter mismatch).

## 3. Identifiability & Fair Comparison Rule
- Cross-channel condition numbers ($\kappa$) and subspace angles ($\theta_{\min}$) must be evaluated under like-for-like excitation trajectories. Never compare metric A from trajectory 1 against metric B from trajectory 2.
- Channel weighting across mixed units (e.g. $\mathrm{N\cdot m}$ vs $\mathrm{N}$) must use inverse noise covariance (Fisher Information Matrix), not arbitrary scalar scalings like $\bar{s} = 10\,\mathrm{m}$.
- Always report configuration-dependent singularities (e.g. $\det = 0$ at $q_3 = 0^\circ$).

## 4. Read-Only Verification Invariant
- Audit and verification scripts must be strictly read-only.
- Never use `save_system`, file writing, or string replacement (`strrep`) inside an audit test to force a "PASS".
- If an assertion fails, the script must throw an error and exit with a non-zero exit code.

## 5. Strict Provenance & Zero Post-Hoc Speculation
- If the origin of a number, trace, or parameter is unknown, state "origin unknown / unverified".
- Never invent post-hoc physical justifications or instrument claims.
