# Code Explainability Protocol

## Purpose

This protocol defines how research code must be written, explained,
reviewed, modified, and validated in the `shovel-dynamics` project.

Research code is not treated as ordinary programming.

Code is part of the scientific method because it implements the
mathematics, physical assumptions, experimental design, and numerical
procedure used to obtain a research result.

Therefore:

> **No scientifically important code should be a black box to the
> researcher using it.**

The researcher must understand what the code does well enough to explain
and defend it in a thesis, presentation, paper, or viva.

---

# 1. Code must come after the scientific question

Before writing substantial research code, state:

### WHAT?

What calculation, experiment, diagnostic, or analysis is being performed?

### WHY?

Why is it scientifically necessary?

### ACHIEVEMENT?

What specific result will the code produce that helps answer the research
question?

The code should implement a previously understood scientific procedure.

---

# 2. Required explanation before code

Before providing a substantial code block, explain:

1. the scientific objective;
2. the physical quantities involved;
3. the mathematical relationship being implemented;
4. the inputs;
5. the outputs;
6. the algorithm;
7. the validation method;
8. the expected result.

Preferred structure:

**Purpose → Definitions → Physics → Mathematics → Algorithm → Code →
Validation → Interpretation**

---

# 3. Define every symbol before using it

Whenever a new mathematical symbol, variable, abbreviation, parameter,
or notation is introduced:

### First define it.

Then explain:

### Physical meaning

What does it represent physically or mathematically?

Then provide:

### Equation

Only after the definition and meaning are clear.

Then:

### Calculation

Show how the code evaluates it.

Then:

### Interpretation

Explain what the calculated value means.

Do not introduce unexplained symbols merely because they are familiar
from another field.

---

# 4. Respect project nomenclature

Before creating a new variable name or symbol:

1. check the project nomenclature;
2. reuse an existing definition where appropriate;
3. determine whether the proposed name conflicts with an existing meaning;
4. explicitly document a new name when necessary.

A variable must not silently acquire a different physical meaning.

For example, if the project distinguishes:

- `omega_traj` — mechanism trajectory angular frequency;
- `omega_F` — external-force angular frequency;

then code must preserve that distinction.

A generic variable such as `omega` must not be introduced in a way that
makes the two frequencies indistinguishable.

See:

`M2/experiments/00_design/M2_nomenclature.md`

---

# 5. Explain scientifically consequential lines

Not every line of programming syntax requires a detailed explanation.

However, every line that can change a scientific result must be explainable.

Examples include:

- assigning model parameters;
- changing trajectory amplitudes;
- changing excitation frequencies;
- changing force magnitude;
- changing force direction;
- changing force application station;
- selecting signals;
- differentiating signals;
- filtering;
- trimming;
- normalising;
- calculating sensitivities;
- calculating correlations;
- fitting regressions;
- calculating error metrics;
- setting solver parameters;
- reading model-workspace variables;
- saving experimental results.

For each such operation explain:

**What it does → Why it is required → Scientific consequence**

---

# 6. Never hide parameter changes inside code

If code changes an experimental parameter, identify it explicitly.

Example:

```matlab
mw.assignin('omega_traj', omega_value);