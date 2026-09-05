# Claims Integrity Protocol

## Purpose

`claims.md` is the scientific claim ledger for the `shovel-dynamics`
project.

It is not a general notes file.

A claim may eventually appear in:

- the thesis;
- a research paper;
- a presentation;
- an experiment conclusion;
- a literature review;
- a design decision;
- a future research direction.

Therefore, changing a claim can change the scientific record.

This protocol prevents accidental strengthening, weakening, deletion,
duplication, or misattribution of scientific claims.

---

# 1. Core principle

> **Never change a scientific claim merely because it looks wrong,
> redundant, outdated, or inconvenient. First establish what the existing
> claim means, where it came from, and what evidence supports it.**

The correct workflow is:

**Source → Evidence → Existing claim → Git history → Proposed change →
Diff → Review → Commit**

---

# 2. Every claim must have provenance

For every important claim, identify where it came from.

Possible provenance includes:

- published paper;
- equation;
- table;
- figure;
- experimental artifact;
- MATLAB result;
- Simulink model;
- independent calculation;
- previous milestone;
- project design decision.

A claim without identifiable provenance should not silently be treated as
verified.

---

# 3. Distinguish source fact from project interpretation

A source may report one physical quantity while the project uses another
quantity derived from it.

These must remain separate.

For example:

### Source fact

A paper reports a crowd velocity and crowd acceleration.

### Project interpretation

The project may use those values as a reference for trajectory design.

### Incorrect transformation

The project must not relabel an unrelated source quantity as a machine
stroke or coordinate merely because the numerical range appears useful.

The original physical meaning must always be preserved.

---

# 4. Literature claims require exact verification

When a claim comes from a paper, verify:

1. paper identity;
2. machine/system;
3. section/table/equation/figure;
4. reported quantity;
5. units;
6. operating condition;
7. whether the value is measured, calculated, assumed, or optimised;
8. whether it applies to the same machine/model;
9. whether the project is using it as a fact, reference, or design choice.

Do not transfer a quantity from one physical definition to another without
explicit justification.

---

# 5. Numerical claims require artifact provenance

For a computational claim, identify:

- experiment identifier;
- code;
- model;
- parameter values;
- relevant trajectory;
- excitation;
- saved artifact;
- calculation used to obtain the reported value.

Where possible, a reader should be able to reproduce the claim from the
repository.

---

# 6. Claim strength must match evidence

A useful status vocabulary is:

| Status | Meaning |
|---|---|
| **V — verified** | Directly supported by checked evidence |
| **D — diagnostic** | A measured diagnostic result, not necessarily a validated scientific conclusion |
| **H — hypothesis** | A proposition being tested |
| **I — interpretation** | An interpretation of evidence |
| **C — corrected** | Previous wording was scientifically incorrect and has been corrected |
| **U — unverified** | Evidence has not yet been adequately checked |

Do not promote a claim from diagnostic to verified merely because the
result looks convincing.

---

# 7. Correlation claims require special caution

A correlation is evidence of statistical association between two quantities.

It does not automatically prove:

- causality;
- independence;
- physical equivalence;
- identifiability;
- model correctness.

When recording a correlation claim, identify:

- the two quantities;
- their definitions;
- preprocessing;
- whether means were removed;
- the correlation method;
- trajectory;
- experimental conditions;
- interpretation;
- limitation.

A statement such as "high correlation proves non-identifiability" is too
strong unless the experimental design and theoretical argument support
that conclusion.

---

# 8. Existing claims must be checked before editing

Before editing an existing claim:

### Step 1 — Locate it

Find the exact row or passage.

### Step 2 — Understand it

Determine exactly what it currently claims.

### Step 3 — Find its evidence

Locate the source, artifact, equation, or previous result.

### Step 4 — Check Git history

Determine whether the claim is already committed.

Use Git history to answer:

> Was this claim already intentionally recorded?

Do not infer the answer from the current working-tree diff alone.

---

# 9. Git history and working-tree state are different

A file can appear in:

```text
git diff