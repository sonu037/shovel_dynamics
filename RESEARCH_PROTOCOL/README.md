# Research Protocol

## Purpose

This folder contains the mandatory research-workflow protocols for the
`shovel-dynamics` project.

The purpose is not simply to produce code that runs.

The purpose is to ensure that every important computational result is:

1. scientifically motivated,
2. mathematically understandable,
3. physically interpretable,
4. reproducible,
5. traceable to its evidence,
6. auditable in Git, and
7. defensible in a PhD thesis and viva.

These protocols apply to M1, M2, MS02Pxx experiments, future milestones,
mechanics derivations, numerical analyses, identification studies, and
research code.

---

# Master Research Workflow

Every substantial research task should follow this sequence:

**Question
→ Motivation
→ Physical model
→ Definitions
→ Mathematics
→ Experimental/design choices
→ Algorithm
→ Code
→ Validation
→ Evidence
→ Interpretation
→ Limitations
→ Thesis/Viva defence**

Code should not be written before the scientific purpose and intended
calculation are understood.

---

# 1. WHAT are we doing?

Before writing or modifying research code, state:

- the scientific question;
- the experiment or analysis;
- the milestone and part identifier;
- the input signals, parameters, or data;
- the intended output;
- the hypothesis or claim being tested.

The task must be understandable without reading the code.

---

# 2. WHY are we doing it?

Explain:

- why the calculation is scientifically necessary;
- what physical or mathematical reasoning motivates it;
- which previous result led to it;
- what uncertainty or question it addresses;
- what alternative explanations are possible;
- what could go wrong if the calculation is omitted.

The reason for an experiment must not be inferred only after seeing
the result.

---

# 3. DEFINE before using

Every newly introduced mathematical symbol, variable, abbreviation,
parameter, metric, coordinate, or notation must be defined before it is
used.

Preferred order:

**Definition → physical meaning → equation → calculation → interpretation**

Do not introduce unexplained notation in equations, figures, code comments,
tables, or research prose.

Definitions must use the project's established nomenclature where one
already exists.

---

# 4. Separate scientific facts from choices and interpretations

Every important statement should be understood as one of four categories:

### A. Physical/source fact

Something directly supported by:

- a paper,
- a machine specification,
- a measurement,
- an equation,
- a verified model property, or
- another identified source.

### B. Experimental/design choice

Something deliberately chosen by this project.

Examples include:

- trajectory amplitude;
- excitation frequency;
- force amplitude;
- station;
- sampling interval;
- solver setting;
- trim window.

A design choice must never be presented as a source fact.

### C. Numerical result

A value actually produced by the experiment or computation.

It must be traceable to the relevant code, model, and saved artifact.

### D. Interpretation/inference

A conclusion drawn from the evidence.

Interpretation must not be presented as though it were directly measured
or stated by the source.

---

# 5. Research code must be explainable

For every scientifically important section of code, explain:

- what it does;
- why it is required;
- what physical or mathematical quantity it represents;
- which assumption it implements;
- what would change if it were changed.

The code must not become a black box.

Simple programming syntax does not require unnecessary explanation, but
scientifically consequential operations do.

---

# 6. Never silently change scientific assumptions

If a modification changes any of the following, explicitly report it:

- parameter value;
- coordinate definition;
- datum;
- force location;
- trajectory;
- amplitude;
- frequency;
- initial condition;
- sampling;
- filtering;
- trimming;
- solver configuration;
- normalization;
- regression definition;
- acceptance criterion;
- model equation.

For every such change state:

**WHAT changed → WHY → expected effect → whether comparison with
previous results remains valid**

---

# 7. Experimental design must precede execution

Before running a substantial experiment, record:

- research question;
- hypothesis/prediction;
- independent variable;
- dependent variables;
- controlled variables;
- parameter values;
- trajectory definition;
- excitation definition;
- model version;
- expected outcome;
- acceptance criteria.

If the experiment is exploratory, explicitly say so.

Do not create a post-hoc hypothesis simply because a result looks
interesting.

---

# 8. Validation must be independent where possible

A result should not be considered trustworthy merely because the code
executes without error.

Validation may include:

- analytical derivation;
- dimensional analysis;
- limiting-case checks;
- independent calculation;
- known reference value;
- model read-back;
- conservation/physical consistency;
- regression/recovery test;
- comparison against a saved artifact;
- repeated calculation using an independent implementation.

The validation method must itself be documented.

---

# 9. Preserve an evidence trail

Important numerical results should be traceable through:

**Source/code → parameters → model → execution → saved artifact →
diagnostic → claim**

Whenever practical, experiment receipts should preserve:

- experiment identifier;
- date;
- relevant model parameters;
- trajectory settings;
- force settings;
- sampling information;
- measured signals;
- calculated metrics;
- validation checks;
- acceptance results.

A number in a thesis should be recoverable from project evidence.

---

# 10. Interpret before declaring success

After execution, follow:

**Prediction → Result → Comparison → Interpretation → Limitation →
Conclusion**

Ask:

1. Did the result agree with the prediction?
2. If not, what changed?
3. Does the result support the hypothesis?
4. Does it establish the claim?
5. Does it only provide evidence consistent with the claim?
6. Does it contradict the claim?
7. What alternative explanation remains possible?

Do not convert a convenient numerical result into a stronger scientific
claim than the evidence supports.

---

# 11. Distinguish numerical failure from physical failure

If a result changes unexpectedly, do not immediately attribute it to
physics.

First investigate possible causes such as:

- solver behaviour;
- sampling;
- numerical precision;
- differentiation;
- filtering;
- model configuration;
- parameter read-back;
- initialization;
- signal routing;
- workspace variables;
- experimental implementation.

Only after implementation and numerical causes have been examined should
a physical explanation be advanced.

---

# 12. Literature-derived quantities require provenance

Whenever a value from a paper is used, record:

- the exact source;
- table/equation/figure where applicable;
- what the quantity actually represents;
- units;
- machine/system to which it applies;
- whether it is directly transferable to this project;
- whether this project is using it as a fact, reference, or design choice.

A numerical range from a paper must not be relabelled as a different
physical quantity merely because its numerical values are convenient.

---

# 13. Claims require evidence

A claim must have an identifiable status.

Recommended statuses include:

- **V — verified**
- **D — diagnostic**
- **H — hypothesis**
- **I — interpretation**
- **C — corrected**
- **U — unverified**

The status must match the strength of the evidence.

A diagnostic correlation, for example, must not automatically become a
validated identifiability claim.

---

# 14. Git is part of the research audit trail

Git history is not merely software-development history.

It records the evolution of the scientific work.

Before changing an existing scientific claim or result:

1. identify the existing claim;
2. identify its evidence;
3. check relevant Git history;
4. determine whether it is already committed;
5. make the smallest justified change;
6. inspect the working diff;
7. stage deliberately;
8. inspect the staged diff;
9. commit with a scientifically meaningful message;
10. push only when explicitly authorized.

Never infer that work is uncommitted solely because it appears in a
working-tree diff.

---

# 15. Claims Integrity

Scientific claims receive additional protection.

Before deleting, merging, rewriting, or reinterpreting an existing claim:

**Source → Evidence → Existing claim → Git history → Proposed change →
Diff → Review → Commit**

Do not "clean up" claims based only on appearance.

See:

`CLAIMS_INTEGRITY_PROTOCOL.md`

---

# 16. Experiment Defensibility

Every major experiment must ultimately answer:

### Scientific question
Why was this experiment necessary?

### Hypothesis
What did we predict?

### Method
How was the experiment performed?

### Mathematics
What equations describe the calculation?

### Implementation
How were those equations implemented?

### Validation
How do we know the implementation is correct?

### Result
What was actually observed?

### Interpretation
What does the result mean physically?

### Limitation
What does the result NOT establish?

### Next step
What uncertainty remains?

See:

`EXPERIMENT_DEFENSIBILITY_PROTOCOL.md`

---

# 17. Code Explainability

Research code must be understandable to a researcher who did not write
the original line.

The code should allow the author to answer:

> "Why is this line here?"

for every scientifically consequential operation.

See:

`CODE_EXPLAINABILITY_PROTOCOL.md`

---

# 18. Thesis/Viva Standard

The final standard for substantial research work is:

> **If the result is important enough to appear in the thesis, the
> researcher must understand and be able to defend the reasoning that
> produced it.**

For every major result, the project should be able to explain:

**Why → What → How → Equation → Implementation → Validation →
Result → Meaning → Limitation**

The objective is not merely reproducibility.

The objective is **scientific understanding plus reproducibility**.

---

# 19. Default rule

When uncertain whether an explanation, validation, provenance check, or
audit step is necessary:

**prefer the more defensible research workflow.**

Additional explanation is acceptable.

An unexplained scientific assumption is not.

---

## Related protocols

- `CODE_EXPLAINABILITY_PROTOCOL.md`
- `CLAIMS_INTEGRITY_PROTOCOL.md`
- `EXPERIMENT_DEFENSIBILITY_PROTOCOL.md`

These documents are complementary.

This README defines the overall workflow; the individual protocols define
the detailed requirements for their respective areas.