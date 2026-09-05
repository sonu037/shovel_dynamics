# Experiment Defensibility Protocol

## Purpose

This protocol defines the standard for designing, executing, validating,
interpreting, documenting, and defending experiments in the
`shovel-dynamics` research project.

An experiment is not complete merely because:

- the code executes;
- Simulink runs;
- a figure is produced;
- a correlation is calculated;
- a regression returns a value;
- an expected trend appears.

An experiment is research-complete only when its scientific reasoning,
implementation, evidence, interpretation, and limitations are understood.

The final test is:

> **Can the researcher defend this experiment to a thesis examiner?**

---

# 1. The experiment must begin with a scientific question

Before designing the experiment, write the question in one sentence.

The question should identify:

- what physical or mathematical issue is being investigated;
- what uncertainty exists;
- what quantity is being changed;
- what quantity is being observed.

Avoid starting with:

> "I want to run a frequency sweep."

Instead state the scientific purpose first.

For example:

> "Does changing the mechanism trajectory frequency increase the
> distinguishability between the external-load sensitivity and the
> mass-error sensitivity?"

The frequency sweep is then the experimental method, not the research
question.

---

# 2. State the hypothesis or prediction

Before running the experiment, state what is expected and why.

A prediction should be based on:

- governing equations;
- mechanics;
- dimensional reasoning;
- previous experimental evidence;
- literature;
- or a clearly identified hypothesis.

Separate:

**known → predicted → unknown**

Do not create a prediction after seeing the results.

---

# 3. Define every quantity before using it

Every mathematical symbol, variable, parameter, abbreviation, coordinate,
or metric must be defined before it appears in an equation or important
argument.

Use:

**Definition → physical meaning → equation → calculation →
interpretation**

Include:

- units;
- coordinate convention;
- sign convention;
- reference frame where relevant;
- whether the quantity is measured, injected, prescribed, or derived.

Use the project's existing nomenclature whenever possible.

---

# 4. Establish the physical model

Before the experiment, identify:

- system being modelled;
- degrees of freedom;
- coordinates;
- masses;
- inertias;
- forces;
- torques;
- geometry;
- friction;
- gravity;
- external excitation;
- measured quantities.

State which parameters are:

- frozen from previous milestones;
- deliberately varied;
- experimentally prescribed;
- identified;
- assumed.

A parameter must not change accidentally between experiments.

---

# 5. Establish the mathematical model

Document the governing equations required for the experiment.

For each important equation:

1. define every symbol;
2. explain the physical meaning;
3. state the assumptions;
4. derive or cite the equation;
5. explain how it is implemented numerically.

Do not present a final equation without understanding where it came from.

---

# 6. Identify independent, dependent, and controlled quantities

Every experiment should explicitly identify:

### Independent quantity

The quantity deliberately varied.

### Dependent quantity

The quantity measured or calculated in response.

### Controlled quantities

Quantities intentionally held constant.

This prevents accidental confounding.

For example, if trajectory frequency is varied while external-force
frequency is intended to remain fixed, that distinction must be explicit.

---

# 7. Define the experimental matrix before execution

For a sweep or matrix experiment, document the complete design before
running it.

Record:

- cases;
- independent-variable values;
- controlled parameters;
- trajectory;
- excitation;
- sampling;
- solver;
- output metrics.

Do not choose only the successful cases after seeing preliminary results
without documenting that change.

If the design changes, record why.

---

# 8. Establish the baseline

Before a new experiment, run or identify a baseline case.

The baseline provides:

- a reference for comparison;
- evidence that the model still behaves as expected;
- a way to identify accidental model changes.

Record:

- baseline parameters;
- expected result;
- observed result;
- acceptance criteria.

A new experiment should not be interpreted without knowing whether the
baseline remains valid.

---

# 9. Separate mechanism motion from excitation

When a mechanism has prescribed motion and an external excitation, treat
them as separate experimental inputs unless deliberately coupled.

For M2:

- `omega_traj` is the mechanism trajectory angular frequency;
- `omega_F` is the external-force angular frequency.

They are independent quantities.

If they are set to the same numerical value in an experiment, state that
as a design choice.

Do not infer that they are physically the same quantity.

---

# 10. Verify the actual model state

The parameter requested by the experiment is not automatically the
parameter used by the simulation.

Where the simulation environment permits workspace or block-level
overrides:

1. assign the parameter;
2. read it back;
3. verify it;
4. only then run the experiment.

This is particularly important for Simulink Model Workspace variables.

The experiment receipt should record the verified value where practical.

---

# 11. Validate the trajectory

Before interpreting an experiment, verify that the mechanism actually
followed the intended trajectory.

Check, as appropriate:

- amplitude;
- bias;
- frequency;
- velocity;
- acceleration;
- initial condition;
- sampling.

Distinguish:

**requested trajectory → actual trajectory**

If they differ materially, the experiment may not represent the intended
design.

---

# 12. Validate the excitation

Similarly verify:

- force amplitude;
- force direction;
- force frequency;
- force application station;
- timing;
- signal routing.

Do not assume that a parameter assignment proves that the intended force
was actually applied.

Where possible, compare requested and achieved excitation.

---

# 13. Keep source facts separate from machine applicability

A value reported by a paper is not automatically a limit for this
project's machine.

For literature-derived experimental references, record:

- source machine;
- source operating condition;
- source quantity;
- units;
- exact location in the source;
- reason for using it;
- whether it is a reference, constraint, or design inspiration.

Never convert a source reference into a machine specification without
evidence.

---

# 14. Validate the numerical implementation

Before interpreting physical results, establish that the computation itself
works.

Possible tests include:

- known-input recovery;
- analytical comparison;
- zero-input case;
- limiting case;
- dimensional check;
- independent calculation;
- regression test;
- numerical precision check.

A successful simulation run is not numerical validation.

---

# 15. Define acceptance criteria before execution

An experiment should have explicit criteria for:

- valid execution;
- expected signal behaviour;
- recovery;
- numerical accuracy;
- parameter read-back;
- trajectory fidelity;
- required diagnostic thresholds where justified.

Acceptance criteria should not be invented after seeing the result.

If no meaningful threshold can be justified, state that the metric is
diagnostic rather than an acceptance criterion.

---

# 16. Save an experiment receipt

Every important experiment should produce a machine-readable or otherwise
auditable receipt.

Where practical, preserve:

- experiment identifier;
- date;
- code version;
- model version;
- parameter values;
- trajectory settings;
- excitation settings;
- sampling;
- solver;
- calculated metrics;
- validation results;
- acceptance results.

The receipt should make it possible to answer:

> "Exactly how was this number produced?"

---

# 17. Never rely on memory for experimental parameters

A researcher should not need to remember:

> "I think the force was 100 kN."

The artifact or experiment record should establish it.

Likewise for:

- frequency;
- amplitude;
- station;
- trim;
- sampling;
- solver;
- trajectory bias;
- trajectory amplitude.

Important parameters belong in the experimental record.

---

# 18. Analyse the result at multiple levels

Do not rely on one metric.

Where appropriate, examine:

### Physical response

What happened to the mechanism?

### Sensitivity

How did the relevant physical causes affect the measured quantity?

### Statistical relationship

How similar or different are the signatures?

### Recovery

Can the known injected quantity be recovered?

### Numerical quality

Is the calculation stable and sufficiently precise?

Different metrics answer different questions.

---

# 19. Correlation is a diagnostic, not automatically a conclusion

If a correlation is used, state:

- the two quantities;
- their definitions;
- preprocessing;
- trajectory;
- conditions;
- correlation method;
- interpretation.

A high correlation means the shapes are similar under the tested
conditions.

A low correlation means the shapes are less similar.

Neither automatically proves parameter identifiability.

---

# 20. Investigate unexpected results systematically

When a result is surprising, use the following order where appropriate:

### Level 1 — Input

Were the requested parameters actually applied?

### Level 2 — Model

Was the intended model configuration used?

### Level 3 — Signal

Were the correct signals measured and routed?

### Level 4 — Numerical method

Could sampling, differentiation, filtering, solver behaviour, or precision
explain the result?

### Level 5 — Experimental design

Could the trajectory or excitation have produced the observed behaviour?

### Level 6 — Physics

Only after the above checks should a new physical explanation be
advanced.

This prevents numerical artefacts from being incorrectly reported as
physical discoveries.

---

# 21. Interpret the result against the prediction

After running the experiment:

### Prediction

What did we expect?

### Observation

What did we actually obtain?

### Comparison

Did they agree?

### Explanation

Why?

### Scientific implication

What does the result tell us?

### Limitation

What remains unresolved?

Do not skip directly from observation to conclusion.

---

# 22. Scope every conclusion

A conclusion should identify the conditions under which it was established.

Prefer:

> "Under the tested common-frequency excitation and trajectory family,
> the frequency sweep did not provide sufficient separation."

over:

> "Frequency cannot improve identifiability."

The second statement is much broader and requires substantially stronger
evidence.

---

# 23. Distinguish failure of an experiment from failure of a hypothesis

If an experiment does not produce the desired result, distinguish:

### Experimental failure

The implementation did not execute correctly.

### Methodological limitation

The experiment executed correctly but could not answer the question.

### Hypothesis failure

The experiment validly contradicted the prediction.

These are scientifically different outcomes.

A failed experiment can still be a valuable research result.

---

# 24. Do not rerun without a scientific reason

A rerun is justified when:

- the model was wrong;
- a parameter was incorrectly applied;
- the implementation contained an error;
- the experimental design changed;
- the result cannot be trusted;
- an additional control is scientifically required.

A rerun is not justified merely because the result is inconvenient.

If the result is valid but unexpected, preserve it and explain it.

---

# 25. Preserve negative results

A valid experiment that fails to support the hypothesis is evidence.

Do not delete it because:

> "It didn't work."

Document:

- what was tested;
- why;
- what happened;
- why it matters;
- what it rules out;
- what remains possible.

Negative results can determine the next experiment.

---

# 26. Maintain comparison integrity

When comparing two experiments, verify that the following are compatible:

- model;
- parameters;
- coordinate definitions;
- trajectory;
- force;
- sampling;
- solver;
- preprocessing;
- metric definitions.

If something differs intentionally, state it.

If something differs unintentionally, resolve it before drawing conclusions.

---

# 27. Every important figure must be explainable

For every thesis-quality figure, be able to explain:

- x-axis;
- y-axis;
- units;
- signals;
- preprocessing;
- parameter values;
- experiment identifier;
- why the figure exists;
- what pattern is visible;
- what the pattern means;
- what it does not prove.

A figure should answer a scientific question rather than merely look
interesting.

---

# 28. Every table must preserve provenance

For each important table:

- identify the source;
- define the quantities;
- define units;
- identify experimental conditions;
- identify whether values are measured, calculated, or sourced;
- preserve sufficient precision;
- avoid false precision.

Do not report more precision than the underlying evidence justifies.

---

# 29. Thesis-ready result structure

A major experiment should eventually be documentable as:

## Research question

What are we trying to learn?

## Background

Why does the question matter?

## Hypothesis

What do we predict?

## Model

What physical and mathematical model is used?

## Experimental design

What is varied and what is controlled?

## Implementation

How is the experiment executed?

## Validation

How do we know it was executed correctly?

## Results

What happened?

## Analysis

What do the metrics show?

## Interpretation

What does it mean physically?

## Limitations

What does it not establish?

## Conclusion

What is the defensible scientific conclusion?

## Next experiment

What uncertainty remains?

---

# 30. Viva-defence test

For every major experiment, the researcher should be able to answer the
following without relying on the code as a black box.

### Why did you do this experiment?

### Why did you choose these parameter values?

### Why did you hold those variables constant?

### What equations govern the experiment?

### What does each important symbol mean?

### What assumptions did you make?

### How did you implement the equations?

### How did you verify that the model used the intended parameters?

### How did you validate the numerical implementation?

### How do you know the result is not a numerical artefact?

### What does the result establish?

### What does it not establish?

### What alternative explanation remains?

### What experiment would distinguish the remaining possibilities?

If these questions cannot be answered, the experiment is not yet fully
defensible.

---

# 31. Final completion criterion

An experiment is complete only when:

**Question defined  
+ hypothesis/prediction stated  
+ variables defined  
+ physics understood  
+ mathematics understood  
+ design documented  
+ model state verified  
+ trajectory verified  
+ excitation verified  
+ numerical implementation validated  
+ evidence saved  
+ results analysed  
+ interpretation scoped  
+ limitations documented  
+ thesis/viva defence possible**

---

# Final principle

> **The objective of an experiment is not merely to generate a number.**

The objective is to generate a number whose:

**origin → mathematics → physical meaning → validation → limitation →
scientific conclusion**

can all be explained and defended.

That is the standard for thesis-level computational research.