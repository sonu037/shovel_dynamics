# AI Collaboration Protocol

How to get useful work out of an AI assistant on this project, and how to catch
it when it is wrong. Written 2026-09-05 from observed failures, not from theory.

---

## PART 1 — Paste this at the start of every session

```
PROJECT: shovel-dynamics. Load-torque estimation for mining electric rope
shovels. IIT/SRIC/R/AEH/2026/104, IIT Kharagpur Mining Engineering.

REPO: github.com/sonu037/shovel_dynamics (public). If you have web or bash
access, CLONE IT AND READ THE FILES. Do not answer about my repository from
memory or from what I paste. If you cannot clone, say so plainly and ask me
to paste specific files.

AUTHORITY ORDER, highest first:
  1. claims.md            - which numbers are verified, and their status
  2. the evidence artifact (.mat, .slx, script source)
  3. research_log.md      - what happened and why
  4. the milestone specification
  5. conversation history  <- LOWEST. Never cite this as evidence.

BEFORE you tell me anything is "blocking", "unresolved", "missing" or "not
started": grep claims.md and read the actual file. Three items were
re-investigated on 2026-09-02 that were already closed there.

BEFORE you state a number: tell me the line of code or the file and line that
produced it. If you cannot, label it ESTIMATE every time you use it.

WHEN I ask for a command: give the pseudocode and the scientific reason first,
then the command, then what each outcome would mean. I need to defend this work.

WHEN you disagree with me: say so directly. Do not open with "excellent" or
"you're absolutely right" and then agree. I have three AI assistants on this
project and agreement between them is not corroboration.
```

---

## PART 2 — Standing rules for the assistant

**Verify, don't recall.** Everything about this repository must come from
reading the file. A summary of a previous session is not evidence.

**Distinguish estimate from measurement, every time.** "250:1" was an unmeasured
magnitude estimate. It was repeated through a physics explanation, an argument
about mechanism viability, and into a frozen specification before anyone
computed it. Measured value: 40.25:1.

**Name the failure mode, not just the fix.** A command that solves the problem
without explaining what class of problem it was teaches nothing and cannot be
generalised.

**Predict before running.** State the expected outcome and what each result
would mean, before the experiment. A prediction that survives is evidence the
mechanism is understood. A prediction that fails is more informative than no
prediction at all. Record both.

**Report the scope of a claim.** One trajectory, one frequency, one amplitude is
not a general result. Say what was tested and what was not.

**Own errors immediately and specifically.** Not "I may have been imprecise" but
"I said X, X is wrong, here is why, here is the correct statement."

---

## PART 3 — When to distrust the assistant

These are real failures from this project. Each was caught by running something.

| Failure | What happened | Caught by |
|---|---|---|
| Invented ratio | 250:1 stated as physics, repeated into a frozen spec | Measuring it: 40.25:1 |
| Metric artifact read as mechanism | corr = -0.41 taken as evidence of a separation mechanism; it was a mean offset. Mean-removed value +0.96 | Printing mean and std before interpreting |
| Stale belief propagated | "Izz is blocking" repeated 5 times; closed in claims.md a month earlier | Reading claims.md |
| Capability misreported | "I cannot access GitHub" said four times; the tools were available | Trying it |
| Source not read | "9.50-10.04 m is not in the paper"; it is, in Tables 6-7 | Reading Tables 6-7 |
| Wrong explanation offered confidently | narrow-stroke improvement attributed to Coriolis; the in/co ratio is invariant | Computing the ratio |
| Silent parameter override | six "different" amplitudes all ran at 100 kN | Noticing six identical rows |
| Second assistant invented a result | "kappa ~ 500" presented as a computed value; kappa had never been computed | Knowing what had been run |

**The pattern:** the assistant is most dangerous when it sounds most fluent.
Physics explanations, mechanism proposals and quoted ratios are where errors
that look like knowledge get generated. Where you run code and paste output,
reality has a vote.

**The counter:** compute rather than accept. Every correction above came from
running something, not from arguing.

---

## PART 4 — Red flags in an AI response

- Opens with praise, then agrees with whatever was said last
- States a number without a source line
- Says "as we established" about something not in a file
- Proposes a mechanism reasoned forward from equations, with no test
- Uses "clearly", "obviously", "of course" about something unmeasured
- Offers a fix without naming what class of problem it was
- Agrees with a correction instantly and completely, with no pushback

The last one matters. An assistant that never argues is agreeing with the most
recent speaker, not with the evidence.

---

## PART 5 — The three questions that catch most errors

Ask these of any claim, from any assistant, including yourself:

1. **What line of code produced this number?**
2. **What would have to be true for this to be wrong, and did we check?**
3. **Is this in claims.md, and with what status?**

---

## PART 6 — What actually worked

Recorded because it is reproducible, not because it is flattering.

- **Running the thing.** Every substantive correction this session came from
  executing code and comparing output against a stated expectation.
- **Pre-registered predictions.** MS02P03's criterion predicted that relative
  error would RISE at low amplitude for a benign reason; when it did, it was
  read correctly instead of hunted as a bug. MS02P04 had no criterion and the
  validity check became retrospective.
- **Refusing to proceed on an unexplained command.** Repeatedly asking "why this
  and not that" surfaced the incomplete s_dm derivation, the trim-window
  inheritance, and the omega_traj / omega_F conflation.
- **Insisting the source be read.** Bi et al.'s crowd limits turned out exact
  and the extension band turned out misattributed. Neither was knowable from
  discussion.

---

## PART 7 — Session close

Before ending any session:

```
git status --short          # must return nothing
git log --oneline -3        # confirm what actually went in
```

A result that exists only in a MATLAB workspace or a chat window is not a
result. Closing MATLAB destroyed an hour of analysis on 2026-08-28.

Write the log entry the same day. Include what was wrong, not only what worked —
the failed prediction in MS02P04 is more useful to a future reader than the
successful measurement.
