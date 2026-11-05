---
name: hand-mano-eval
description: Compare dexterous robot hands with MANO and audit unexpected rankings through verified correspondences, shape and scale fitting, frozen calibration, and constrained pose IK. Use for URDF-to-human morphology or retargeting geometry, with separate evidence for hardware performance.
---

# Hand–MANO evaluation

Deliver a reproducible geometric comparison and explain what determines its ordering. Correct FK arithmetic does not establish anatomical correspondence or benchmark fairness.

Read [the method](references/method.md) before choosing metrics and [the audited example](references/audit-example.md) when investigating rank changes.

## Establish the comparison

- Fix exact model/SKU/revision, side, units, URDF/mesh hashes and official/derivative status. An older public model cannot answer a newer SKU question.
- Resolve every visual within its exact URDF/package tree. Never search other models by basename: different models often reuse filenames. A transform audit against a wrong asset manifest cannot validate asset identity. Compare resolved paths and hashes with the declared source before inspecting overlays.
- A revolute origin is arbitrary along its rotation axis. Check its location against the actual hinge/finger midplane; a side-mounted origin must not be scored as an anatomical centre. Any reviewed correction stays on the same physical axis and has an explicit offset and uncertainty. Inspect the full physical chain before mapping MCP/PIP/DIP or thumb CMC/MCP/IP. Co-located flexion/abduction axes are one centre; a displaced axis/link origin is not automatically an anatomical centre. Record rationale and confidence for each mapping.
- Distinguish physical URDF, baked calibration and virtual representation joints. Respect encoded limits/mimic; record missing tendon or closed-chain constraints.
- Complete the five fingertips before calling a result a full-digit comparison. Use source endpoint frames only after checking them against actual terminal geometry, or record reviewed link-local surface endpoint offsets. A distal axis origin or tactile sensor frame is not automatically a tip. Report feature count and landmark coverage separately; 20 distances do not mean 20 points.
- Distinguish 20 finger points from 21 points including a wrist. Record a true wrist mapping only when supported. A proximal-palm attachment proxy may be inspected separately, but cannot silently become an anatomical wrist or enter the common hand score.
- Decide whether the question concerns size, proportions, fitted position, movement changes, endpoints or actual tasks. Report these separately; no arbitrary overall winner.

## Compute and audit

1. Extract identical static features. Fit one whole-hand scale per candidate beta, then select beta by a declared objective. Distinguish discrete matching from continuous fitting; record MANO convention and priors.
2. Report physical-size and shape-only residuals, scale and per-finger contributions. Equal item weights do not imply anatomical balance: feature magnitudes affect the scale fit and normalization denominator; actual error contributions require residual SSE. Pairwise base distances are dependent and base spacing may depend on reference q. For bone-only, thumb/base or terminal-length sensitivity, state whether beta/alpha are refitted or held fixed; compare percentages only within the same feature definition.
3. Select or fit beta/scale by the declared shape protocol and freeze them before calibrating physical reference angles q0 and proper R,t on calibration targets excluded from evaluation. Distinguish static q_ref, fitted q0 and a verified natural open hand. For held-out poses optimize only the selected feasible physical q; save optimized/frozen variables, targets, predictions and status. Do not align each pose again.
   Use the complete declared point set in both calibration and evaluation. Adding tips only after a 15-centre calibration leaves a reference bias. If reference IK folds a thumb to imitate flat MANO, inspect that result rather than claiming a natural open hand. For a reference illustration, keep robot q_ref fixed and fit a bounded MANO pose under a separately labelled diagnostic registration; never reuse that diagnostic pose or score as held-out robot IK evidence.
4. Show reference-position RMS, absolute multi-pose RMS and movement-increment RMS together. Declare whether increment scores reuse absolute-objective q or come from a separate optimization. A reference residual is not a lower bound for every pose. Never subtract two RMS values as a decomposition or infer motion quality from their rank correlation.
5. Separate internal-centre and fingertip scores. An internal joint centre cannot observe its own terminal rotation. State whether tips came from joint+tip or tip-only IK.
6. Compute each pose's RMS of 3D point distances, then mean/P95 across poses; pose P95 is not a percentile of pooled point errors. Report predeclared threshold attainment and per-family or equal-family statistics. Label adapted-beta/common-beta conditions on every result table. Counts weight gesture families. Synthetic targets establish synthetic-test attainment, not human coverage or temporal dynamics.
   Report all 20 finger points, internal15, tips5 and each finger separately when available. Missing or semantically different points cannot receive zero error or share the same ranking. Overlays must mark fitted versus diagnostic points and display the q and registration associated with the selected condition. Surface display is not surface fitting.
7. Investigate surprises in this order: model identity, point semantics, units/axes/limits, registration, solver restarts, then weights/point/target sensitivity. Inspect actual point overlays before accepting anatomical claims. Correct evidenced faults, preserve original outputs and identify changed inputs.

## Reusable modules

- `scripts/metrics.py`: static scale/beta matching; per-feature contributions; absolute and neutral-relative pose scoring; CLI and synthetic self-check (NumPy).
- `scripts/robot.py`: physical URDF FK/Jacobian with composed mimic limits (NumPy/SciPy). Supply reviewed landmark names.
- `scripts/mano.py`: right-hand MANO FK and tip LBS from external user-owned arrays (NumPy/SciPy).
- `scripts/landmarks.py`: exact package/URI resolution and explicit fixed link-local landmarks; no cross-model basename fallback or automatic anatomical inference.
- `scripts/evaluate.py`: matching-point-set calibration and constrained multi-pose IK from external calibration/evaluation targets; saves q, targets, predictions and solver diagnostics.
- [Input schemas and commands](references/inputs.md).

The supplied modules include a generic calibration/IK runner. The archived experiment's private inputs, cohort, robot mappings and pose generator are not bundled. Supply and record those parts before claiming reproduction of a particular experiment.

Validate independent FK/Jacobian, terminal-tip motion, mimic composition, proper registration, scale convention, known-MANO recovery and saved summaries as relevant. Multi-start convergence does not prove global optimality or infeasibility.

## Deliver and publish

Lead with a bounded conclusion, exact computation and a compact table. Explain important rank changes. Attach identities, mapping, pose-set provenance/hashes, settings, per-pose outputs and audit receipts.

Use original code and synthetic fixtures when publishing this workflow. Keep licensed MANO weights, private cohort betas, credentials and internal links outside the repository; users supply their own assets. Preserve source licenses. A past winner or a fixed pose count is not a default for later evaluations.
