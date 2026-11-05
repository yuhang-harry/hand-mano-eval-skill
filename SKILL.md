---
name: hand-mano-eval
description: Compare dexterous robot hands with MANO and audit unexpected rankings through verified correspondences, shape and scale fitting, frozen calibration, and constrained pose IK. Use for URDF-to-human morphology or retargeting geometry, with separate evidence for hardware performance.
---

# Hand–MANO evaluation

Deliver a reproducible geometric comparison and explain what determines its ordering. Correct FK arithmetic does not establish anatomical correspondence or benchmark fairness.

Read [the method](references/method.md) before choosing metrics and [the audited example](references/audit-example.md) when investigating rank changes.

## Establish the comparison

- Fix exact model/SKU/revision, side, units, URDF/mesh hashes and official/derivative status. An older public model cannot answer a newer SKU question.
- Inspect the full physical chain before mapping MCP/PIP/DIP or thumb CMC/MCP/IP. Co-located flexion/abduction axes are one centre; a displaced axis/link origin is not automatically an anatomical centre. Record rationale and confidence for each mapping.
- Distinguish physical URDF, baked calibration and virtual representation joints. Respect encoded limits/mimic; record missing tendon or closed-chain constraints.
- Verify endpoints. A distal joint origin is not a fingertip. Evaluate a verified common tip subset separately; do not invent missing points or mix point counts.
- Decide whether the question concerns size, proportions, fitted position, movement changes, endpoints or actual tasks. Report these separately; no arbitrary overall winner.

## Compute and audit

1. Extract identical static features. Fit one whole-hand scale per candidate beta, then select beta by a declared objective. Distinguish discrete matching from continuous fitting; record MANO convention and priors.
2. Report physical-size and shape-only residuals, scale and per-finger contributions. Equal item weights do not imply anatomical balance: feature magnitudes affect the scale fit and normalization denominator; actual error contributions require residual SSE. Pairwise base distances are dependent and base spacing may depend on reference q. For bone-only, thumb/base or terminal-length sensitivity, state whether beta/alpha are refitted or held fixed; compare percentages only within the same feature definition.
3. Select or fit beta/scale by the declared shape protocol and freeze them before calibrating physical reference angles q0 and proper R,t on calibration targets excluded from evaluation. Distinguish static q_ref, fitted q0 and a verified natural open hand. For held-out poses optimize only the selected feasible physical q; save optimized/frozen variables, targets, predictions and status. Do not align each pose again.
4. Show reference-position RMS, absolute multi-pose RMS and movement-increment RMS together. Declare whether increment scores reuse absolute-objective q or come from a separate optimization. A reference residual is not a lower bound for every pose. Never subtract two RMS values as a decomposition or infer motion quality from their rank correlation.
5. Separate internal-centre and fingertip scores. An internal joint centre cannot observe its own terminal rotation. State whether tips came from joint+tip or tip-only IK.
6. Compute each pose's RMS of 3D point distances, then mean/P95 across poses; pose P95 is not a percentile of pooled point errors. Report predeclared threshold attainment and per-family or equal-family statistics. Label adapted-beta/common-beta conditions on every result table. Counts weight gesture families. Synthetic targets establish synthetic-test attainment, not human coverage or temporal dynamics.
7. Investigate surprises in this order: model identity, point semantics, units/axes/limits, registration, solver restarts, then weights/point/target sensitivity. Inspect actual point overlays before accepting anatomical claims. Correct evidenced faults, preserve original outputs and identify changed inputs.

## Reusable modules

- `scripts/metrics.py`: static scale/beta matching; per-feature contributions; absolute and neutral-relative pose scoring; CLI and synthetic self-check (NumPy).
- `scripts/robot.py`: physical URDF FK/Jacobian with composed mimic limits (NumPy/SciPy). Supply reviewed landmark names.
- `scripts/mano.py`: right-hand MANO FK and tip LBS from external user-owned arrays (NumPy/SciPy).
- [Input schemas and commands](references/inputs.md).

The supplied modules are adapters and scorers. They do not provide the archived experiment's calibration/IK runner, pose generator or licensed/private inputs. Supply and record those parts before claiming end-to-end reproduction.

Validate independent FK/Jacobian, terminal-tip motion, mimic composition, proper registration, scale convention, known-MANO recovery and saved summaries as relevant. Multi-start convergence does not prove global optimality or infeasibility.

## Deliver and publish

Lead with a bounded conclusion, exact computation and a compact table. Explain important rank changes. Attach identities, mapping, pose-set provenance/hashes, settings, per-pose outputs and audit receipts.

Use original code and synthetic fixtures when publishing this workflow. Keep licensed MANO weights, private cohort betas, credentials and internal links outside the repository; users supply their own assets. Preserve source licenses. A past winner or a fixed pose count is not a default for later evaluations.
