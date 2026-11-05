# Measurement and interpretation

## Identity and correspondence

Record source URL/revision/hash, hand side, units, physical/independent/mimic counts, reference q and explicit landmark names/offsets. Mechanical names do not prove anatomy. Inspect chain, visual mesh and endpoint frames; if joint-origin equivalence is unresolved, call the result an axis-origin proxy comparison.

Native MANO indices: wrist0; index1–3, middle4–6, little7–9, ring10–12, thumb13–15. Helpers output thumb/index/middle/ring/little. Right tips `[745,317,444,556,673]` are a declared convention. MANO v1.1 changed shape-space scaling; fix shape basis, hand mean, PCA/full-axis convention and tip IDs.

## Static descriptor

One partial descriptor is ten internal segment lengths plus ten pairwise distances among five digit-base centres. It omits all terminal-to-tip lengths, widths, thicknesses and surfaces. Construct robot `d_R` at declared reference q, and MANO `d_M(beta)` at neutral theta. For each beta in declared set B:

```
alpha(beta) = dot(d_M, d_R) / dot(d_M, d_M)
r(beta) = alpha(beta)*d_M(beta) - d_R
beta_star = argmin_beta norm(r(beta))/norm(d_R)
shape_pct = 100*norm(r(beta_star))/norm(d_R)
distance_RMS_mm = 1000*norm(r(beta_star))/sqrt(number_of_features)
```

`alpha*MANO ~= robot`; one positive scale, no per-finger scaling. Beta also contains size, so alpha is a remaining scale relative to that beta, not absolute hand size. Separately fix alpha=1 to preserve physical size; the best beta may change.

Discrete B matching finds a best member of B, not the globally nearest plausible human. Continuous fitting needs a prior and extrapolation diagnostics. Neither beta norm nor residual alone establishes human typicality.

Equal item weights do not equal anatomical balance: longer distances dominate squared energy and base-pair distances are dependent. Report feature-group contributions and sensitivity; declare balanced weights in advance if needed. Do not tune them for a preferred product. Base positions can move with upstream joints/metacarpals, so reference q must be anatomically comparable or the dependence explicit.

## Calibration and multi-pose IK

With declared beta/alpha, one neutral calibration example is:

```
min_{q0 in Q, R in SO(3), t} sum_i ||FK_i(q0)-(alpha R J_i(beta,0)+t)||²
```

Inspect neutral q, limit contacts and point overlays: the optimizer may fold a mechanically displaced joint to fit an anatomical proxy. Test independent calibration poses when registration is weakly identified. Fitting registration on evaluation poses leaks information.

Freeze beta, alpha, R,t. For each held-out pose theta:

```
y_i(theta) = alpha R J_i(beta,theta)+t
q_theta = best_found argmin_{q in Q} sum_i ||FK_i(q)-y_i(theta)||²
e_abs(theta) = 1000/alpha*sqrt(mean_i ||FK_i(q_theta)-y_i(theta)||²)
e_delta(theta) = 1000/alpha*sqrt(mean_i
  ||[FK_i(q_theta)-FK_i(q0)]-[y_i(theta)-y_i(0)]||²)
```

Q contains available physical limits/coupling. Missing real constraints make this an optimistic feasible set. Finite-seed local IK failure does not prove infeasibility. `1000/alpha` gives MANO-equivalent mm for metre inputs; robot-physical mm answers a different question. Absolute error includes shape/registration. Delta removes the neutral residual vector but retains pose-dependent geometry/axes; it is not pure actuator/control quality. RMS quantities cannot be simply subtracted.

For tips, verify equivalent endpoints. A five-tip summary from a joint+tip fit differs from independent tip-only IK; label each condition.

## Statistics and claims

First compute RMS across selected points within each pose; then mean/P95 across poses. This differs from pooling all squared point errors. Define thresholds and strict/non-strict comparison in advance. A synthetic ≤10mm rate is test-specific attainment, not human coverage or hardware acceptance.

Report family-balanced/per-family results: interpolated poses are correlated and counts weight families. A correct MANO LBS implementation does not prove anatomical realism, non-collision or contact. Poses synthesized to close tips at beta0 may not close after changing beta.

For the same person across robots, freeze a common target beta and declared size treatment. Adapted-beta versus beta0 with a newly fitted alpha changes two variables and is not a beta-only ablation. Independent pose IK has no velocity, continuity, torque or closed-loop task evidence.

Numerical checks validate code; source/mapping/visual checks validate meaning. If relevant feature, calibration or target controls change the winner, present the trade-off rather than an overall winner.

Primary references: [MANO](https://mano.is.tue.mpg.de/), [SMPL-X LBS](https://github.com/vchoutas/smplx/blob/main/smplx/lbs.py), [dex-retargeting](https://github.com/dexsuite/dex-retargeting), [ROS joint/mimic documentation](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/joints_userdoc.html).
