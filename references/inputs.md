# Inputs and commands

Coordinates are metres; angles radians. NumPy/SciPy are external dependencies. These scripts perform no downloads or hardware access.

## Static matching

NPZ keys: `robot_descriptor` `(D,)`, `human_descriptors` `(B,D)`. Optional `betas` `(B,10)` is echoed only into the local output requested. `descriptor20()` accepts `(5,3,3)` thumb-first centres. Use reviewed, identical feature definitions; record source, mapping, q and feature names in the run manifest.

Optional `weights` `(D,)` defaults to all ones; entries must be finite/nonnegative with positive sum. It changes the scale, NRMSE and RMS as specified in [the method](method.md). Record supplied weights; they are feature weights, not pose-family weights. `fixed_alpha1_beta_index` is selected separately and can differ from the shape-only `beta_index`.

```
python scripts/metrics.py static --input static_inputs.npz --output static_result.json
```

## Pose scoring

NPZ: `robot_m`, `target_m` `(P,N,3)`; `neutral_robot_m`, `neutral_target_m` `(N,3)`; scalar `alpha`. Optional `families` length P. Exclude calibration targets before preparing this input.

```
python scripts/metrics.py poses --input fitted_poses.npz --output scores.json --thresholds 5 10 15
python scripts/metrics.py self-check
```

Reports neutral RMS, absolute/increment errors, sample-weighted and equal-family metrics. Equal-family P95 uses a discrete weighted-CDF inverse; unweighted P95 uses NumPy linear interpolation. Neither convention creates a human-distribution estimate.

Each pose score is RMS across 3D point distances; the reported P95 is across these pose scores. Inputs must already contain registered targets and the chosen fitted q results. The scorer does not solve calibration or IK, and increment scores reuse the supplied robot poses. For a five-tip statistic from joint+tip IK, supply the five tip points and their reference points, retaining provenance of the 20-point optimization.

## URDF adapter

```
from robot import MotionRobot
r = MotionRobot({
  'name': 'exact SKU/revision', 'urdf': '/path/to/hand.urdf',
  'landmark_joints': [[...], [...], [...], [...], [...]],
  'reference_q': {...}, 'tip_joints': [...]  # tips only if verified
}, include_tips=False)
xyz = r.points(r.reference_x)
jac = r.jacobian(r.reference_x)  # (N,3,nq)
```

Supports15/20 explicitly named axis origins; no anatomical auto-detection. Common-upstream rigid variables and angles invisible to selected points stay at reference q. Mimic chains/limits compose; inspect `joint_manifest()`. Real omitted transmission constraints cannot be inferred.

## MANO adapter

Supply your own legally obtained right-hand NPZ: `v_template(778,3)`, `shapedirs(778,3,10)`, `J_regressor(16,778)`, `kintree_table(2,16)`, `posedirs(778,3,135)`, `weights(778,16)`. Arrays must be in metres.

```
from mano import ManoMotionReference
m = ManoMotionReference('/path/to/MANO_RIGHT_arrays.npz')
xyz = m.points(beta10, theta45, with_tips=False)
```

theta45: native local axis-angle joints1…15; flat mean, root identity; wrist-centred output. Returned points are thumb-first, with per-finger tip appended only when requested. Validate your assets/conventions against an independent implementation before comparing products. The repository contains no MANO model arrays.
