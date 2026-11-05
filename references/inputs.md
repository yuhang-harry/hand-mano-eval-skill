# Inputs and commands

Coordinates are metres; angles radians. NumPy/SciPy are external dependencies. These scripts perform no downloads or hardware access.

## Static matching

NPZ keys: `robot_descriptor` `(D,)`, `human_descriptors` `(B,D)`. Optional `betas` `(B,10)` is echoed only into the local output requested. `descriptor20()` accepts `(5,3,3)` thumb-first centres. Use reviewed, identical feature definitions; record source, mapping, q and feature names in the run manifest.

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
