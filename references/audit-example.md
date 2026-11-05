# Audited example: why a poor score is not a poor product

Review date: 2026-11-05. Scope: one frozen set of public robot models and programmatic MANO targets. This example teaches interpretation; it is not a current product recommendation.

The Sharpa asset was **W01**, not W02. The official model revision inspected was `0d19cac602f46456b819e4b6a2c09a74982c9a3e` from [sharpa-urdf-usd-xml](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml). The original score matched physical axis origins to MANO anatomical proxies. No direct anatomical or hardware registration was available.

In the common-beta0 control, the W01 internal15 neutral RMS was10.32 MANO-equivalent mm; mean across98 synthetic poses was10.35mm, while movement-increment RMS was3.91mm. Neutral error and pose error were nearly the same. A conclusion that W01 could not reproduce movement was therefore unsupported by the absolute score.

Additional checks:

- 24 neutral calibration seeds per condition recovered the same optimum to numerical precision; there was no evidence that a missed obvious registration solution explained the floor.
- The source model has a66.10mm thumb first selected segment,39mm second segment, and47/31.5mm segments for all four long fingers. These are actual mechanical-centre distances, not proof that the centres are anatomy-equivalent.
- Changing from the URDF's rectangular limits to the manual's expressible one-dimensional ROM box changed common-beta0 internal15 mean10.35→10.62mm. The manual also describes a nonrectangular MCP workspace, so this box experiment did not recover full hardware constraints.
- On one official MANO `hands_mean` probe, W01 internal15 RMS was10.90mm versus5.07mm for WUJI2.2, while their five-tip RMS values were4.60mm versus5.41mm. These tip values came from a20point joint+tip fit. A single probe is a counterexample to an overall claim, not a new benchmark winner.

Static distances, absolute registered positions, movement changes and fingertips can produce different orders. In the original experiment, changing thumb-base features or using tip P95 changed the leading model. The correct response was to expose the axes of comparison, withdraw the broad product claim and retain reproducible proxy scores. Adjusting weights until a preferred brand wins would not validate the evaluation.

The manual/URDF limit mismatch is a source-consistency problem worth recording. It does not establish which is correct for a particular delivered hand. Keep exact SKU/firmware/CAD revisions and real coupling as unresolved until independently obtained.
