# Orc Rig IK / Animation Foundation — Final Design

Status: PROPOSED. Author target: `kinematics/`, `mesh-x/src/skinning/`, `homestead-assets/src/orc/`, `games/homestead/src/human/arm/`.

## 1. Goals & Non-Goals

### Goals
1. **Declarative per-joint stiffness + rest-bias (the literal ask).** An author states intent — "this swing covers 130 deg", "keep the upper arm ~horizontal while the elbow drops", "favor default joint angles; add a weight when a joint leaves rest" — and the system DERIVES the pose. Two distinct per-axis dials: **W = rotStiffness** (reluctance to move at all) and **K = restStiffness** (penalty for leaving the default angle).
2. **Believable, stylized animation quality.** No candy-wrapper, no chicken-wing; weight reads. The body-vs-arm split that `swing()` does by hand today (`bodyShare`) becomes EMERGENT from joint stiffness across an extended chain.
3. **Robust, temporally-smooth, DETERMINISTic bakes.** No frame-to-frame pops near full extension / limits; re-baking yields byte-identical GLBs (the project already verifies this).
4. **ONE shared kernel.** The same pure-math solver serves the offline crowd bake (Euler proxies -> morph targets) AND the future hero runtime (live THREE bones), killing the duplicated law-of-cosines + CCD in `IKChain.js`.
5. **Pure-JS, layered, code-style-compliant kernel.** Lives in `kinematics/` with no THREE / no deps; fails loud on missing config; no `?.`/`??`.
6. **Incremental migration.** Never breaks the working analytic 2-bone path or current clips mid-flight; keep analytic as a fast-path where it suffices.
7. **Generalizes.** Longer chains (spine/tail), legs, future creatures, and BOTH position and orientation goals — without a new solver per case.

### Non-Goals
- A general rigid-body / full physics simulation. We solve a kinematic POSE, not dynamics.
- A runtime hero rig shipped now — the hero is deferred; this design only guarantees the kernel is ready for it.
- Embedding a runtime skeleton in the shipped crowd GLB (`orcWarrior`/`orcArcher` stay morph-baked, no runtime bones).
- Replacing `ORC_MASS` COM measurement — that is a mass model, not IK, and stays as-is.

## 2. Chosen Solver & Why

**XPBD (Extended Position-Based Dynamics): Gauss-Seidel relaxation of COMPLIANT constraints over a per-axis joint-angle chain, with the analytic 2-bone law-of-cosines kept as a detected closed-form fast-path.**

A pose is the configuration that minimizes a sum of weighted soft constraints, where compliance `alpha = 1/stiffness` is the single currency. A reach goal, a "keep upper arm horizontal" aim, an elbow hinge limit, and a rest-spring are all just constraints differing only by compliance. The solver negotiates them across a REDUNDANT chain (Spine -> Chest -> Clavicle -> Arm -> Forearm -> Hand) — and redundancy is exactly what makes per-joint stiffness mean anything.

Why XPBD over the alternatives, decided against the judging criteria:

| Property | XPBD (chosen) | Jacobian-DLS | FABRIK |
|---|---|---|---|
| Per-joint W (reluctance) | first-class compliance | first-class (metric on step) | **absent** |
| Per-joint K (rest-bias) | first-class compliance | first-class (null-space) | per-sweep lerp only |
| Determinism / byte-identical bakes | strong (step-independent compliance, no matrix inverse, no adaptive damping) | weakest (Cholesky + logSO3 + adaptive lambda = largest FP surface) | strongest (position lerps only) |
| Singularity blow-up | none (no inverse) | must damp a singularity it creates | none |
| SoA / SAB crowd batching | trivial array Gauss-Seidel | hard (matrix factorization) | trivial |
| Orientation goals | needs graft (logSO3) | first-class | second-class |
| Future secondary-motion as same primitive | yes (more constraints) | bolt-on | bolt-on |

XPBD wins the combined balance of **determinism + runtime batchability + temporal coherence + a unified declarative vocabulary**, while still exposing BOTH of the user's diagonals. It loses to FABRIK only on raw FP-surface minimality and to DLS only on the rigor of "rest never fights reach"; both gaps are closed by grafts (below).

### Grafted ideas (resolving the disagreements)
- **From DLS:** logSO3 (matrix-log) rotation residual as a first-class orientation channel, so `aimHand`'s separate roll pass folds into the same solve and weapon-grip / foot-on-slope are rigorous; and a **locked / saturated axis is a true DOF REMOVAL** (dropped from the angle-state vector), never an infinitely-stiff spring (the FBIK pitfall). Plus the **analytic-equivalence unit test** that licenses the fast-path.
- **From FABRIK:** the elbow/knee **pole is a REQUIRED, constant-across-a-gesture input that throws if absent** on a short chain — a hard fail-loud guard, stronger than XPBD's soft continuous-pole, matching the "no fallbacks" rule; and an explicit **unreachable-target early-out** (chain straightens toward the goal in one pass) so the fixed iteration budget is never spent spinning; and the **position-in / rotation-out adapter discipline** (the kernel never imports THREE; each host converts frames at the boundary only).
- **From XPBD's own honesty:** prominently sequence rig-chain extension (Clavicle/Spine) BEFORE authors set stiffness, since K/W are inert on a non-redundant chain.

### Convergence
`solvePose(chain, goals, solver)` runs a **fixed iteration budget** (bake: 6-8 for quality; runtime warm: 2-4). NO `while(residual > eps)` loop — unreachable goals must never spin or spike. Per-constraint XPBD Lagrange multipliers accumulate across sweeps and reset ONCE per solve (not per iteration), so compliance is honored regardless of sweep count. A residual (max constraint violation) is RETURNED for diagnostics/asserts only, never used as a termination that could vary by platform. The FABRIK-grafted unreachable early-out: if the tip stops moving between sweeps, straighten toward the goal and stop.

### Singularity handling
There is no matrix inverse, so nothing blows up. A fully-extended chain simply has the reach already satisfied with the rest-spring / limit deciding the redundant bend, so it is well-conditioned by construction. Layered guards:
- **Reach soften band:** within a distance ratio of max reach, the reach constraint's effective compliance ramps up (tanh remap), easing the limb toward extension instead of snapping straight at the boundary.
- **Soft joint limits:** tanh-saturated inequality constraints with a margin band — C1-continuous, no velocity pop at a stop. A fully `locked` axis is removed from the state vector (DOF removal), not a heavy spring.
- **Constant required pole:** the bend plane is pinned by a continuously-varying-but-required pole, so the elbow/knee can never flip basins.

### Temporal coherence (warm-start is the backbone)
1. The `Chain` carries `chain.angles` (and `chain.lambda`) persistently; each solve **seeds from the prior converged angles**. A smoothly moving target needs only 2-4 sweeps and the solution stays in one basin — this kills frame-to-frame pops at the source AND is the speed win.
2. **Bake order is strictly chronological**; frame N inherits frame N-1's angles. For LOOPING clips, frame 0 is seeded from the last frame and re-solved to close the seam byte-identically (preserving today's `fn(0) == fn(1)` authoring assumption as a SOLVED seam, not an asserted one).
3. Bend-plane pinned by a continuous pole (never a sign-ambiguous quaternion).
4. Soft limits + reach soften remove the limit-stop and full-extension discontinuities.
5. Low warm iteration count keeps the solution near the temporally-coherent warm pose.
6. **Determinism:** fixed iterations + accumulator-reset-per-solve + no residual early-exit + pure deterministic ops + pre-allocated flat buffers (no Map-iteration order, no hot-loop allocation) = same input -> same output on every platform.
7. **Secondary motion is layered AFTER the solve** (critically-damped springs on loose parts), never fed back in; goal targets are eased / spring-smoothed BEFORE entering the solve.

## 3. Architecture & Layering

```
kinematics/                      PURE math. No THREE, no mesh-x, no Bone. Numbers + flat arrays only.
  src/index.js                   KEEP: solveTwoBone{Planar,3D,Positions}, fromTo, axisAngle, matToEulerXYZ, clamp.
  src/chain.js          (NEW)    makeJoint(spec), makeChain(spec), makeGoal(spec) — throw on any missing field.
  src/constraints.js    (NEW)    pure projections: reachConstraint, aimConstraint, restConstraint,
                                 limitConstraint, planeConstraint, logSO3 (rotation residual).
  src/solve.js          (NEW)    solvePose(chain, goals, solver), chainToLocalRot(chain), composeFK(chain).
                                 Detects 2-bone-single-reach -> calls solveTwoBonePositions (fast-path).

mesh-x/src/skinning/             BAKE bridge. Knows proxies (Y-up Euler) + composeBoneWorld FK; calls kernel.
  poseIk.js                      reachChain/reachArm/aimHand kept as THIN wrappers -> postureRig.
  postureRig.js         (NEW)    poseChain(b, skeleton, chainSpec, goals, solver): build Chain from proxy
                                 state + the orc rig table, solvePose, write chainToLocalRot back as
                                 matToEulerXYZ proxy rotations. Frame conversion (zUpToYUp/yUpToZUp) lives HERE.

homestead-assets/src/orc/        GESTURE layer. Authors INTENT only.
  baseOrc.js            (EDIT)   add Clavicle{R,L} + Spine bones to the hierarchy + rest offsets + mounts.
  orcRig.js             (NEW)    the rig STIFFNESS TABLE: per joint restEuler/axes/rotStiffness/restStiffness/limit.
  motion.js             (EDIT)   swing/balance/shootBow rewritten declaratively; bodyShare/pole/maxBend deleted.

games/homestead/src/human/arm/   RUNTIME (hero, deferred).
  IKChain.js            (EDIT)   thin THREE adapter over solvePose; DELETE _solve2Bone + _solveCCD.
```

**Layer discipline:** `kinematics` sees only consistent number arrays — it is frame-agnostic. The three coordinate frames (Z-up build, Y-up baker, three.js runtime) are each converted at THEIR OWN adapter boundary (`postureRig.js` for bake, `IKChain.js` for runtime), matching `composeBoneWorld`'s existing design. The kernel never knows which host called it, so a baked crowd pose and a live hero pose for the same gesture are numerically identical modulo float frame.

## 4. Data Model (concrete JS)

All in `kinematics/src/` as plain data. Factories THROW on any missing field (no defaults). No `?.`/`??`. Internals are flat `Float64Array` (SoA, worker/SAB-friendly).

```js
// kinematics/src/chain.js
export function makeJoint(spec) {
  if (!spec.name) throw new Error('Joint.name required')
  if (spec.parentIndex === undefined) throw new Error('Joint.parentIndex required (-1 for root)')
  if (!spec.restEuler) throw new Error('Joint.restEuler required')
  if (!spec.restOffset) throw new Error('Joint.restOffset required')
  if (!spec.axes) throw new Error('Joint.axes required')
  if (!spec.rotStiffness) throw new Error('Joint.rotStiffness (W) required')
  if (!spec.restStiffness) throw new Error('Joint.restStiffness (K) required')
  if (!spec.limit) throw new Error('Joint.limit required')
  return {
    name: spec.name,
    parentIndex: spec.parentIndex,    // int; -1 for chain root
    restEuler: spec.restEuler,        // {x,y,z} three.js XYZ-order — the DEFAULT/preferred pose (q_rest)
    restOffset: spec.restOffset,      // [x,y,z] local translation to this joint (source of bone length)
    axes: spec.axes,                  // [[x,y,z],...] 1..3 UNIT local rotation axes this joint may use
                                      //   one axis = pure hinge (elbow/knee); three = ball (shoulder/chest)
    rotStiffness: spec.rotStiffness,  // W: per-axis reluctance to MOVE from current. >0 (small=loose).
    restStiffness: spec.restStiffness,// K: per-axis pull back toward restEuler. >=0. THE user's 'weight when leaving default'.
    limit: spec.limit,                // per axis: { mode:'free'|'limited'|'locked', min, max, margin } radians
                                      //   'locked' => axis DROPPED from the state vector (true DOF removal)
  }
}

export function makeChain(spec) {
  if (!spec.joints) throw new Error('Chain.joints required')
  if (!spec.rootFrame) throw new Error('Chain.rootFrame required')
  if (!spec.iterations) throw new Error('Chain.iterations required')   // fail loud, no default
  // angles/lambda are derived (sum of active axes) and warm-started across solves
  return {
    joints: spec.joints,              // depth-first, parentIndex < self (mirrors baseOrc hierarchy)
    rootFrame: spec.rootFrame,        // { pos:[x,y,z], rot: mat3 } world placement of joint[0]'s parent
    iterations: spec.iterations,      // fixed budget
    angles: null,                     // Float64Array packed per ACTIVE DOF (warm-startable). Allocated on first solve.
    lambda: null,                     // Float64Array XPBD multiplier accumulator, reset per solve.
  }
}

export function makeGoal(spec) {
  if (spec.jointIndex === undefined) throw new Error('Goal.jointIndex required')
  if (spec.kind === undefined) throw new Error('Goal.kind required')          // 'reach'|'aim'|'level'|'plane'
  if (spec.posWeight === undefined) throw new Error('Goal.posWeight required')
  if (spec.rotWeight === undefined) throw new Error('Goal.rotWeight required')
  if (spec.compliance === undefined) throw new Error('Goal.compliance required')
  if (spec.soften === undefined) throw new Error('Goal.soften required')
  // pole is REQUIRED on short chains; the builder for reach goals throws if absent (FABRIK graft)
  return {
    jointIndex: spec.jointIndex,      // effector bone (tip, or ANY bone for multi-goal)
    kind: spec.kind,
    target: spec.target,              // [x,y,z] world pos (reach) OR null
    targetRot: spec.targetRot,        // mat3 world orientation (aim/level) OR null; residual via logSO3
    localAxis: spec.localAxis,        // for 'aim': item lead axis in effector local frame
    posWeight: spec.posWeight,        // 0..1 position channel enable/blend (ozz/FBIK posAlpha; 0 costs nothing)
    rotWeight: spec.rotWeight,        // 0..1 orientation channel enable/blend (rotAlpha)
    compliance: spec.compliance,      // 1/pullWeight: low = hard pin (feet), high = soft suggestion
    soften: spec.soften,              // 0..1 reach-softening band (distance ratio from max reach)
    pole: spec.pole,                  // [x,y,z] REQUIRED bend-plane hint on short chains; constant across a gesture
  }
}
```

Kernel API (`kinematics/src/solve.js`, all pure, pre-allocated, no hot-loop allocation):

```js
export function solvePose(chain, goals, solver)   // warm-started XPBD; mutates chain.angles; returns { residual, reached }
                                                   // detects 2-bone single-reach -> solveTwoBonePositions fast-path
export function chainToLocalRot(chain)            // angles -> per-joint row-major mat3 (host converts: Euler at bake, quat at runtime)
export function composeFK(chain)                  // parent-index FK (mirrors composeBoneWorld math, no Bone class)
```

## 5. Declarative Authoring API

The author declares a CHAIN SPEC (which joints, from the rig table) + a list of GOALS (intent), then solves. The body/arm split is no longer hand-written — it FALLS OUT of putting Spine/Chest/Clavicle in the chain with their own stiffness.

The orc gets ONE rig stiffness table; gestures state only deviations.

```js
// homestead-assets/src/orc/orcRig.js  — authored ONCE
// 'favor rest' = high restStiffness; 'reluctant to move' = high rotStiffness; hinge = single axis.
export const ORC_RIG = {
  Spine:    { restEuler: REST.Spine,    axes: BALL,  rotStiffness: [6,6,6],   restStiffness: [5,5,5],   limit: SPINE_LIM },   // barely yields
  Chest:    { restEuler: REST.Chest,    axes: BALL,  rotStiffness: [4,4,4],   restStiffness: [3,3,3],   limit: CHEST_LIM },
  ClavicleR:{ restEuler: REST.ClavicleR,axes: BALL,  rotStiffness: [2,2,2],   restStiffness: [2,2,2],   limit: CLAV_LIM },
  ArmR:     { restEuler: REST.ArmR,     axes: BALL,  rotStiffness: [0.5,0.5,0.5], restStiffness: [0.6,0.6,0.6], limit: SHOULDER_LIM },
  ForearmR: { restEuler: REST.ForearmR, axes: ELBOW, rotStiffness: [0.3],     restStiffness: [0.4],     limit: { mode:'limited', min: deg(15), max: deg(160), margin: deg(8) } },
  HandR:    { restEuler: REST.HandR,    axes: BALL,  rotStiffness: [1,1,1],   restStiffness: [1,1,1],   limit: WRIST_LIM },
  // ...mirror L, plus Thigh/Shin/Foot for legs...
}
```

### swing — REWRITTEN (no bodyShare, no manual chest twist, no maxBend)

```js
export function swing(b, skeleton, side, { arc, phase, center, height, radius, pivot, weaponAxis }) {
  const s = side === 'L' ? -1 : 1
  const phi = center * s + (phase - 0.5) * arc * s
  const hand = [pivot[0] + radius * Math.sin(phi), pivot[1] - radius * Math.cos(phi), height]
  // arm chain = Spine -> Chest -> Clavicle -> Arm -> Forearm -> Hand, stiffness from ORC_RIG
  const chain = armChain(b, skeleton, side)
  const goals = [
    reachGoal(chain, 'Hand' + side, hand, { compliance: 0.002, soften: 0.15, pole: [0.3 * s, 0, -1] }),
  ]
  if (weaponAxis)                       // orientation as a SECOND channel, not a separate aimHand pass
    goals.push(aimGoal(chain, 'Hand' + side, weaponAxis, [Math.sin(phi), -Math.cos(phi), 0], { rotWeight: 1, compliance: 0.02 }))
  solvePose(chain, goals, { iterations: 6 })
  writeChain(b, chain)
  // The chest's restStiffness (high) vs the arm's (low) decides how much the body turns to help.
  // That replaces bodyShare. The elbow hinge limit replaces maxBend.
}
```

### bowDraw — "favor defaults / keep upper arm horizontal while the elbow drops"

This is the user's exact phrasing, expressed as constraints — never hand-posed angles.

```js
export function bowDraw(b, skeleton, { draw /*0..1*/, aimDir }) {
  const bowArm  = armChain(b, skeleton, 'L')      // holds the bow forward
  const drawArm = armChain(b, skeleton, 'R')      // string hand pulls back by `draw`

  // 'upper arm ~horizontal' = a SOFT level goal on the upper bone's segment direction; it yields only if it must.
  // 'favor rest' is the global K behavior from ORC_RIG; the elbow's low restStiffness + hinge limit makes it DROP.
  const bowGoals = [
    levelGoal(bowArm, 'ArmL', [1, 0, 0], { rotWeight: 1, compliance: 0.03 }),  // keep upper arm ~horizontal (soft)
    aimGoal(bowArm,   'HandL', BOW_AXIS, aimDir, { rotWeight: 1, compliance: 0.01 }),
    reachGoal(bowArm, 'HandL', bowHoldPoint(aimDir), { compliance: 0.005, soften: 0.2, pole: BOW_POLE_L }),
  ]
  solvePose(bowArm, bowGoals, { iterations: 8 }); writeChain(b, bowArm)

  // draw hand pulls the string to the cheek; elbow drop is EMERGENT (rest-bias + hinge limit + low pole).
  const drawGoals = [
    reachGoal(drawArm, 'HandR', nockAnchor(aimDir, draw), { compliance: 0.004, soften: 0.1, pole: [0.2, -1, -0.3] }),
  ]
  // per-gesture overlay on ArmR/ForearmR (high K toward horizontal upper arm; free, low-K elbow):
  solvePose(drawArm, drawGoals, { iterations: 8, overrides: {
    ArmR:     { restEuler: UPPER_ARM_HORIZONTAL_R, restStiffness: [8, 8, 8] }, // pull upper arm toward horizontal
    ForearmR: { restStiffness: [0.4] },                                        // elbow drops freely
  }}); writeChain(b, drawArm)
}
```

Mechanism: the level/horizontal goal is SOFT (high compliance) so it holds the upper arm horizontal UNTIL the hard hand goal genuinely needs the shoulder to move, at which point it trades a little horizontality for the reach — exactly "keep it ~horizontal" rather than a hard lock. "Elbow drops" is `restStiffness` + the hinge limit + low pole. "Favor rest" is the global K springs. The author states intent; the solver derives the pose.

### balance — REWRITTEN (keep COM math, replace hand-tuned scalars)

```js
export function balance(b, skeleton, { weapon, offArm, legs }) {
  const com = measureCOM(b, skeleton, ORC_MASS, weapon)   // UNCHANGED mass-model math
  const sup = footMidpoint(b, skeleton)
  const off = horiz(sub(com, sup))
  if (Math.hypot(off[0], off[1]) < DEADZONE) return { com, off }
  // ONE multi-goal solve. Gauss-Seidel ORDER is the priority: feet pinned hardest, then COM, then off-arm.
  const goals = [
    footGoal('R', braceFoot('R', off), { compliance: 0.0005, pole: [0, -1, 0], ground: true }),  // pinned, levelled
    footGoal('L', braceFoot('L', off), { compliance: 0.0005, pole: [0, -1, 0], ground: true }),
    comGoal(bodyChain(b, skeleton), sup, { compliance: 0.05 }),               // spine counter-leans to recenter COM
    counterGoal(armChain(b, skeleton, offArm), off, { compliance: 0.01 }),    // off-arm counterweight
  ]
  solveMulti(b, skeleton, goals, { iterations: 8 })
  return { com, off }
}
```

`leanGain`, `hipShift`, `offReach`, `headFollow`, and the manual `b.Chest.rotation.x/z` hacks are DELETED. Counter-lean emerges from the spine's low W + the COM goal; the off-arm fling emerges from the shoulder's low stiffness while the chest's high stiffness holds the torso.

## 6. Shared Bake + Runtime Story

ONE function — `solvePose(chain, goals, solver)` — is the entire shared kernel. Both hosts differ only in how they adapt their bone representation to the `Chain` SoA and how they write results back.

**BAKE (crowd, mesh-x, Node, Euler proxies -> morphs).** `mesh-x/src/skinning/postureRig.js`:
1. Read each bone proxy's `position` as `restOffset` and the `ORC_RIG` table for `restEuler`/W/K/limits.
2. Set `rootFrame` from `composeBoneWorld`'s parent `worldPos`/`worldRot` (the existing FK walk, unchanged).
3. Seed `chain.angles` from the proxies' current Euler (warm-start; keep a per-clip, per-chain cache of last frame).
4. Convert goal targets `zUpToYUp` and call `solvePose` in the baker's Y-up frame.
5. `chainToLocalRot` -> `matToEulerXYZ` -> write back into `b[name].rotation`.
6. `bakeSkeletalAnimations` then does FK and captures geometry as morph targets EXACTLY as today. Shipped `orcWarrior`/`orcArcher` GLBs still have NO runtime skeleton.

`reachChain`/`reachArm`/`aimHand` become thin wrappers that construct a single-goal/aim-goal Chain on the SAME kernel, so existing clips keep working unchanged during migration (and hit the analytic fast-path).

**RUNTIME (hero, falcra, live THREE bones -> quaternions).** `games/homestead/src/human/arm/IKChain.js` is reimplemented as a thin adapter:
- Build a `Chain` from `THREE.Bone` (`restOffset` from `bone.position`, `restEuler` from `bone.quaternion` at `build()`).
- Carry `chain.angles` across frames for warm-start.
- Call the SAME `solvePose` each tick.
- Apply `chainToLocalRot` as `bone.quaternion` corrections, layered over sampled animation (ozz-style `final_local = blend(sampled, ik, ikAlpha)`).
- `solveBlended` maps to `posWeight`/`rotWeight`; `rest(factor)` maps to raising `restGain`; `addJoint` constraints map to `Joint.limit`.

The kernel never imports THREE; the adapter is the only THREE-aware code. This DELETES the duplicated `_solve2Bone` (law of cosines) AND `_solveCCD`, and removes the `?? Infinity` / `?? null` / `?? Math.PI` style violations currently in `IKChain.js`. Both hosts are SoA, so a crowd of hero-quality solves can batch across the worker/SAB pipeline.

**Determinism for byte-identical GLBs:** kernel uses only deterministic ops, a fixed iteration cap, pre-allocated flat buffers (no Map-iteration order, no allocation-dependent FP), per-solve lambda reset, and chronological warm-start — so re-baking yields identical morphs.

## 7. Incremental Migration

The analytic 2-bone path survives as a DETECTED fast-path and existing clips bake byte-identically until a rig is deliberately given redundancy + stiffness. A **GLB byte-diff CI gate is a hard precondition on every phase that should be a no-op**, and `swingLegacy`-style old gestures live alongside new ones so clips switch ONE AT A TIME.

- **Phase 0 — kernel beside the analytic one, nothing wired.** Add `kinematics/src/{chain,constraints,solve}.js`. KEEP `solveTwoBone*`/`fromTo`/`axisAngle`/`matToEulerXYZ` exactly. Mocha test: a 2-bone single-reach chain returns angles within 1e-9 of `solveTwoBonePositions` (proves the analytic path is the XPBD fixed point and licenses the fast-path); a 3+ chain is deterministic across runs.
- **Phase 1 — fast-path detection inside `solvePose`.** When the active chain reduces to 2 free bones + one reach goal + no extra constraints, call `solveTwoBonePositions` and return. `solvePose` is now a safe superset.
- **Phase 2 — bake bridge under unchanged clips.** Add `mesh-x/src/skinning/postureRig.js`; re-implement `reachChain`/`reachArm`/`aimHand` to delegate, defaulting every existing call to a 2-bone chain (fast-path). Re-bake `orcWarrior`/`orcArcher`; assert BYTE-IDENTICAL GLBs. This proves the bridge is a no-op refactor before any clip changes.
- **Phase 3 — give the orc redundancy.** Add `Clavicle{R,L}` + `Spine` to `baseOrc.js` (hierarchy, rest offsets, mounts/poseAnchors re-validated). Author `orcRig.js`. Add `reachGoal`/`aimGoal`/`levelGoal`/`comGoal`/`poseChain` builders to `motion.js`. This is the FIRST intentional visual/GLB change (byte-diff gate is re-baselined here). Eyeball via `npm run preview` + a GPU playtest.
- **Phase 4 — rewrite gestures declaratively, one at a time.** `shootBow` first (archer crowd, morph-baked, easy to eyeball). Then `swing` (drop `bodyShare`/manual chest twist/`maxBend`). Then `balance` (keep `ORC_MASS` COM, multi-goal solve with feet-first Gauss-Seidel priority). Keep each old gesture until the new one is signed off; byte-diff gate green per migrated clip; tune W/K in `tuning.json`, not magic scalars.
- **Phase 5 — unify the runtime.** Reimplement `IKChain.js` as the THREE adapter over `solvePose`; DELETE `_solve2Bone` + `_solveCCD` + the `??` violations. Map `solveBlended`/`rest`/`addJoint`. Verify hero arm matches the baked crowd pose for the same gesture. Leave `conquest`/`conquest-old` IKChains dead per memory.
- **Phase 6 — generalize.** Spine look-at, foot-on-slope (`planeConstraint` + ground-normal orientation goal, replacing `levelEnd`), two-handed bow (two simultaneous hand goals), future creatures — all by writing new rig tables only. Layer secondary-motion spring bones on top at runtime.

## 8. Risks & Open Questions

**Risks**
- Stiffness/compliance is a tuning surface; believable weight needs a GPU/browser playtest (the headless `npm run preview` shows only static shape+swatch, not baked-morph motion).
- Stiffness is INERT on a non-redundant chain — `baseOrc.js` has Chest directly under Root today, so Phase 3 chain extension MUST precede gesture rewrites.
- Determinism now depends on discipline: fixed iterations, per-solve lambda reset, no residual early-exit, chronological warm-start, no hot-loop allocation, no Map-order dependence, and a FIXED documented Gauss-Seidel projection order (feet -> reach -> rest). The byte-diff gate is the net.
- Three coordinate frames must be bridged at the ADAPTER boundary only; a conversion bug desyncs hero vs crowd while the kernel looks correct.
- Emergent distribution shifts debugging from one `bodyShare` scalar to per-joint W/K ratios across Spine/Chest/Clavicle — harder to localize; mitigated by live `tuning.json`.
- "Rest never fights reach" and orientation faithfulness degrade near full extension; reach-soften + soft limits + constant pole must be tuned together.
- Crowd bakes of long clips multiply iterative cost; keep iteration budgets tight + rely on warm-start + fast-path.
- Adding Clavicle/Spine changes the skeleton: mounts, baked morphs, bone indices must be re-validated; the GLB legitimately changes at Phase 3.

**Open questions**
- Exact rest placement of the new Clavicle/Spine bones.
- Starting W/K/limit values for `orcRig.js`; live in `tuning.json` from day one?
- Iteration budgets (bake 6-8 vs runtime 2-4) — confirm against full clip set bake time.
- `balance()` multi-goal priority — fixed Gauss-Seidel order vs explicit compliance ratios?
- Hero IK as a correction layer over sampled animation vs primary pose source?
- Loop-seam re-solve count — closes byte-identically without a hitch?
- Two-handed bow — sequential single-chain + coupling constraint vs a true simultaneous solve?

## 9. Broader Better-Animations Roadmap (unlocked by this foundation)

Because every concept (goal, limit, rest-spring, aim, plane) is the SAME compliant-constraint primitive, these become incremental additions, not new systems:

1. **Secondary motion / follow-through.** Critically-damped spring bones layered AFTER the solve (capes, pouches, weapon tip overshoot). Springs can also smooth GOAL TARGETS before they enter the solve, giving stable, wobble-free goals.
2. **Foot & hand locking (IK pinning).** A foot planted during a stride is a low-compliance reach goal + ground-normal orientation goal (`planeConstraint`) — eliminates foot sliding on slopes/stairs, replacing `levelEnd`. Hand-on-weapon / hand-on-ledge is the same.
3. **Look-at / head + eye tracking.** An aim goal on Neck/Head/Spine with high rest-bias so the body contributes only its natural share — the redundant spine chain makes a believable whole-body look-at automatic.
4. **Additive / layered poses.** Solve a base locomotion pose, then apply gesture goals as additive corrections via `posWeight`/`rotWeight` blends (ozz-style), enabling aim-while-walking and damage-flinch overlays without authoring combinatorial clips.
5. **Multi-effector full-body balance.** Two hands on a bow, foot + foot + hand + look in one prioritized solve (Gauss-Seidel order = priority) — the design already supports it; Phase 6 just adds goals.
6. **Reach-aware interactions.** Gather/attack targets become reach goals with the unreachable early-out giving a graceful "stretch toward but can't quite touch" read instead of a snap.
7. **New creatures / longer chains.** Tails, wings, quadrupeds, spine-driven serpents — all are new rig tables (joints + W/K/limits) over the unchanged kernel.