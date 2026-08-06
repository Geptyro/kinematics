# kinematics

Pure, framework-agnostic kinematics math. No engine, no renderer, no dependencies —
the module sees only consistent number arrays and never learns which host called it.

It exists so that **one** solver serves both ends of an animation pipeline:

- the **build-time** animation baker (`mesh-x`), which poses Euler bone proxies and
  captures the result as morph targets, and
- the **runtime** pose solvers (`falcra`), which drive live skeleton bones each frame.

A gesture baked offline and the same gesture solved live are then numerically
identical modulo float frame, because they are the same code.

## Install

```
npm i github:Geptyro/kinematics
```

## What's in it

| Export | What it does |
|---|---|
| `solveTwoBonePlanar(l0, l1, fwd, up)` | Closed-form 2-bone IK in a plane. Returns `[upperAngle, lowerAngle]` as **absolute** plane angles. |
| `solveTwoBone3D({base, target, forward, up, lengths})` | Projects a 3D target into the chain's plane, then delegates to the above. |
| `solveTwoBonePositions(...)` | Same solve, returning world joint positions + segment directions. |
| `makeChain` / `makeJoint` / `makeGoal` | Declarative chain + goal description for the general solver. |
| `solvePose(chain, goals, solver)` | XPBD constraint-relaxation solver over an arbitrary chain: per-joint stiffness, rest bias and limits. |
| `composeFK` / `chainToLocalRot` / `restAnglesFromEuler` | FK and rotation-format helpers for host adapters. |
| `axisAngle` / `fromTo` / `matToEulerXYZ` | Small rotation utilities. |

`solvePose` is a safe superset of the analytic path: when a chain reduces to two free
bones with a single reach goal and no extra constraints, it detects that and routes to
the closed form.

## Conventions worth knowing

**Angles are absolute, not parent-local.** `solveTwoBonePlanar` returns both angles in
the solve plane. Hosts whose bones compose transforms (three.js, glTF) want the second
bone's angle *relative* to the first:

```js
const [upper, lower] = solveTwoBonePlanar(L1, L2, fwd, up)
hipBone.rotation.x  = upper
shinBone.rotation.x = lower - upper   // parent-local
```

**Reach clamping is the host's policy, not the kernel's.** The kernel clamps the target
to maximum extension and no further. It does **not** clamp targets that fall inside the
inner hole (closer to the base than `|l0 - l1|`), and it has no opinion about what an
unreachable goal should look like — "extend toward it", "hold the last pose", "shorten
the stride" are all legitimate host answers. Decide that at your adapter boundary,
before calling in. See `falcra`'s `FootPlanter.reachTwoBone` for a worked example.

**The kernel is frame-agnostic.** Z-up build frame, Y-up baker frame and the runtime
frame are each converted at their own adapter boundary. A conversion bug there desyncs
hosts while the kernel itself stays correct.

**Determinism is a feature.** Fixed iteration budgets, no residual-based early exit,
pre-allocated flat buffers, per-solve Lagrange-multiplier reset and chronological
warm-start — so re-baking an asset yields byte-identical output.

## Test

```
npm test
```

## Design

`docs/animation-ik-design.md` records why XPBD was chosen over FABRIK/DLS/Jacobian, how
the fast path stays byte-compatible with the analytic solver, and the staged migration
plan for adopting it across hosts.

## License

ISC
