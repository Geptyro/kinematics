/**
 * Pure constraint residuals for the XPBD pose solver (see solve.js). Each returns
 * `{ C, grad, compliance }` for ONE scalar constraint, given the current forward-kinematics
 * snapshot — the solver owns the XPBD bookkeeping (per-joint W weights, Lagrange multipliers,
 * theta updates). Splitting it this way keeps the math testable and the solver loop uniform.
 *
 *   • reachRow  — one position component (x|y|z) of an effector toward a world target
 *   • aimRow    — one component of the rotation-vector residual aligning a local axis to a world dir
 *   • restRow   — pull one axis toward its preferred restAngle (compliance = 1/K)
 *   • limitRow  — soft one-sided joint-limit push (only when violated)
 *
 * `grad` is a Float64Array over the chain's active DOFs; for reach/aim only DOFs on the path
 * from the effector to the chain root contribute (a non-ancestor joint cannot move the effector).
 */
import { add3, sub3, mul3, dot3, cross3, len3, norm3, matVec3, logSO3 } from './vec.js'
import { fromTo } from './index.js'

// Joint limits are near-hard: a tiny compliance keeps them stiff without the ill-conditioning
// of an infinitely-stiff spring (a `locked` axis is removed from the state entirely instead).
export const LIMIT_COMPLIANCE = 1e-6

/** Boolean mask of joints on the path from `g` up to the chain root (inclusive). */
function ancestorMask(chain, g) {
	const mask = new Array(chain.joints.length).fill(false)
	let i = g
	while (i >= 0) { mask[i] = true; i = chain.joints[i].parentIndex }
	return mask
}

/** Reach soften: ramp compliance up as the target nears/exceeds the chain's max reach, easing extension. */
function softenCompliance(chain, fk, goal, mask) {
	if (goal.soften <= 0) return goal.compliance
	let maxLen = 0
	for (let i = 0; i < chain.joints.length; i++) { if (mask[i] && i !== 0) maxLen += chain.joints[i].len }
	if (maxLen <= 0) return goal.compliance
	const base = fk.worldPos[0]
	const dist = Math.hypot(goal.target[0] - base[0], goal.target[1] - base[1], goal.target[2] - base[2])
	const over = Math.max(0, Math.min(goal.soften, dist / maxLen - (1 - goal.soften)))
	const ratio = over / goal.soften
	return goal.compliance * (1 + 8 * ratio)
}

export function reachRow(chain, fk, goal, comp) {
	const g = goal.jointIndex
	const tip = fk.worldPos[g]
	const mask = ancestorMask(chain, g)
	const grad = new Float64Array(chain.dof.length)
	for (let d = 0; d < chain.dof.length; d++) {
		if (!mask[chain.dof[d].joint]) continue
		const r = sub3(tip, fk.pivot[d])
		grad[d] = cross3(fk.axisWorld[d], r)[comp]
	}
	return { C: tip[comp] - goal.target[comp], grad, compliance: softenCompliance(chain, fk, goal, mask) }
}

export function aimRow(chain, fk, goal, comp) {
	const g = goal.jointIndex
	const cur = norm3(matVec3(fk.worldRot[g], goal.localAxis)) // where the local axis points now (world)
	const e = logSO3(fromTo(cur, norm3(goal.target)))          // rotation vector to apply to align it
	const mask = ancestorMask(chain, g)
	const grad = new Float64Array(chain.dof.length)
	for (let d = 0; d < chain.dof.length; d++) {
		if (!mask[chain.dof[d].joint]) continue
		grad[d] = fk.axisWorld[d][comp]                           // dω/dθ = the DOF's world rotation axis
	}
	return { C: -e[comp], grad, compliance: goal.compliance }   // drive Σ grad·dθ → e
}

/**
 * Pole / swivel: steer the MID joint (elbow/knee = the reach effector's parent) around the
 * base→tip axis toward the `pole` direction, WITHOUT changing how far it bends. Analytic 2-bone
 * gets this for free from its bend plane; a multi-joint XPBD chain does not, so this is what makes
 * the elbow controllable (keeps it OUT of the body instead of falling wherever the solve lands).
 * `base` = the mid joint's parent. Returns a reach-style row pulling the mid toward its swivelled
 * position; soft (its own compliance) so it steers the elbow without fighting the tip reach.
 */
export const POLE_COMPLIANCE = 0.12 // soft: steers the elbow but lets the (firmer) reach win the tip
export function poleRow(chain, fk, goal, comp) {
	const tip = goal.jointIndex
	const mid = chain.joints[tip].parentIndex
	if (mid < 0) return null
	const base = chain.joints[mid].parentIndex
	if (base < 0) return null
	const bP = fk.worldPos[base], mP = fk.worldPos[mid], tP = fk.worldPos[tip]
	const axis = norm3(sub3(tP, bP))
	if (len3(axis) < 1e-6) return null
	// Desired elbow direction ⟂ the base→tip axis. Two ways to specify it: a SWIVEL ANGLE (the
	// honest 1-DOF control — angle around the axis from a reference dir), or a pole VECTOR (legacy).
	let polePerp
	if (typeof goal.swivel === 'number') {
		let u = sub3(goal.swivelRef, mul3(axis, dot3(goal.swivelRef, axis))) // reference ⟂ axis → swivel zero
		if (len3(u) < 1e-5) return null                                      // reference parallel to the limb
		u = norm3(u)
		const w = cross3(axis, u)                                            // completes the in-plane basis
		polePerp = add3(mul3(u, Math.cos(goal.swivel)), mul3(w, Math.sin(goal.swivel)))
	} else {
		polePerp = sub3(goal.pole, mul3(axis, dot3(goal.pole, axis)))
		if (len3(polePerp) < 1e-5) return null                              // pole parallel to the limb → undefined swivel
		polePerp = norm3(polePerp)
	}
	const rel = sub3(mP, bP)
	const along = dot3(rel, axis)
	const r = len3(sub3(rel, mul3(axis, along)))                      // current bend radius (preserved)
	const desiredMid = add3(add3(bP, mul3(axis, along)), mul3(polePerp, r))
	const mask = new Array(chain.joints.length).fill(false)
	for (let i = mid; i >= 0; i = chain.joints[i].parentIndex) mask[i] = true
	const grad = new Float64Array(chain.dof.length)
	for (let d = 0; d < chain.dof.length; d++) {
		if (!mask[chain.dof[d].joint]) continue
		grad[d] = cross3(fk.axisWorld[d], sub3(mP, fk.pivot[d]))[comp]
	}
	return { C: mP[comp] - desiredMid[comp], grad, compliance: POLE_COMPLIANCE }
}

/**
 * Self-collision keep-out: push a joint OUT of a CAPSULE (segment a–b + radius) — a cheap, mesh-free
 * approximation of body collision (the torso/head). Active only while the joint is inside; pushes it
 * radially to the surface. Reusable for any limb-vs-body case (bow draw, swing, hero). `cap` = { a, b,
 * radius } in the solve frame. Protects the named joint (its ancestors do the moving).
 */
export const KEEPOUT_COMPLIANCE = 1e-5
export function keepOutRow(chain, fk, cap, jointIdx) {
	const P = fk.worldPos[jointIdx]
	// closest point on the collider: a segment for a capsule, the centre for a sphere
	let axisPt
	if (cap.kind === 'sphere') axisPt = cap.c
	else { const ab = sub3(cap.b, cap.a); const tt = Math.max(0, Math.min(1, dot3(sub3(P, cap.a), ab) / (dot3(ab, ab) || 1))); axisPt = add3(cap.a, mul3(ab, tt)) }
	const radial = sub3(P, axisPt) // from the collider surface-origin out to the joint
	const d = len3(radial)
	const grad = new Float64Array(chain.dof.length)
	if (d >= cap.radius) return { C: 0, grad, compliance: KEEPOUT_COMPLIANCE } // outside → inactive (no-op)
	const n = d > 1e-5 ? [radial[0] / d, radial[1] / d, radial[2] / d] : [1, 0, 0] // push direction (radial out)
	const mask = new Array(chain.joints.length).fill(false)
	for (let i = jointIdx; i >= 0; i = chain.joints[i].parentIndex) mask[i] = true
	for (let k = 0; k < chain.dof.length; k++) {
		if (!mask[chain.dof[k].joint]) continue
		const col = cross3(fk.axisWorld[k], sub3(P, fk.pivot[k]))
		grad[k] = n[0] * col[0] + n[1] * col[1] + n[2] * col[2] // d(distance-from-axis)/dθ
	}
	return { C: d - cap.radius, grad, compliance: KEEPOUT_COMPLIANCE } // C < 0 (penetrating) → driven out to the surface
}

export function restRow(chain, d) {
	const { joint, axis } = chain.dof[d]
	const K = chain.joints[joint].restStiffness[axis]
	if (K <= 0) return null
	const grad = new Float64Array(chain.dof.length); grad[d] = 1
	return { C: chain.theta[d] - chain.joints[joint].restAngle[axis], grad, compliance: 1 / K }
}

export function limitRow(chain, d) {
	const { joint, axis } = chain.dof[d]
	const lim = chain.joints[joint].limit[axis]
	if (lim.mode !== 'limited') return null
	const t = chain.theta[d]
	let C = 0
	if (t > lim.max) C = t - lim.max
	else if (t < lim.min) C = t - lim.min
	// inactive (in-range): C=0, grad=0 → projection relaxes this row's multiplier to 0, a no-op.
	const grad = new Float64Array(chain.dof.length); if (C !== 0) grad[d] = 1
	return { C, grad, compliance: LIMIT_COMPLIANCE }
}
