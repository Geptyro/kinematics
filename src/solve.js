/**
 * Constraint-relaxation pose solver — the shared kernel (bake + runtime). Pure: numbers and
 * flat arrays only, no THREE, no frame knowledge (hosts convert coordinate frames at their own
 * adapter boundary). See animation-ik-design.md.
 *
 * `solvePose(chain, goals, solver?)` poses a joint chain so its effectors meet their goals while
 * each joint resists motion (W = rotStiffness) and is pulled toward its preferred angle (K =
 * restStiffness), within joint limits. The method is XPBD: a FIXED number of Gauss-Seidel sweeps
 * over compliant constraints — no residual-based early-exit and per-constraint multipliers reset
 * once per solve, so the result is reproducible (protects byte-identical bakes). State (chain.theta)
 * persists for WARM-STARTING the next frame, which is both the temporal-coherence and the speed win.
 *
 * FAST PATH: a free 3-joint chain with a single reach goal IS the analytic 2-bone problem, so it is
 * detected and routed to the closed-form `solveTwoBonePositions` (byte-identical to the legacy path).
 */
import { axisAngle, fromTo, solveTwoBonePositions } from './index.js'
import { IDENT3, add3, sub3, mul3, cross3, len3, norm3, clamp, matVec3, mat3mul, transpose3 } from './vec.js'
import { reachRow, poleRow, aimRow, restRow, limitRow, keepOutRow } from './constraints.js'

// ── chain state (DOF list + warm-started angles) ──────────────────────────────
function buildDof(chain) {
	if (chain.dof) return
	const dof = []
	const dofIndex = chain.joints.map((j) => j.axes.map(() => -1))
	for (let i = 0; i < chain.joints.length; i++) {
		const j = chain.joints[i]
		for (let k = 0; k < j.axes.length; k++) {
			if (j.limit[k].mode === 'locked') continue // locked axis = true DOF removal
			dofIndex[i][k] = dof.length
			dof.push({ joint: i, axis: k })
		}
	}
	chain.dof = dof
	chain.dofIndex = dofIndex
}

function ensureTheta(chain) {
	buildDof(chain)
	if (chain.theta && chain.theta.length === chain.dof.length) return // warm-start: keep prior angles
	const t = new Float64Array(chain.dof.length)
	for (let d = 0; d < chain.dof.length; d++) { const { joint, axis } = chain.dof[d]; t[d] = chain.joints[joint].restAngle[axis] }
	chain.theta = t
}

function angleAt(chain, i, k) { const di = chain.dofIndex[i][k]; return di >= 0 ? chain.theta[di] : chain.joints[i].restAngle[k] }

/** Forward kinematics + per-DOF world rotation axes/pivots (the geometric Jacobian columns). */
function forwardK(chain) {
	const n = chain.joints.length
	const worldPos = new Array(n), worldRot = new Array(n)
	const axisWorld = new Array(chain.dof.length), pivot = new Array(chain.dof.length)
	for (let i = 0; i < n; i++) {
		const j = chain.joints[i]
		const p = j.parentIndex
		const pRot = p < 0 ? chain.rootFrame.rot : worldRot[p]
		const pPos = p < 0 ? chain.rootFrame.pos : worldPos[p]
		worldPos[i] = add3(pPos, matVec3(pRot, j.restOffset))
		let partial = pRot // = pRot · (product of axis rotations before axis k)
		let local = IDENT3
		for (let k = 0; k < j.axes.length; k++) {
			const di = chain.dofIndex[i][k]
			if (di >= 0) { axisWorld[di] = matVec3(partial, j.axes[k]); pivot[di] = worldPos[i] }
			const R = axisAngle(j.axes[k], angleAt(chain, i, k))
			local = mat3mul(local, R)
			partial = mat3mul(partial, R)
		}
		worldRot[i] = mat3mul(pRot, local)
	}
	return { worldPos, worldRot, axisWorld, pivot }
}

/** Public FK: per-joint world position + rotation, honoring the fast-path override when present. */
export function composeFK(chain) {
	const locals = chainToLocalRot(chain)
	const n = chain.joints.length
	const worldPos = new Array(n), worldRot = new Array(n)
	for (let i = 0; i < n; i++) {
		const p = chain.joints[i].parentIndex
		const pRot = p < 0 ? chain.rootFrame.rot : worldRot[p]
		const pPos = p < 0 ? chain.rootFrame.pos : worldPos[p]
		worldPos[i] = add3(pPos, matVec3(pRot, chain.joints[i].restOffset))
		worldRot[i] = mat3mul(pRot, locals[i])
	}
	return { worldPos, worldRot }
}

/** Per-joint LOCAL rotation (row-major mat3) — fast-path override if set, else composed from theta. */
export function chainToLocalRot(chain) {
	if (chain.localRot) return chain.localRot
	ensureTheta(chain)
	const out = new Array(chain.joints.length)
	for (let i = 0; i < chain.joints.length; i++) {
		let local = IDENT3
		for (let k = 0; k < chain.joints[i].axes.length; k++) local = mat3mul(local, axisAngle(chain.joints[i].axes[k], angleAt(chain, i, k)))
		out[i] = local
	}
	return out
}

// ── XPBD projection of one scalar constraint row ──────────────────────────────
function project(chain, row, lambda, s) {
	const grad = row.grad
	let denom = row.compliance
	for (let d = 0; d < chain.dof.length; d++) {
		const g = grad[d]
		if (g === 0) continue
		const j = chain.dof[d]
		denom += (1 / chain.joints[j.joint].rotStiffness[j.axis]) * g * g
	}
	if (denom === 0) return
	const dl = (-row.C - row.compliance * lambda[s]) / denom
	lambda[s] += dl
	for (let d = 0; d < chain.dof.length; d++) {
		const g = grad[d]
		if (g === 0) continue
		const j = chain.dof[d]
		chain.theta[d] += (1 / chain.joints[j.joint].rotStiffness[j.axis]) * g * dl
	}
}

function maxReachResidual(chain, fk, goals) {
	let r = 0
	for (const goal of goals) {
		if (goal.kind !== 'reach' || goal.posWeight <= 0) continue
		const t = fk.worldPos[goal.jointIndex]
		r = Math.max(r, len3(sub3(t, goal.target)))
	}
	return r
}

/**
 * Pose `chain` to satisfy `goals`. Mutates chain.theta (or chain.localRot on the fast path).
 * @param {object} chain  - from makeChain (carries warm-started state)
 * @param {object[]} goals - from makeGoal
 * @param {object} [solver] - { iterations } overrides chain.iterations
 * @returns {{ residual:number }} max reach residual (diagnostics only — never a termination test)
 */
export function solvePose(chain, goals, solver) {
	if (fastPathApplies(chain, goals)) return analyticReach(chain, goals[0])
	for (const g of goals) if (g.maxBend !== null || g.levelEnd !== null) throw new Error('solvePose: maxBend/levelEnd are only supported on the analytic 2-bone fast path')
	chain.localRot = null
	ensureTheta(chain)
	const iters = (solver && solver.iterations) ? solver.iterations : chain.iterations

	// Fixed, stable descriptor order (Gauss-Seidel order is load-bearing → documented):
	// reach (pos) → aim/level (orient) → rest springs → joint limits. lambda is per-descriptor,
	// reset to 0 here (once per solve) and accumulated across sweeps — the XPBD step-independence.
	const desc = []
	for (const goal of goals) if (goal.kind === 'reach' && goal.posWeight > 0) for (let a = 0; a < 3; a++) desc.push({ kind: 'reach', goal, comp: a })
	for (const goal of goals) if (goal.kind === 'reach' && (goal.pole || typeof goal.swivel === 'number')) for (let a = 0; a < 3; a++) desc.push({ kind: 'pole', goal, comp: a }) // swivel the elbow/knee (pole vector or swivel angle)
	for (const goal of goals) if ((goal.kind === 'aim' || goal.kind === 'level') && goal.rotWeight > 0) for (let a = 0; a < 3; a++) desc.push({ kind: 'aim', goal, comp: a })
	for (let d = 0; d < chain.dof.length; d++) { const j = chain.dof[d]; if (chain.joints[j.joint].restStiffness[j.axis] > 0) desc.push({ kind: 'rest', dof: d }) }
	for (let d = 0; d < chain.dof.length; d++) { const j = chain.dof[d]; if (chain.joints[j.joint].limit[j.axis].mode === 'limited') desc.push({ kind: 'limit', dof: d }) }
	if (solver && solver.keepout) for (const cap of solver.keepout.colliders) for (const ji of solver.keepout.joints) desc.push({ kind: 'keepout', joint: ji, cap }) // push protected joints out of each body collider
	const lambda = new Float64Array(desc.length)

	for (let it = 0; it < iters; it++) {
		for (let s = 0; s < desc.length; s++) {
			const fk = forwardK(chain) // true Gauss-Seidel: refresh FK before each projection
			const row = evalDescriptor(chain, fk, desc[s])
			if (row) project(chain, row, lambda, s)
		}
	}
	return { residual: maxReachResidual(chain, forwardK(chain), goals) }
}

function evalDescriptor(chain, fk, d) {
	if (d.kind === 'reach') return reachRow(chain, fk, d.goal, d.comp)
	if (d.kind === 'pole') return poleRow(chain, fk, d.goal, d.comp)
	if (d.kind === 'aim') return aimRow(chain, fk, d.goal, d.comp)
	if (d.kind === 'rest') return restRow(chain, d.dof)
	if (d.kind === 'limit') return limitRow(chain, d.dof)
	if (d.kind === 'keepout') return keepOutRow(chain, fk, d.cap, d.joint)
	throw new Error('solvePose: unknown constraint kind ' + d.kind)
}

// ── analytic 2-bone fast path (byte-identical to the legacy reachChain math) ───
function fastPathApplies(chain, goals) {
	if (goals.length !== 1) return false
	const g = goals[0]
	if (g.kind !== 'reach' || g.posWeight <= 0 || g.jointIndex !== 2 || !g.pole) return false
	const j = chain.joints
	if (j.length !== 3 || j[0].parentIndex !== -1 || j[1].parentIndex !== 0 || j[2].parentIndex !== 1) return false
	// Every joint must be a fully-free ball (3 axes, no limits, no rest bias) so the analytic
	// solution IS the XPBD fixed point and the solved rotations are representable.
	for (const b of j) {
		if (b.axes.length !== 3) return false
		for (let k = 0; k < 3; k++) { if (b.restStiffness[k] > 0) return false; if (b.limit[k].mode !== 'free') return false }
	}
	return true
}

function analyticReach(chain, goal) {
	const j0 = chain.joints[0], j1 = chain.joints[1], j2 = chain.joints[2]
	const Pc = chain.rootFrame.rot
	const S = add3(chain.rootFrame.pos, matVec3(Pc, j0.restOffset)) // upper-joint (shoulder) world
	const e0 = j1.restOffset, h0 = j2.restOffset
	const L1 = len3(e0), L2 = len3(h0)
	const u0 = norm3(e0), v0 = norm3(h0)
	const PcT = transpose3(Pc)
	const d = matVec3(PcT, sub3(goal.target, S)) // target in the parent-local frame
	let c = clamp(len3(d), 1e-4, (L1 + L2) * 0.999)
	if (goal.maxBend !== null) c = Math.min(c, Math.sqrt(Math.max(0, L1 * L1 + L2 * L2 - 2 * L1 * L2 * Math.cos(goal.maxBend))))
	const dn = norm3(d)
	const poleL = matVec3(PcT, goal.pole)
	let nrm = cross3(dn, poleL)
	if (len3(nrm) < 1e-5) nrm = cross3(dn, Math.abs(dn[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0])
	nrm = norm3(nrm)
	const sol = solveTwoBonePositions({ base: [0, 0, 0], target: mul3(dn, c), up: dn, forward: norm3(cross3(dn, nrm)), lengths: [L1, L2] })
	const Ra = fromTo(u0, sol.dir0)
	const Rf = fromTo(v0, matVec3(transpose3(Ra), sol.dir1))
	let Rh = IDENT3
	if (goal.levelEnd !== null) {
		const lowerWorld = mat3mul(mat3mul(Pc, Ra), Rf) // end bone's parent world rot
		const goalWorld = Array.isArray(goal.levelEnd) ? fromTo([0, 1, 0], goal.levelEnd) : IDENT3
		Rh = mat3mul(transpose3(lowerWorld), goalWorld)
	}
	chain.localRot = [Ra, Rf, Rh]
	const tip = add3(S, matVec3(Pc, mul3(dn, c)))
	return { residual: len3(sub3(tip, goal.target)) }
}
