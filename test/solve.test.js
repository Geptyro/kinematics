/**
 * Tests for the constraint-relaxation pose solver (Phase 0/1). Uses Node's built-in test
 * runner (`node --test`) so `kinematics` stays dependency-free.
 *
 * The load-bearing test is EQUIVALENCE: a free 3-joint single-reach chain must route through
 * the analytic 2-bone fast path and produce local rotations byte-identical to the legacy
 * reachChain math — this is the gate that lets Phase 2 swap reachChain onto solvePose without
 * changing any baked GLB.
 */
import { test } from 'node:test'
import assert from 'node:assert/strict'
import {
	makeJoint, makeChain, makeGoal, restAnglesFromEuler,
	solvePose, composeFK, chainToLocalRot, solveTwoBonePositions, fromTo,
} from '../src/index.js'
import { add3, sub3, mul3, cross3, len3, norm3, matVec3, transpose3 } from '../src/vec.js'

const IDENT = [1, 0, 0, 0, 1, 0, 0, 0, 1]
const free = () => ({ mode: 'free' })

// A fully-free ball joint (the legacy reach uses these).
function ball(name, parentIndex, restOffset, opts = {}) {
	return makeJoint({
		name, parentIndex, restOffset,
		axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
		restAngle: opts.restAngle || [0, 0, 0],
		rotStiffness: opts.W || [1, 1, 1],
		restStiffness: opts.K || [0, 0, 0],
		limit: opts.limit || [free(), free(), free()],
	})
}

function armChain(rootFrame, iterations = 8, opts = {}) {
	return makeChain({
		joints: [
			ball('Arm', -1, [0, 0, 0], opts.j0),
			ball('Forearm', 0, [0, 1, 0], opts.j1),
			ball('Hand', 1, [0, 1, 0], opts.j2),
		],
		rootFrame, iterations,
	})
}

const matClose = (a, b, eps) => { for (let i = 0; i < 9; i++) assert.ok(Math.abs(a[i] - b[i]) <= eps, `m[${i}] ${a[i]} vs ${b[i]}`) }

// Independent re-statement of the legacy reachChain math (poseIk.js), to cross-check the kernel.
function legacyReach(rootFrame, off0, off1, off2, target, pole) {
	const Pc = rootFrame.rot
	const S = add3(rootFrame.pos, matVec3(Pc, off0))
	const L1 = len3(off1), L2 = len3(off2), u0 = norm3(off1), v0 = norm3(off2)
	const PcT = transpose3(Pc)
	const d = matVec3(PcT, sub3(target, S))
	const c = Math.max(1e-4, Math.min(len3(d), (L1 + L2) * 0.999))
	const dn = norm3(d)
	let nrm = cross3(dn, matVec3(PcT, pole))
	if (len3(nrm) < 1e-5) nrm = cross3(dn, Math.abs(dn[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0])
	nrm = norm3(nrm)
	const sol = solveTwoBonePositions({ base: [0, 0, 0], target: mul3(dn, c), up: dn, forward: norm3(cross3(dn, nrm)), lengths: [L1, L2] })
	const Ra = fromTo(u0, sol.dir0)
	const Rf = fromTo(v0, matVec3(transpose3(Ra), sol.dir1))
	return { Ra, Rf }
}

test('fast path: reaches the target exactly (in range)', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const chain = armChain(rootFrame)
	const target = [0.6, 1.2, 0.4] // |.| = 1.4 < 2
	const goal = makeGoal({ jointIndex: 2, kind: 'reach', target, posWeight: 1, rotWeight: 0, compliance: 0.001, soften: 0, pole: [0, 0, 1] })
	const { residual } = solvePose(chain, [goal])
	assert.ok(chain.localRot, 'fast path should set chain.localRot')
	assert.ok(residual < 1e-9, `residual ${residual}`)
	const fk = composeFK(chain)
	assert.ok(len3(sub3(fk.worldPos[2], target)) < 1e-9, 'tip should land on target')
	// bone lengths preserved by the solved rotations
	assert.ok(Math.abs(len3(sub3(fk.worldPos[1], fk.worldPos[0])) - 1) < 1e-9, 'upper length')
	assert.ok(Math.abs(len3(sub3(fk.worldPos[2], fk.worldPos[1])) - 1) < 1e-9, 'lower length')
})

test('fast path: local rotations are byte-identical to the legacy reachChain math', () => {
	// non-identity parent frame to exercise the parent-local transforms
	const rot = fromTo([0, 1, 0], norm3([0.2, 1, 0.1]))
	const rootFrame = { pos: [0.3, 1.0, -0.2], rot }
	const chain = armChain(rootFrame)
	const target = [0.5, 1.6, 0.3]
	const pole = [0.1, 0, 1]
	const goal = makeGoal({ jointIndex: 2, kind: 'reach', target, posWeight: 1, rotWeight: 0, compliance: 0.001, soften: 0, pole })
	solvePose(chain, [goal])
	const { Ra, Rf } = legacyReach(rootFrame, [0, 0, 0], [0, 1, 0], [0, 1, 0], target, pole)
	matClose(chain.localRot[0], Ra, 1e-12)
	matClose(chain.localRot[1], Rf, 1e-12)
	matClose(chain.localRot[2], IDENT, 1e-12)
})

test('fast path: maxBend caps the elbow (shorter effective reach)', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const target = [0, 1.95, 0] // almost straight
	const pole = [0, 0, 1]
	const open = armChain(rootFrame)
	solvePose(open, [makeGoal({ jointIndex: 2, kind: 'reach', target, posWeight: 1, rotWeight: 0, compliance: 0.001, soften: 0, pole })])
	const capped = armChain(rootFrame)
	solvePose(capped, [makeGoal({ jointIndex: 2, kind: 'reach', target, posWeight: 1, rotWeight: 0, compliance: 0.001, soften: 0, pole, maxBend: Math.PI / 2 })])
	const tipOpen = composeFK(open).worldPos[2][1]
	const tipCapped = composeFK(capped).worldPos[2][1]
	assert.ok(tipCapped < tipOpen - 0.05, `maxBend should keep the chain bent: ${tipCapped} vs ${tipOpen}`)
})

test('determinism: identical chains produce identical state', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const goalSpec = { jointIndex: 3, kind: 'reach', target: [0.8, 2.0, 0.5], posWeight: 1, rotWeight: 0, compliance: 0.002, soften: 0.1, pole: [0, 0, 1] }
	const build = () => makeChain({
		joints: [
			ball('A', -1, [0, 0, 0], { K: [0.5, 0.5, 0.5] }),
			ball('B', 0, [0, 0.8, 0], { K: [0.5, 0.5, 0.5] }),
			ball('C', 1, [0, 0.8, 0], { K: [0.5, 0.5, 0.5] }),
			ball('D', 2, [0, 0.6, 0], { K: [0.5, 0.5, 0.5] }),
		],
		rootFrame, iterations: 12,
	})
	const c1 = build(); solvePose(c1, [makeGoal(goalSpec)])
	const c2 = build(); solvePose(c2, [makeGoal(goalSpec)])
	assert.equal(c1.theta.length, c2.theta.length)
	for (let i = 0; i < c1.theta.length; i++) assert.equal(c1.theta[i], c2.theta[i], `theta[${i}]`)
})

test('XPBD: redundant chain converges to a reachable target', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const chain = makeChain({
		joints: [ball('A', -1, [0, 0, 0]), ball('B', 0, [0, 0.8, 0]), ball('C', 1, [0, 0.8, 0]), ball('D', 2, [0, 0.6, 0])],
		rootFrame, iterations: 80,
	})
	const target = [0.7, 1.4, 0.5] // well within total length 2.2
	const { residual } = solvePose(chain, [makeGoal({ jointIndex: 3, kind: 'reach', target, posWeight: 1, rotWeight: 0, compliance: 0.0005, soften: 0, pole: [0, 0, 1] })])
	assert.ok(residual < 3e-2, `residual ${residual}`) // soft pole/swivel adds a small steady-state tip error (~2cm on a 2.2-unit chain)
})

test('rest bias: higher K keeps joints nearer their preferred angle', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const target = [1.2, 1.0, 0.6]
	const goalSpec = { jointIndex: 3, kind: 'reach', target, posWeight: 1, rotWeight: 0, compliance: 0.001, soften: 0, pole: [0, 0, 1] }
	const mk = (K) => makeChain({
		joints: [ball('A', -1, [0, 0, 0], { K }), ball('B', 0, [0, 0.8, 0], { K }), ball('C', 1, [0, 0.8, 0], { K }), ball('D', 2, [0, 0.6, 0], { K })],
		rootFrame, iterations: 60,
	})
	const loose = mk([0.01, 0.01, 0.01]); solvePose(loose, [makeGoal(goalSpec)])
	const stiff = mk([8, 8, 8]); solvePose(stiff, [makeGoal(goalSpec)])
	const dev = (c) => { let s = 0; for (let i = 0; i < c.theta.length; i++) s += Math.abs(c.theta[i]); return s } // restAngle is 0
	assert.ok(dev(stiff) < dev(loose), `stiff dev ${dev(stiff)} should be < loose dev ${dev(loose)}`)
})

test('rest bias: a stiff (K>>W) rest spring relaxes a perturbed pose to rest; a soft one settles partway', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const perturbAndRelax = (K) => {
		const chain = makeChain({ joints: [ball('A', -1, [0, 0, 0], { K: [K, K, K] }), ball('B', 0, [0, 1, 0], { K: [K, K, K] })], rootFrame, iterations: 40 })
		chainToLocalRot(chain) // build dof + theta
		for (let i = 0; i < chain.theta.length; i++) chain.theta[i] = 0.7 // perturb away from rest (0)
		solvePose(chain, []) // no goals → only rest springs act
		return chain.theta[0]
	}
	// XPBD compliant equilibrium for an unopposed spring is θ₀·W/(W+K); W=1 here.
	assert.ok(Math.abs(perturbAndRelax(200)) < 0.05, 'stiff spring should pull (almost) fully to rest')
	assert.ok(Math.abs(perturbAndRelax(200)) < Math.abs(perturbAndRelax(1)), 'stiffer spring relaxes further toward rest')
})

test('joint limit: a hinge stays within [min,max]', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const hinge = makeJoint({ name: 'Hinge', parentIndex: 0, restOffset: [0, 1, 0], axes: [[1, 0, 0]], restAngle: [0], rotStiffness: [1], restStiffness: [0], limit: [{ mode: 'limited', min: 0, max: 1.0, margin: 0.05 }] })
	const chain = makeChain({ joints: [ball('Root', -1, [0, 0, 0]), hinge, ball('Tip', 1, [0, 1, 0])], rootFrame, iterations: 80 })
	// target that would want the hinge to fold well past 1.0 rad
	const target = [0, 0.4, 1.6]
	solvePose(chain, [makeGoal({ jointIndex: 2, kind: 'reach', target, posWeight: 1, rotWeight: 0, compliance: 0.001, soften: 0, pole: [0, 0, 1] })])
	const di = chain.dofIndex[1][0]
	assert.ok(chain.theta[di] <= 1.0 + 1e-3, `hinge angle ${chain.theta[di]} should respect max 1.0`)
	assert.ok(chain.theta[di] >= 0 - 1e-3, `hinge angle ${chain.theta[di]} should respect min 0`)
})

test('locked axis is removed from the solved DOFs', () => {
	const rootFrame = { pos: [0, 0, 0], rot: IDENT }
	const j = makeJoint({ name: 'J', parentIndex: 0, restOffset: [0, 1, 0], axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], restAngle: [0, 0.25, 0], rotStiffness: [1, 1, 1], restStiffness: [0, 0, 0], limit: [free(), { mode: 'locked' }, free()] })
	const chain = makeChain({ joints: [ball('Root', -1, [0, 0, 0]), j, ball('Tip', 1, [0, 1, 0])], rootFrame, iterations: 10 })
	chainToLocalRot(chain)
	assert.equal(chain.dofIndex[1][1], -1, 'locked Y axis should have no DOF')
	assert.ok(chain.dofIndex[1][0] >= 0 && chain.dofIndex[1][2] >= 0, 'free axes keep DOFs')
})

test('factories fail loud on missing config', () => {
	assert.throws(() => makeJoint({ name: 'x', parentIndex: 0, restOffset: [0, 1, 0], axes: [[0, 1, 0]], restAngle: [0], rotStiffness: [1], restStiffness: [0] }), /limit/)
	assert.throws(() => makeJoint({ name: 'x', parentIndex: 0, restOffset: [0, 1, 0], axes: [[0, 1, 0]], restAngle: [0], rotStiffness: [0], restStiffness: [0], limit: [free()] }), /rotStiffness .* > 0/)
	assert.throws(() => makeGoal({ jointIndex: 2, kind: 'reach', target: [0, 1, 0], posWeight: 1, rotWeight: 0, compliance: 0.01 }), /soften/) // soften still required
	assert.throws(() => makeGoal({ jointIndex: 2, kind: 'reach', target: [0, 1, 0], posWeight: 1, rotWeight: 0, compliance: 0.01, soften: 0, pole: [0, 1] }), /pole must be/) // pole optional, but malformed throws
	assert.throws(() => makeChain({ joints: [ball('A', -1, [0, 0, 0])], rootFrame: { pos: [0, 0, 0], rot: IDENT } }), /iterations/)
})

test('restAnglesFromEuler maps to cardinal axes', () => {
	const r = restAnglesFromEuler({ x: 0.1, y: -0.2, z: 0.3 })
	assert.deepEqual(r.axes, [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	assert.deepEqual(r.restAngle, [0.1, -0.2, 0.3])
})
