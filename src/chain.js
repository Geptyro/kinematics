/**
 * Data model for the constraint-relaxation pose solver (see solve.js). Pure data + fail-loud
 * factories — no THREE, no defaults: every required field throws if absent (project rule:
 * configuration must fail visibly, never silently default).
 *
 * A POSE is solved as a chain of joints, each carrying TWO independent per-axis dials that
 * are the heart of the design:
 *   • rotStiffness (W) — reluctance to MOVE at all (inverse participation in goals)
 *   • restStiffness (K) — pull back toward the joint's DEFAULT/preferred angle ("favor rest")
 * plus per-axis joint limits. A joint's local rotation is the product of axisAngle(axes[k],
 * angle[k]); a `locked` axis is DROPPED from the solved state (true DOF removal), an authored
 * constant. Angles are ABSOLUTE (radians); `restAngle[k]` is the preferred value K pulls toward
 * and the value a locked axis holds, and limits are expressed on the absolute angle.
 *
 * Refinement vs the design doc: joints are specced with explicit per-axis `axes` + `restAngle`
 * (canonical, unambiguous) rather than a single `restEuler`. `restAnglesFromEuler` converts a
 * three.js XYZ Euler to the cardinal-axis form for ball joints when authoring the rig table.
 */
import { len3 } from './vec.js'

function req(cond, msg) { if (!cond) throw new Error(msg) }
const LIMIT_MODES = { free: 1, limited: 1, locked: 1 }

/**
 * @param {object} spec
 * @param {string}   spec.name
 * @param {number}   spec.parentIndex      index into the chain's joints (-1 = chain root → rootFrame)
 * @param {number[]} spec.restOffset       [x,y,z] local translation from parent joint to this one (bone length source)
 * @param {number[][]} spec.axes           1..3 UNIT local rotation axes, in composition order (1 = hinge, 3 = ball)
 * @param {number[]} spec.restAngle        per-axis preferred/default angle (rad); K pulls here, locked axes hold here
 * @param {number[]} spec.rotStiffness     W per axis, > 0 (reluctance to move; inverse participation)
 * @param {number[]} spec.restStiffness    K per axis, >= 0 (pull toward restAngle)
 * @param {object[]} spec.limit            per axis { mode:'free'|'limited'|'locked', min, max, margin } (rad)
 */
export function makeJoint(spec) {
	req(spec, 'makeJoint: spec required')
	req(typeof spec.name === 'string' && spec.name.length > 0, 'makeJoint: name required')
	req(spec.parentIndex !== undefined && spec.parentIndex !== null, `makeJoint(${spec.name}): parentIndex required (-1 for chain root)`)
	req(Array.isArray(spec.restOffset) && spec.restOffset.length === 3, `makeJoint(${spec.name}): restOffset [x,y,z] required`)
	req(Array.isArray(spec.axes) && spec.axes.length >= 1 && spec.axes.length <= 3, `makeJoint(${spec.name}): axes (1..3 unit vectors) required`)
	const nAxes = spec.axes.length
	for (const a of spec.axes) req(Array.isArray(a) && a.length === 3, `makeJoint(${spec.name}): each axis must be [x,y,z]`)
	req(Array.isArray(spec.restAngle) && spec.restAngle.length === nAxes, `makeJoint(${spec.name}): restAngle must match axes length`)
	req(Array.isArray(spec.rotStiffness) && spec.rotStiffness.length === nAxes, `makeJoint(${spec.name}): rotStiffness (W) must match axes length`)
	for (const w of spec.rotStiffness) req(typeof w === 'number' && w > 0, `makeJoint(${spec.name}): rotStiffness (W) must be > 0`)
	req(Array.isArray(spec.restStiffness) && spec.restStiffness.length === nAxes, `makeJoint(${spec.name}): restStiffness (K) must match axes length`)
	for (const k of spec.restStiffness) req(typeof k === 'number' && k >= 0, `makeJoint(${spec.name}): restStiffness (K) must be >= 0`)
	req(Array.isArray(spec.limit) && spec.limit.length === nAxes, `makeJoint(${spec.name}): limit must match axes length`)
	for (const l of spec.limit) {
		req(l && LIMIT_MODES[l.mode], `makeJoint(${spec.name}): limit.mode must be free|limited|locked`)
		if (l.mode === 'limited') { req(typeof l.min === 'number' && typeof l.max === 'number' && typeof l.margin === 'number', `makeJoint(${spec.name}): limited axis needs min/max/margin`) }
	}
	return {
		name: spec.name,
		parentIndex: spec.parentIndex,
		restOffset: spec.restOffset.slice(),
		axes: spec.axes.map((a) => a.slice()),
		restAngle: spec.restAngle.slice(),
		rotStiffness: spec.rotStiffness.slice(),
		restStiffness: spec.restStiffness.slice(),
		limit: spec.limit.map((l) => ({ mode: l.mode, min: l.min, max: l.max, margin: l.margin })),
		len: len3(spec.restOffset), // bone length (parent → this joint)
	}
}

/**
 * @param {object} spec
 * @param {object[]} spec.joints           depth-first; each joint.parentIndex < its own index, [0].parentIndex === -1
 * @param {object}   spec.rootFrame        { pos:[x,y,z], rot: row-major mat3 } world placement of joint[0]'s parent
 * @param {number}   spec.iterations       fixed XPBD sweep budget (no residual-based early-exit → deterministic)
 */
export function makeChain(spec) {
	req(spec, 'makeChain: spec required')
	req(Array.isArray(spec.joints) && spec.joints.length >= 1, 'makeChain: joints required')
	req(spec.joints[0].parentIndex === -1, 'makeChain: joints[0] must be the chain root (parentIndex === -1)')
	for (let i = 1; i < spec.joints.length; i++) req(spec.joints[i].parentIndex >= 0 && spec.joints[i].parentIndex < i, `makeChain: joint ${i} parentIndex must reference an earlier joint`)
	req(spec.rootFrame && Array.isArray(spec.rootFrame.pos) && Array.isArray(spec.rootFrame.rot) && spec.rootFrame.rot.length === 9, 'makeChain: rootFrame { pos:[3], rot: mat3[9] } required')
	req(typeof spec.iterations === 'number' && spec.iterations >= 1, 'makeChain: iterations (>=1) required')
	return {
		joints: spec.joints,
		rootFrame: { pos: spec.rootFrame.pos.slice(), rot: spec.rootFrame.rot.slice() },
		iterations: spec.iterations,
		// solver state (built/warm-started by solvePose):
		dof: null,        // [{ joint, axis }] active (non-locked) DOFs, deterministic order
		dofIndex: null,   // dofIndex[jointIdx][axisIdx] → index into dof/theta, or -1 if locked
		theta: null,      // Float64Array of absolute angles per active DOF (warm-started across solves)
		localRot: null,   // fast-path override: per-joint row-major mat3, bypassing theta
	}
}

const GOAL_KINDS = { reach: 1, aim: 1, level: 1 }

/**
 * @param {object} spec
 * @param {number} spec.jointIndex   effector joint the goal acts on
 * @param {string} spec.kind         'reach' (position) | 'aim' | 'level' (orientation)
 * @param {number[]} spec.target     reach: world [x,y,z]; aim/level: world unit DIRECTION
 * @param {number[]} [spec.localAxis] aim/level: the item/segment lead axis in the joint's local frame
 * @param {number} spec.posWeight    position channel enable/blend (0 = off)
 * @param {number} spec.rotWeight    orientation channel enable/blend (0 = off)
 * @param {number} spec.compliance   1/pullWeight: low = hard pin, high = soft suggestion
 * @param {number} spec.soften       reach-soften band as a ratio of max reach (0..1)
 * @param {number[]} [spec.pole]     reach: REQUIRED bend-plane hint (throws if absent on a short chain)
 * @param {number} [spec.maxBend]    fast-path only: cap the interior reach angle
 * @param {(boolean|number[])} [spec.levelEnd] fast-path only: hold the end bone level / to a normal
 */
export function makeGoal(spec) {
	req(spec, 'makeGoal: spec required')
	req(spec.jointIndex !== undefined && spec.jointIndex !== null, 'makeGoal: jointIndex required')
	req(GOAL_KINDS[spec.kind], 'makeGoal: kind must be reach|aim|level')
	req(typeof spec.posWeight === 'number', 'makeGoal: posWeight required')
	req(typeof spec.rotWeight === 'number', 'makeGoal: rotWeight required')
	req(typeof spec.compliance === 'number' && spec.compliance >= 0, 'makeGoal: compliance (>=0) required')
	req(typeof spec.soften === 'number', 'makeGoal: soften required')
	req(Array.isArray(spec.target) && spec.target.length === 3, 'makeGoal: target [x,y,z] required')
	// Elbow/knee swivel control (optional, on a reach). EITHER a pole VECTOR, OR a SWIVEL ANGLE
	// (the minimal 1-DOF control: angle around the base→tip axis from `swivelRef`). Omit both to
	// leave the bend to the solve.
	if (spec.pole !== undefined) req(Array.isArray(spec.pole) && spec.pole.length === 3, 'makeGoal: pole must be [x,y,z]')
	if (spec.swivel !== undefined) { req(typeof spec.swivel === 'number', 'makeGoal: swivel must be a number (radians)'); req(Array.isArray(spec.swivelRef) && spec.swivelRef.length === 3, 'makeGoal: swivelRef [x,y,z] required when swivel is set') }
	if (spec.kind === 'aim' || spec.kind === 'level') req(Array.isArray(spec.localAxis) && spec.localAxis.length === 3, `makeGoal(${spec.kind}): localAxis [x,y,z] required`)
	return {
		jointIndex: spec.jointIndex,
		kind: spec.kind,
		target: spec.target.slice(),
		localAxis: spec.localAxis ? spec.localAxis.slice() : null,
		posWeight: spec.posWeight,
		rotWeight: spec.rotWeight,
		compliance: spec.compliance,
		soften: spec.soften,
		pole: spec.pole ? spec.pole.slice() : null,
		swivel: spec.swivel === undefined ? null : spec.swivel,
		swivelRef: spec.swivelRef ? spec.swivelRef.slice() : null,
		maxBend: spec.maxBend === undefined ? null : spec.maxBend,
		levelEnd: spec.levelEnd === undefined ? null : spec.levelEnd,
	}
}

/**
 * Convert a three.js XYZ-order Euler {x,y,z} to the canonical cardinal-axis `axes`+`restAngle`
 * pair for a ball joint, so the rig table can author a preferred pose as an Euler.
 * (axisAngle(X,x)·axisAngle(Y,y)·axisAngle(Z,z) == eulerXYZ(x,y,z).)
 */
export function restAnglesFromEuler(euler) {
	req(euler && typeof euler.x === 'number' && typeof euler.y === 'number' && typeof euler.z === 'number', 'restAnglesFromEuler: {x,y,z} required')
	return { axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], restAngle: [euler.x, euler.y, euler.z] }
}
