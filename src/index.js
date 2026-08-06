/**
 * kinematics — pure, framework-agnostic kinematics math. No THREE, no engine, no
 * build-tool deps; just numbers and arrays. ONE source of truth for limb IK, shared by:
 *   • the build-time animation baker (mesh-x re-exports these; bake-time IK poses arms
 *     onto a target, then captures the result as morph targets), and
 *   • runtime pose solvers (falcra's IKChain wraps these for live THREE.Bone hierarchies).
 *
 * Convention: angles use the atan2(forward, up) frame — 0 points straight along `up`,
 * +angle rotates toward `forward`. The mid joint (knee/elbow) bends toward +forward of
 * the base→target line.
 */

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)) }

// ── small vector helpers (row-major 3x3 matrices as flat arrays) ──────────────
function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
function cross3(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] }
function len3(a) { return Math.hypot(a[0], a[1], a[2]) }
function norm3(a) { const l = len3(a) || 1; return [a[0] / l, a[1] / l, a[2] / l] }

/**
 * Row-major 3x3 rotation about a UNIT axis by `ang` (Rodrigues). Framework-agnostic.
 */
export function axisAngle(v, ang) {
	const si = Math.sin(ang), co = Math.cos(ang), t = 1 - co, x = v[0], y = v[1], z = v[2]
	return [t * x * x + co, t * x * y - si * z, t * x * z + si * y, t * x * y + si * z, t * y * y + co, t * y * z - si * x, t * x * z - si * y, t * y * z + si * x, t * z * z + co]
}

/**
 * Minimal row-major 3x3 rotation matrix taking UNIT vector `a` → UNIT vector `b`.
 * Handles the parallel (identity) and antiparallel (180° about any perpendicular) cases.
 */
export function fromTo(a, b) {
	const c = clamp(dot3(a, b), -1, 1)
	if (c > 0.999999) return [1, 0, 0, 0, 1, 0, 0, 0, 1]
	let v = cross3(a, b)
	if (len3(v) < 1e-6) { // antiparallel → 180° about any perpendicular axis (R = 2vvᵀ − I)
		const ax = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0]
		v = norm3(cross3(a, ax))
		return [2 * v[0] * v[0] - 1, 2 * v[0] * v[1], 2 * v[0] * v[2], 2 * v[1] * v[0], 2 * v[1] * v[1] - 1, 2 * v[1] * v[2], 2 * v[2] * v[0], 2 * v[2] * v[1], 2 * v[2] * v[2] - 1]
	}
	return axisAngle(norm3(v), Math.acos(c))
}

/**
 * Extract Euler {x,y,z} (three.js 'XYZ' order) from a row-major 3x3 — the inverse of
 * three's makeRotationFromEuler(order:'XYZ') (== mesh-x mat3 `eulerXYZ`). Bake-time bone
 * proxies are Euler; the runtime sets quaternions directly, so it doesn't need this.
 */
export function matToEulerXYZ(m) {
	const ry = Math.asin(clamp(m[2], -1, 1))
	if (Math.abs(m[2]) < 0.9999999) return { x: Math.atan2(-m[5], m[8]), y: ry, z: Math.atan2(-m[1], m[0]) }
	return { x: Math.atan2(m[7], m[4]), y: ry, z: 0 }
}

/**
 * 2-bone analytical IK in a plane (law of cosines). Targets beyond reach clamp to a
 * straight chain.
 * @param {number} l0 - upper segment length (base→mid)
 * @param {number} l1 - lower segment length (mid→tip)
 * @param {number} fwd - target distance along the forward axis
 * @param {number} up - target distance along the up axis
 * @returns {[number, number]} [upperAngle, lowerAngle]
 */
export function solveTwoBonePlanar(l0, l1, fwd, up) {
	const rawD = Math.sqrt(fwd * fwd + up * up)
	const d = Math.min(rawD, l0 + l1 - 0.0001)
	const baseAngle = Math.atan2(fwd, up)
	const cosAlpha = (l0 * l0 + d * d - l1 * l1) / (2 * l0 * d)
	const alpha = Math.acos(clamp(cosAlpha, -1, 1))
	const upperAngle = baseAngle - alpha
	const cosKnee = (l0 * l0 + l1 * l1 - d * d) / (2 * l0 * l1)
	const kneeAngle = Math.acos(clamp(cosKnee, -1, 1))
	const lowerAngle = upperAngle + (Math.PI - kneeAngle)
	return [upperAngle, lowerAngle]
}

/**
 * 2-bone analytical IK for a 3D target. Projects the target into the chain's plane
 * (forward/up) about its base, then delegates to solveTwoBonePlanar.
 * @param {object} cfg - { base:[x,y,z], target:[x,y,z], forward:unit[x,y,z], up:unit[x,y,z], lengths:[l0,l1] }
 * @returns {[number, number]} [upperAngle, lowerAngle]
 */
export function solveTwoBone3D({ base, target, forward, up, lengths }) {
	const dx = target[0] - base[0], dy = target[1] - base[1], dz = target[2] - base[2]
	const fwdDist = dx * forward[0] + dy * forward[1] + dz * forward[2]
	const upDist = dx * up[0] + dy * up[1] + dz * up[2]
	return solveTwoBonePlanar(lengths[0], lengths[1], fwdDist, upDist)
}

/**
 * 2-bone IK that returns world-space JOINT POSITIONS + segment directions (handy for
 * bake-time IK, which needs the bone directions to derive local rotations). Solves in
 * the forward/up plane, so the chain lies in that plane.
 * @param {object} cfg - same as solveTwoBone3D
 * @returns {{ upperAngle, lowerAngle, mid:[x,y,z], tip:[x,y,z], dir0:[x,y,z], dir1:[x,y,z] }}
 *   mid = elbow/knee world pos, tip = hand/foot world pos, dirN = unit segment directions.
 */
export function solveTwoBonePositions({ base, target, forward, up, lengths }) {
	const [a0, a1] = solveTwoBone3D({ base, target, forward, up, lengths })
	const dir = (ang) => [
		Math.cos(ang) * up[0] + Math.sin(ang) * forward[0],
		Math.cos(ang) * up[1] + Math.sin(ang) * forward[1],
		Math.cos(ang) * up[2] + Math.sin(ang) * forward[2],
	]
	const dir0 = dir(a0), dir1 = dir(a1)
	const mid = [base[0] + lengths[0] * dir0[0], base[1] + lengths[0] * dir0[1], base[2] + lengths[0] * dir0[2]]
	const tip = [mid[0] + lengths[1] * dir1[0], mid[1] + lengths[1] * dir1[1], mid[2] + lengths[1] * dir1[2]]
	return { upperAngle: a0, lowerAngle: a1, mid, tip, dir0, dir1 }
}

// ── constraint-relaxation pose solver (XPBD stiffness/rest-bias kernel) ─────────
// The general multi-joint solver. The analytic functions above are its closed-form fast path
// (auto-detected for a free 3-joint single-reach chain). See docs/animation-ik-design.md.
export { makeJoint, makeChain, makeGoal, restAnglesFromEuler } from './chain.js'
export { solvePose, composeFK, chainToLocalRot } from './solve.js'
