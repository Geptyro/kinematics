/**
 * Pure vector / row-major-3x3-matrix helpers for the constraint-relaxation solver.
 * Numbers and flat arrays only — no THREE, no deps. Row-major matrices match the
 * convention in index.js (axisAngle/fromTo return row-major; matVec3 reads rows).
 *
 * NOTE: index.js keeps its OWN private copies of dot3/cross3/etc so the proven analytic
 * 2-bone path stays byte-for-byte untouched; this module serves the NEW solver files.
 */

export const IDENT3 = [1, 0, 0, 0, 1, 0, 0, 0, 1]

export function add3(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
export function sub3(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
export function mul3(a, s) { return [a[0] * s, a[1] * s, a[2] * s] }
export function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
export function cross3(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] }
export function len3(a) { return Math.hypot(a[0], a[1], a[2]) }
export function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)) }

/** Normalize; returns the zero vector unchanged (callers guard degenerate cases explicitly). */
export function norm3(a) { const l = len3(a); if (l === 0) return [0, 0, 0]; return [a[0] / l, a[1] / l, a[2] / l] }

/** Row-major 3x3 times column vector. */
export function matVec3(m, v) {
	return [
		m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
		m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
		m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
	]
}

/** Row-major 3x3 matrix product A·B. */
export function mat3mul(a, b) {
	return [
		a[0] * b[0] + a[1] * b[3] + a[2] * b[6], a[0] * b[1] + a[1] * b[4] + a[2] * b[7], a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
		a[3] * b[0] + a[4] * b[3] + a[5] * b[6], a[3] * b[1] + a[4] * b[4] + a[5] * b[7], a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
		a[6] * b[0] + a[7] * b[3] + a[8] * b[6], a[6] * b[1] + a[7] * b[4] + a[8] * b[7], a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
	]
}

export function transpose3(m) { return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]] }

/**
 * Matrix log of a row-major rotation matrix → the rotation VECTOR (unit axis × angle).
 * Used for orientation residuals (aim/level goals). Returns [0,0,0] at identity; the
 * near-π branch recovers the axis from the symmetric part (R + I) since the skew part
 * vanishes there.
 */
export function logSO3(R) {
	const tr = R[0] + R[4] + R[8]
	const c = clamp((tr - 1) / 2, -1, 1)
	const ang = Math.acos(c)
	if (ang < 1e-7) return [0, 0, 0]
	if (ang > Math.PI - 1e-6) {
		// near π: axis from the largest diagonal of (R + I)/2 = axis·axisᵀ
		const xx = (R[0] + 1) / 2, yy = (R[4] + 1) / 2, zz = (R[8] + 1) / 2
		let ax
		if (xx >= yy && xx >= zz) { const x = Math.sqrt(Math.max(0, xx)); ax = [x, (R[1] + R[3]) / 4 / (x || 1), (R[2] + R[6]) / 4 / (x || 1)] }
		else if (yy >= zz) { const y = Math.sqrt(Math.max(0, yy)); ax = [(R[1] + R[3]) / 4 / (y || 1), y, (R[5] + R[7]) / 4 / (y || 1)] }
		else { const z = Math.sqrt(Math.max(0, zz)); ax = [(R[2] + R[6]) / 4 / (z || 1), (R[5] + R[7]) / 4 / (z || 1), z] }
		const a = norm3(ax)
		return [a[0] * ang, a[1] * ang, a[2] * ang]
	}
	const k = ang / (2 * Math.sin(ang))
	return [(R[7] - R[5]) * k, (R[2] - R[6]) * k, (R[3] - R[1]) * k]
}
