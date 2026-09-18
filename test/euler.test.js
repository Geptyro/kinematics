/**
 * `matFromEulerXYZ` is the stated inverse of `matToEulerXYZ`, and "stated" is not
 * good enough for a pair a runtime IK solver reads a bone with and writes it back
 * with: get one of them transposed and every solved arm is subtly mirrored while
 * every unit test that only ever goes one way stays green.
 *
 * So both directions are pinned. THE ROUND TRIP is the weaker half — a wrong-but-
 * self-consistent pair passes it — so the first test compares against three's own
 * composition rule written out longhand, which is where the convention actually
 * comes from.
 */
import { test } from 'node:test'
import assert from 'node:assert/strict'
import { matFromEulerXYZ, matToEulerXYZ } from '../src/index.js'

/** three's makeRotationFromEuler(order:'XYZ') is Rx·Ry·Rz. Composed the long way. */
function rxryrz(x, y, z) {
	const mul = (a, b) => {
		const o = new Array(9)
		for (let r = 0; r < 3; r++) {
			for (let c = 0; c < 3; c++) {
				o[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c]
			}
		}
		return o
	}
	const cx = Math.cos(x), sx = Math.sin(x)
	const cy = Math.cos(y), sy = Math.sin(y)
	const cz = Math.cos(z), sz = Math.sin(z)
	const Rx = [1, 0, 0, 0, cx, -sx, 0, sx, cx]
	const Ry = [cy, 0, sy, 0, 1, 0, -sy, 0, cy]
	const Rz = [cz, -sz, 0, sz, cz, 0, 0, 0, 1]
	return mul(mul(Rx, Ry), Rz)
}

const ANGLES = [
	[0, 0, 0],
	[0.3, -0.2, 0.1],
	[-1.374473, -0.892896, -0.248317],   // the marine's shouldered off shoulder
	[1.2, 1.5, -2.9],                    // close to the +Y gimbal
	[-2.9, -1.5, 2.9],                   // and the -Y one
	[0.7, 0, -3.0],
]

test('matFromEulerXYZ IS three.js Rx·Ry·Rz, element for element', () => {
	for (const [x, y, z] of ANGLES) {
		const got = matFromEulerXYZ(x, y, z)
		const want = rxryrz(x, y, z)
		for (let i = 0; i < 9; i++) {
			assert.ok(Math.abs(got[i] - want[i]) < 1e-12,
				`[${x}, ${y}, ${z}] element ${i}: ${got[i]} vs ${want[i]}`)
		}
	}
})

test('it round-trips through matToEulerXYZ', () => {
	for (const [x, y, z] of ANGLES) {
		// Away from the gimbal the euler comes back as itself; at it, only the
		// MATRIX has to come back, which is the property anything downstream uses.
		const m = matFromEulerXYZ(x, y, z)
		const e = matToEulerXYZ(m)
		const back = matFromEulerXYZ(e.x, e.y, e.z)
		for (let i = 0; i < 9; i++) {
			assert.ok(Math.abs(back[i] - m[i]) < 1e-12,
				`[${x}, ${y}, ${z}] did not round-trip: element ${i} ${back[i]} vs ${m[i]}`)
		}
	}
})
