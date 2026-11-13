import {Vec2, Vec3, addVec2,addVec3,subVec2,subVec3,multVec2,multVec3,divVec2,divVec3,multVec22,multVec32,divVec22,divVec32,dist2, dist3, URD2, URD3, norm2, norm3, dot2, dot3, reflect2, reflect3} from "./vec.js"

import { Rigid2, Rigid3 } from "./rigid.js"
import { Obj2, Obj3 } from "./obj.js"
export class BarB2 {
    constructor(b1, b2, len, stiffness = 1) {
        this.b1 = b1
        this.b2 = b2
        this.len = len
        this.stiffness = stiffness
    }

    solve() {
        const b1 = this.b1
        const b2 = this.b2
        if (!b1 || !b2) return
        const dv = subVec2(b2.p, b1.p)
        const d = Math.hypot(dv.x, dv.y)
        if (d < 1e-6) return
        const m1 = b1.m ?? b1.w ?? 1
        const m2 = b2.m ?? b2.w ?? 1
        const inv1 = b1.f ? 0 : 1 / Math.max(m1, 1e-4)
        const inv2 = b2.f ? 0 : 1 / Math.max(m2, 1e-4)
        const invSum = inv1 + inv2
        if (invSum === 0) return
        const diff = (d - this.len) / d
        const maxCorrection = 5
        let c = multVec22(dv, diff * this.stiffness);
        const corrLen = Math.hypot(c.x, c.y)
        if (corrLen > maxCorrection) {
            c = multVec22(c, maxCorrection / corrLen)
        }
        const move1 = inv1 / invSum
        const move2 = inv2 / invSum
        const offset1 = multVec22(c, move1)
        const offset2 = multVec22(c, move2)
        b1.p = addVec2(b1.p, offset1)
        b1.pp = addVec2(b1.pp, offset1)
        b2.p = subVec2(b2.p, offset2)
        b2.pp = subVec2(b2.pp, offset2)
        const n = divVec22(dv, d)
        const v1 = subVec2(b1.p, b1.pp)
        const v2 = subVec2(b2.p, b2.pp)
        const relVel = dot2(subVec2(v2, v1), n)
        const dv1 = multVec22(n, relVel * move1)
        const dv2 = multVec22(n, -relVel * move2)
        b1.pp = subVec2(b1.pp, dv1)
        b2.pp = subVec2(b2.pp, dv2)
    }
}

export class BarR2 {
    constructor(p1, p2, pivot1, pivot2, stiffness = 1, len = null) {
        this.p1 = p1
        this.p2 = p2
        const m1 = p1.pts[0]?.m ?? 1
        const m2 = p2.pts[0]?.m ?? 1
        this.pivot1 = new Obj2({
            x: pivot1.x,
            y: pivot1.y,
            rad: 0,
            fixed: p1.f,
            mass: m1
        })
        this.pivot2 = new Obj2({
            x: pivot2.x,
            y: pivot2.y,
            rad: 0,
            fixed: p2.f,
            mass: m2
        })
        const initialDist = Math.hypot(pivot2.x - pivot1.x, pivot2.y - pivot1.y)
        this.len = len ?? initialDist
        this.stiffness = stiffness
        this.bars = []
        for (const pt of p1.pts) {
            const d = Math.hypot(pt.p.x - this.pivot1.p.x, pt.p.y - this.pivot1.p.y)
            this.bars.push(new BarB2(pt, this.pivot1, d, 1))
        }
        for (const pt of p2.pts) {
            const d = Math.hypot(pt.p.x - this.pivot2.p.x, pt.p.y - this.pivot2.p.y)
            this.bars.push(new BarB2(pt, this.pivot2, d, 1))
        }
        this.mainBar = new BarB2(this.pivot1, this.pivot2, this.len, this.stiffness)
        this.bars.push(this.mainBar)
    }
    integrate(gravity) {
        this.pivot1.f = this.p1.f
        this.pivot2.f = this.p2.f
        this.pivot1.integrate(gravity)
        this.pivot2.integrate(gravity)
    }
    solve() {
        this.pivot1.f = this.p1.f
        this.pivot2.f = this.p2.f
        for (const bar of this.bars) {
            bar.solve()
        }
    }
}

export class BarB3 {
    constructor(b1, b2, len, stiffness = 1) {
        this.b1 = b1
        this.b2 = b2
        this.len = len
        this.stiffness = stiffness
    }

    solve() {
        const b1 = this.b1
        const b2 = this.b2
        if (!b1 || !b2) return
        const dv = subVec3(b2.p, b1.p)
        const d = Math.hypot(dv.x, dv.y, dv.z)
        if (d < 1e-6) return
        const m1 = b1.m ?? b1.w ?? 1
        const m2 = b2.m ?? b2.w ?? 1
        const inv1 = b1.f ? 0 : 1 / Math.max(m1, 1e-4)
        const inv2 = b2.f ? 0 : 1 / Math.max(m2, 1e-4)
        const invSum = inv1 + inv2
        if (invSum === 0) return
        const diff = (d - this.len) / d
        const maxCorrection = 5
        let c = multVec32(dv, diff * this.stiffness);
        const corrLen = Math.hypot(c.x, c.y, c.z)
        if (corrLen > maxCorrection) {
            c = multVec32(c, maxCorrection / corrLen)
        }
        const move1 = inv1 / invSum
        const move2 = inv2 / invSum
        const offset1 = multVec32(c, move1)
        const offset2 = multVec32(c, move2)
        b1.p = addVec3(b1.p, offset1)
        b1.pp = addVec3(b1.pp, offset1)
        b2.p = subVec3(b2.p, offset2)
        b2.pp = subVec3(b2.pp, offset2)
        const n = divVec32(dv, d)
        const v1 = subVec3(b1.p, b1.pp)
        const v2 = subVec3(b2.p, b2.pp)
        const relVel = dot3(subVec3(v2, v1), n)
        const dv1 = multVec32(n, relVel * move1)
        const dv2 = multVec32(n, -relVel * move2)
        b1.pp = subVec3(b1.pp, dv1)
        b2.pp = subVec3(b2.pp, dv2)
    }
}

export class BarR3 {
    constructor(p1, p2, pivot1, pivot2, stiffness = 1, len = null) {
        this.p1 = p1
        this.p2 = p2
        const m1 = p1.pts[0]?.m ?? 1
        const m2 = p2.pts[0]?.m ?? 1
        this.pivot1 = new Obj3({
            x: pivot1.x,
            y: pivot1.y,
            z: pivot1.z,
            rad: 0,
            fixed: p1.f,
            mass: m1
        })
        this.pivot2 = new Obj3({
            x: pivot2.x,
            y: pivot2.y,
            z: pivot2.z,
            rad: 0,
            fixed: p2.f,
            mass: m2
        })
        const initialDist = Math.hypot(pivot2.x - pivot1.x, pivot2.y - pivot1.y, pivot2.z-pivot1.z)
        this.len = len ?? initialDist
        this.stiffness = stiffness
        this.bars = []
        for (const pt of p1.pts) {
            const d = Math.hypot(pt.p.x - this.pivot1.p.x, pt.p.y - this.pivot1.p.y, pt.p.z-this.pivot1.p.z)
            this.bars.push(new BarB3(pt, this.pivot1, d, 1))
        }
        for (const pt of p2.pts) {
            const d = Math.hypot(pt.p.x - this.pivot2.p.x, pt.p.y - this.pivot2.p.y, py.p.z-this.pivot2.p.z)
            this.bars.push(new BarB3(pt, this.pivot2, d, 1))
        }
        this.mainBar = new BarB3(this.pivot1, this.pivot2, this.len, this.stiffness)
        this.bars.push(this.mainBar)
    }
    integrate(gravity) {
        this.pivot1.f = this.p1.f
        this.pivot2.f = this.p2.f
        this.pivot1.integrate(gravity)
        this.pivot2.integrate(gravity)
    }
    solve() {
        this.pivot1.f = this.p1.f
        this.pivot2.f = this.p2.f
        for (const bar of this.bars) {
            bar.solve()
        }
    }
}