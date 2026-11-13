import {Vec2, Vec3, addVec2,addVec3,subVec2,subVec3,multVec2,multVec3,divVec2,divVec3,multVec22,multVec32,divVec22,divVec32,dist2, dist3, URD2, URD3, norm2, norm3, dot2, dot3, reflect2, reflect3} from "./vec.js"

import { Rigid2, Rigid3 } from "./rigid.js"
import { Obj2, Obj3 } from "./obj.js"
export class BarB2{
    constructor(b1, b2, len, stiffness){
        this.b1=b1
        this.b2=b2
        this.len=len
        this.stiffness=stiffness
    }
    solve(){
        const b1 = this.b1
        const b2 = this.b2
        const dv = subVec2(b2.p, b1.p)
        const d = Math.hypot(dv.x, dv.y)
        if (d < 1e-6) return
        const inv1 = b1.f ? 0 : 1 / Math.max(b1.w || 1, 1);
        const inv2 = b2.f ? 0 : 1 / Math.max(b2.w || 1, 1);
        const invSum = inv1 + inv2;
        if (invSum === 0) return;
        const diff = (d - bar.l) / d;
        const stiffness = this.stiffness;
        const maxCorrection = 5;
        let c = multVec22(multVec22(dv, diff), stiffness)
        const corrLen = Math.hypot(c.x,c.y)
        if (corrLen > maxCorrection) {
            const corrScale = maxCorrection / corrLen
            c = multVec22(c, corrScale)
        }
        const move1 = inv1 / invSum
        const move2 = inv2 / invSum
        b1.p = multVec22(c, move1)
        b1.pp = multVec22(c, move1)
        b2.p = multVec22(c, move2)
        b2.pp = multVec22(c, move2)
        const n = divVec22(dv, d)
        const v1 = subVec2(b1.p - b1.pp)
        const v2 = subVec2(b2.p - b2.pp)
        const relVel = (v2.x - v1.x) * n.x + (v2.y - v1.y) * n.y // I'm too lazy to actually adapt this into vec.js functions
        const dv1 = multVec22(multVec22(n, relVel), move1)
        const dv2 = multVec22(multVec22(n, relVel), -move2)
        b1.pp = subVec2(b1.pp, dv1)
        b2.pp = subVec2(b2.pp, dv2)
        
    }
}
export class BarB3{
    constructor(b1, b2, len, stiffness){
        this.b1=b1
        this.b2=b2
        this.len=len
        this.stiffness=stiffness
    }
    solve(){
        const b1 = this.b1
        const b2 = this.b2
        const dv = subVec3(b2.p, b1.p)
        const d = Math.hypot(dv.x, dv.y, dv.z)
        if (d < 1e-6) return
        const inv1 = b1.f ? 0 : 1 / Math.max(b1.w || 1, 1);
        const inv2 = b2.f ? 0 : 1 / Math.max(b2.w || 1, 1);
        const invSum = inv1 + inv2;
        if (invSum === 0) return;
        const diff = (d - bar.l) / d;
        const stiffness = this.stiffness;
        const maxCorrection = 5;
        let c = multVec32(multVec32(dv, diff), stiffness)
        const corrLen = Math.hypot(cx,cy,cz)
        if (corrLen > maxCorrection) {
            const corrScale = maxCorrection / corrLen
            c = multVec32(c, corrScale)
        }
        const move1 = inv1 / invSum
        const move2 = inv2 / invSum
        b1.p = multVec32(c, move1)
        b1.pp = multVec32(c, move1)
        b2.p = multVec32(c, move2)
        b2.pp = multVec32(c, move2)
        const n = divVec32(dv, d)
        const v1 = subVec3(b1.p - b1.pp)
        const v2 = subVec3(b2.p - b2.pp)
        const relVel = (v2.x - v1.x) * n.x + (v2.y - v1.y) * n.y + (v2.z - v1.z) * n.z // I'm too lazy to actually adapt this into vec.js functions
        const dv1 = multVec32(multVec32(n, relVel), move1)
        const dv2 = multVec32(multVec32(n, relVel), -move2)
        b1.pp = subVec3(b1.pp, dv1)
        b2.pp = subVec3(b2.pp, dv2)
        
    }
}