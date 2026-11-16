import { Obj3 } from "./obj.js"
import { BarB3 } from "./joint.js"
import { Vec3, addVec3, subVec3, multVec3, divVec3, multVec32, divVec32, dist3, URD3, norm3, dot3, reflect3 } from "./vec.js"

export class Cloth{
    constructor (options){
        this.points = []
        this.joints = []
        const ox = options.x ?? 0
        const oy = options.y ?? 0
        const oz = options.z ?? 0
        const w = options.width ?? 20
        const h = options.height ?? 1
        const d = options.depth ?? 20
        const sep = options.seperation ?? 10
        const collisionRadius = options.collisionRadius ?? sep * 0.5
        const sx = ox - (w-1) * 0.5
        const sy = oy - (h-1) * 0.5
        const sz = oz - (d-1) * 0.5
        const stiff = options.stiffness ?? 0.8
        const totMass = options.mass ?? options.weight ?? 50
        const indMass = totMass / (w * h * d)
        for (let z = 0; z < d; z++){
            for (let y = 0; y < h; y++){
                for (let x = 0; x < w; x++){
                    const pt = new Obj3({
                        x: sx + x * sep, y: sy + y * sep, z: sz + z * sep,
                        rad: collisionRadius,
                        bounce: 0,
                        mass: indMass,
                    })
                    this.points.push(pt)
                    const index = this.points.length - 1
                    if (x > 0) {
                        this.joints.push(new BarB3(pt, this.points[index - 1], sep, stiff))
                    }
                    if (y > 0) {
                        this.joints.push(new BarB3(pt, this.points[index - w], sep, stiff))
                    }
                    if (z > 0) {
                        this.joints.push(new BarB3(pt, this.points[index - w * h], sep, stiff))
                    }
                    if (x > 0 && z > 0){
                        this.joints.push(new BarB3(pt, this.points[index - w * h - 1], sep*Math.SQRT2, stiff))
                    }
                    if (x < w - 1 && z > 0) {
                        this.joints.push(new BarB3(pt, this.points[index - w * h + 1], sep*Math.SQRT2, stiff));
                    }
                    const bendSep = sep * 2;
                    const bendStiff = stiff * 0.2;

                    if (x > 1) {
                        this.joints.push(new BarB3(pt, this.points[index - 2], bendSep, bendStiff));
                    }
                    if (z > 1) {
                        this.joints.push(new BarB3(pt, this.points[index - 2 * w * h], bendSep, bendStiff));
                    }
                }
            }
        }
    }
    solve(){
        this.joints.forEach(joint => {
            joint.solve()
        })
    }
    integrate(gravity){
        this.points.forEach(point => {point.integrate(gravity)})
    }
}