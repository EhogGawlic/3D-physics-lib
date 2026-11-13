"use strict";
import * as vec from "./vec.js"
import {Obj2, Obj3} from "./obj.js"
import {Rigid2, Rigid3} from "./rigid.js"
import { Fan2, Fan3 } from "./fan.js";
import { Tcan2, Tcan3} from "./tcan.js"
import { BarB2, BarB3, BarR2, BarR3} from "./joint.js"
export {Vec2, Vec3, addVec2,addVec3,subVec2,subVec3,multVec2,multVec3,divVec2,divVec3,multVec22,multVec32,divVec22,divVec32,dist2, dist3, URD2, URD3, norm2, norm3, dot2, dot3, reflect2, reflect3} from "./vec.js"

export {Obj2, Obj3} from "./obj.js"
export {Rigid2, Rigid3} from "./rigid.js"
export { Fan2, Fan3 } from "./fan.js";
export { Tcan2, Tcan3} from "./tcan.js"
export { BarB2, BarB3, BarR2, BarR3} from "./joint.js"
export class Engine{
    constructor(options){
        this.objs = []
        this.lines = []
        this.valves = []
        this.fans = []
        this.tcans = []
        this.speedos = []
        this.rigidbodies = []
        this.magnets = []
        this.bars = []
        this.mode = Object.hasOwn(options, "mode") ? options.mode : "plain"
        this.dimen = Object.hasOwn(options, "dimensions") ? options.dimensions : 2
        this.walls = Object.hasOwn(options, "walls") ? options.walls : [0,0,400,600]
        this.wtype = Object.hasOwn(options, "walltype") ? options.walltype : "square"
        this.gravity = Object.hasOwn(options, "gravity") ? options.gravity : 0.98
        this.substeps = Object.hasOwn(options, "substeps") ? options.substeps : 4
    }
    collideAllBalls(){
        switch (this.mode){
            case "wasm":
                console.log("wasm")
                //TODO: do this [wasm block in collide all]
                break
            case "wgpu":
                console.log("wgpu")
                //TODO: do this too [wgpu block in collide all]
                break
            default:
                if (this.dimen == 2){
                    for (let i =0; i < this.substeps; i++){
                        this.solveBars()
                        Obj2.collideAll(this.objs)
                        this.collideRigidbodies()
                        this.objs.forEach(ob=>{
                            ob.collideWalls(this.wtype,...this.walls)
                        })
                        this.rigidbodies.forEach(rb=>{
                            rb.collideWalls(this.wtype,...this.walls)
                            rb.solveConstraints()
                        })
                        
                    }
                }

                if (this.dimen == 3){
                    for (let i =0; i < this.substeps; i++){
                        this.solveBars()
                        Obj3.collideAll(this.objs)
                        this.collideRigidbodies()
                        this.objs.forEach(ob=>{
                            ob.collideWalls(this.wtype,...this.walls)
                        })
                        this.rigidbodies.forEach(rb=>{
                            rb.collideWalls(this.wtype,...this.walls)
                            rb.solveConstraints()
                        })
                    }
                }
        }
    }
    collideRigidbodies(){
        if (rbs.length === 0) return
        for (const rb of rbs) rb.update()
        for (const rb of rbs){
            for (const ob of this.objs) rb.collideBall(ob)
        }

        if (this.dimen === 3) {
            Rigid3.solveAll(this)
        } else {
            for (let i = 0; i < rbs.length; i++) {
                for (let j = i + 1; j < rbs.length; j++) rbs[i].collideBody(rbs[j])
            }
        }
    }
    step(){

        this.objs.forEach(ob=>{
            ob.integrate(this.gravity)
        })
        this.rigidbodies.forEach(rb=>{
            rb.integrate(this.gravity)
        })
        if (this.dimen == 2){
            Fan2.applyAll(this.fans, this.objs, this.rigidbodies)
        }
        if (this.dimen == 3){
            Fan3.applyAll(this.fans, this.objs, this.rigidbodies)
        }
    }
    addFan(pos, dir, speed, taper, options={}){
        if (this.dimen == 2){
            const nf = new Fan2(
                pos, dir, options.type ?? 0, speed, taper, options.initialRadius ?? 20
            )
            this.fans.push(nf)
        }
        if (this.dimen == 3){
            const nf = new Fan3(
                pos, dir, options.type ?? 0, speed, taper, options.initialRadius ?? 20
            )
            this.fans.push(nf)
        }
    }
    addTcan(pos, size, options={}){
        const tcan = this.dimen === 2
            ? new Tcan2(pos, size, options.pRbIn)
            : new Tcan3(pos, size, options.pRbIn)
        this.tcans.push(tcan)
        return tcan
    }
    addBox(options){
        const body = this.dimen == 2 ? Rigid2.box(options) : Rigid3.box(options)
        this.rigidbodies.push(body)
        return body
    }
    addRigid(options){
        const body = this.dimen == 2 ? new Rigid2(options) : new Rigid3(options)
        this.rigidbodies.push(body)
        return body
    }
    addCompound(options){
        return this.addRigid(options)
    }
    weld(a, b){
        if (a === b) return a
        a.weld(b)
        const i = this.rigidbodies.indexOf(b)
        if (i >= 0) this.rigidbodies.splice(i, 1)
        return a
    }
    addObj(options){
        this.objs.push(this.dimen == 2 ? new Obj2(options) : new Obj3(options))
    }
    resizeBounds(newwalls){
        this.walls = newwalls
    }
    clear(){
        this.objs = []
        this.rigidbodies = []
        this.fans=[]
        this.tcans=[]
        
    }
    /**
     * fans don't fit! 
     * LATER ME: what
     */
    solveBars(){
        this.bars.forEach(bar=>{
            bar.solve()
        })
    }
    doTcans(){
        if (this.dimen == 2){
            return Tcan2.testAllAndDel(this.tcans, this.objs, this.rigidbodies)
        }
        if (this.dimen == 3){
            return Tcan3.testAllAndDel(this.tcans, this.objs, this.rigidbodies)
        }
    }
}