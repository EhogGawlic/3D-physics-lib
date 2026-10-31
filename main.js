"use strict";
import * as vec from "./vec.js" // "JIC"
import {Obj2, Obj3} from "./obj.js"
import {Rigid2, Rigid3} from "./rigid.js"

//
// A note: Rigidbodies are made by Claude. I do not understand rigidbodies.
// Claude does, though. But I do understand spherical object collisions,
// Verlet physics, how to use Stack Overflow and Wikipedia, etc.
//

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
        const rbs = this.rigidbodies
        if (rbs.length === 0) return
        for (const rb of rbs) rb.update()
        for (const rb of rbs){
            for (const ob of this.objs) rb.collideBall(ob)
        }
        for (let i = 0; i < rbs.length; i++){
            for (let j = i + 1; j < rbs.length; j++) rbs[i].collideBody(rbs[j])
        }
    }
    step(){
        this.objs.forEach(ob=>{
            ob.integrate(this.gravity)
        })
        this.rigidbodies.forEach(rb=>{
            rb.integrate(this.gravity)
        })
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
    addObj(options){
        this.objs.push(this.dimen == 2 ? new Obj2(options) : new Obj3(options))
    }
}