//
//  direct_ems.cpp
//  nori
//  Function: Integrate incident radiance by perfoming emitter sampling.
//
//  Created by Yuan Tian on 4/18/16.
//
//

#include <stdio.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>


NORI_NAMESPACE_BEGIN

class DirectEmitterIntegrator : public Integrator {
public:
    DirectEmitterIntegrator(const PropertyList &props) {
      
    }
    
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        
        int Light_num = scene->getLights().size();
        
        
        float pdf_light = 1.f/Light_num;
      
        Color3f ARadiance(0.f);   //All Radiance
        Color3f LRadiance(0.f);   //Emitter Radiance
        Color3f RRadiance(0.f);   //Reflected Radiance;
        
        const float EPSILON = 0.00001;
        
  
        float rnd = float(rand())/RAND_MAX;
        auto Light = scene->getRandomEmitter(rnd);  ////choose a random emitter
        
        
        Intersection its;
        
        
        if (!scene->rayIntersect(ray, its))            // if the emitter is infinity and camera ray is in the subtanded angle,
        {                                                 //   Lradiance should be returned
            EmitterQueryRecord emitInf(ray.o);
            emitInf.wi=ray.d;
    
            LRadiance = Light->eval(emitInf)/pdf_light;

            return LRadiance;
            
        }
        else if (its.mesh->isEmitter())    // if the intersection is on an emitter mesh
        {
            
                    const Emitter* emitter = its.mesh->getEmitter();
                    EmitterQueryRecord mesh_emit(emitter,its.p,ray.o,its.shFrame.n);

            
                    LRadiance = emitter->ERadiance(mesh_emit);
                    ARadiance += LRadiance;
        }
        
        
        EmitterQueryRecord emit(its.p);
                    
        Point2f sample2D(float(rand())/RAND_MAX, float(rand())/RAND_MAX);

        
        LRadiance = Light->sample(emit,sample2D);
     
        Ray3f l(its.p, emit.wi);
        
        Intersection its_o;
    
        const bool is_shadowed = scene->rayIntersect(l,its_o) && its_o.t + EPSILON < emit.dist;
        
                    if(!is_shadowed)
                    {
                        
                           Normal3f sh_normal= its.shFrame.n;
                
                           float cosi = sh_normal.dot(emit.wi);
                           cosi = (cosi > 0)? cosi:0;
                        
                           const BSDF* bsdf = its.mesh->getBSDF();
                
                           Vector3f Global_wo = -ray.d;     // output light direction in world ordinate
                           Global_wo = Global_wo/Global_wo.norm();
                        
                        
                           Vector3f Local_wi = its.toLocal(emit.wi);    //Transform to local ordinate
                           Vector3f Local_wo = its.toLocal(Global_wo);
                
                
                           BSDFQueryRecord diffuse(Local_wi,Local_wo,ESolidAngle);
                        
                           Color3f BRadiance = bsdf->eval(diffuse);    //fr()
                        
                           if(Light->pdf(emit) > 0)
                               RRadiance = LRadiance*BRadiance*cosi/Light->pdf(emit)/pdf_light;
                        
                        ARadiance += RRadiance;
                    }
        
            
        
    return ARadiance;
    
        
    }
    
    std::string toString() const {
        return "Direct_ems_Integrator[]";
    }
  
    
};

NORI_REGISTER_CLASS(DirectEmitterIntegrator, "direct_ems");
NORI_NAMESPACE_END