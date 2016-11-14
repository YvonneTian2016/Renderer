//
//  path_mats.cpp
//  nori
//
//  Created by Yuan Tian on 5/8/16.
//
//

#include <stdio.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

class Path_Mats : public Integrator{
    
public:
    Path_Mats (const PropertyList &props) {
        // no params
    }
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        
        const float p = 0.2f; //termination probability
    
        Color3f ERadiance(0.f);   //Emitter Radiance
        Color3f BSDF(0.f);        //BSDF sampling Radiance
        Color3f IRadiance(0.f);   //Indirect Radiance;

        int Light_num = scene->getLights().size();
        
        float pdf_light = 1.f/Light_num;
        float rnd = sampler->next1D();
        auto Light = scene->getRandomEmitter(rnd);  //choose a random emitter
        
        
        Intersection its;
        if (!scene->rayIntersect(ray, its))            // if the emitter is infinity and camera ray is in the subtanded angle,
        {
            
            //   Lradiance should be returned
            if(Light->IsDistantLight())
            {
                EmitterQueryRecord emitInf(ray.o);
                emitInf.wi=ray.d;
                
                ERadiance = scene->m_distantEmitter->eval(emitInf);
                
                return ERadiance/pdf_light;
            }
            else
            {
                return Color3f(0.f);
            }
            
        }
        
        else if (its.mesh->isEmitter())    // if the intersection is on an emitter mesh
        {
            
            const Emitter* emitter = its.mesh->getEmitter();
            EmitterQueryRecord mesh_emit(emitter,ray.o,its.p,its.shFrame.n);
            
            
            ERadiance = emitter->ERadiance(mesh_emit);
            
        }
        
        
        //sample BSDF
        BSDFQueryRecord Sam_bsdf(its.toLocal(-ray.d));
        
        Point2f sample2D=sampler->next2D();
        
        Sam_bsdf.measure = ESolidAngle;
        
        BSDF = its.mesh->getBSDF()->sample(Sam_bsdf,sample2D);
        
    
        //sample outgoint light
        Ray3f reflected_ray = Ray3f(its.p,its.toWorld(Sam_bsdf.wo));
        
        
        if(sampler->next1D() > p){
            
            IRadiance = Li(scene,sampler,reflected_ray)*BSDF/(1.f-p);
            
        }
       
             return ERadiance + IRadiance;
                
    
    }
    std::string toString() const {
        return "Path_Mats_Integrator";
    }


};


NORI_REGISTER_CLASS(Path_Mats , "path_mats");
NORI_NAMESPACE_END