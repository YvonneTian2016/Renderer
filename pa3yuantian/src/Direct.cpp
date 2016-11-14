//
//  Direct.cpp
//  nori
//
//  Created by Yuan Tian on 4/6/16.
//
//

#include <stdio.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>


NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator {
public:
    DirectIntegrator(const PropertyList &props) {
        /* No parameters this time */
        //len=props.getFloat("length",5.0f);
    }
    
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        
        Color3f ARadiance = 0.0f;
        
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        
      
        
        
       // PointLight light()
        for(auto Light: scene->getLights())
        {
             EmitterQueryRecord emit(its.p);
             Point2f sample2D(float(rand())/RAND_MAX, float(rand())/RAND_MAX);
              
             Color3f PRadiance = Light->sample(emit,sample2D);
            
          
            
           // Vector3f Global_wi =  emit.p-its.p;     //incident light direction in world ordinate
           
           // Global_wi = Global_wi/Global_wi.norm();
           
            
            Ray3f l(its.p, emit.wi, Epsilon, emit.dist);
            
            
            if(!scene->rayIntersect(l))
            {
                Normal3f sh_normal= its.shFrame.n;
                
                float cos = sh_normal.dot(emit.wi);
                cos = (cos > 0)? cos:0;
                
                const BSDF* bsdf = its.mesh->getBSDF();
                
                Vector3f Global_wo = -ray.d;     // output light direction in world ordinate
                Global_wo = Global_wo/Global_wo.norm();
                //Transform to local ordinate
                Vector3f Local_wi = its.toLocal(emit.wi);
                Vector3f Local_wo = its.toLocal(Global_wo);

               
                BSDFQueryRecord diffuse(Local_wi,Local_wo,ESolidAngle);
                
                Color3f BRadiance = bsdf->eval(diffuse);
                
                ARadiance += PRadiance*cos*BRadiance;
            
            }
        
        }
        
        return ARadiance;
        
        
    }
    
    std::string toString() const {
        return "DirectIntegrator[]";
    }

};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END