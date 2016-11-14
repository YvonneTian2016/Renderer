//
//  DirectMats.cpp
//  nori
//
//  Created by Yuan Tian on 4/25/16.
//
//

#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

class Direct_Mats : public Integrator {
public:
    Direct_Mats (const PropertyList &props) {
        // no parameters here
    }
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        
        Color3f ARadiance(0.f);   //All Radiance
        Color3f LRadiance(0.f);   //Emitter Radiance
        Color3f BRadiance(0.f);   //BRDF Radiance;
        
        
        float rnd = float(rand())/RAND_MAX;
        auto Light = scene->getRandomEmitter(rnd); //choose a random emitter
        
        Intersection its;
        
        
        if (!scene->rayIntersect(ray, its))            // if the emitter is infinity and camera ray is in the subtanded angle,
        {                                                 //   Lradiance should be returned
            EmitterQueryRecord emitInf(ray.o);
            emitInf.wi=ray.d;
            
            LRadiance = Light->eval(emitInf);
            
            return LRadiance;
            
        }
        else if (its.mesh->isEmitter())    // if the intersection is on an emitter mesh
        {
            
            const Emitter* emitter = its.mesh->getEmitter();
            EmitterQueryRecord mesh_emit(emitter,its.p,ray.o,its.shFrame.n);
            
            
            LRadiance = emitter->ERadiance(mesh_emit);
            ARadiance += LRadiance;
        }
        
        //sample BSDF
        BSDFQueryRecord Sam_bsdf(its.toLocal(-ray.d));
        
        Point2f sample2D(float(rand())/RAND_MAX, float(rand())/RAND_MAX);
        
        BRadiance = its.mesh->getBSDF()->sample(Sam_bsdf,sample2D);
        
        //eval incident light
        Ray3f reflected_ray = Ray3f(its.p,its.toWorld(Sam_bsdf.wo));
        Intersection reflected_its;

        if (!scene->rayIntersect(reflected_ray, reflected_its)){
            
            EmitterQueryRecord emitUnk(reflected_ray.o);
            emitUnk.wi=reflected_ray.d;
            LRadiance = Light->eval(emitUnk);
            
            float cosi = its.shFrame.n.dot(emitUnk.wi);
            cosi = (cosi > 0)? cosi:0;
            
           // ARadiance +=LRadiance*BRadiance*cosi;
            ARadiance +=LRadiance*BRadiance;     // see on piazza

        }
        else if (reflected_its.mesh->isEmitter()) {
            const Emitter *emitter2 = reflected_its.mesh->getEmitter();
            EmitterQueryRecord mesh_emit2(emitter2,reflected_its.p,reflected_ray.o,reflected_its.shFrame.n);
            
            LRadiance = emitter2->ERadiance(mesh_emit2);
            
            ARadiance +=LRadiance*BRadiance;
        }
        
        return ARadiance;
    }
    
    std::string toString() const {
        return "Direct_Mats Integrator";
    }
};

NORI_REGISTER_CLASS(Direct_Mats , "direct_mats");
NORI_NAMESPACE_END
