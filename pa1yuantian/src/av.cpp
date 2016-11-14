//
//  av.cpp
//  nori
//
//  Created by Yuan Tian on 4/5/16.
//
//

#include <stdio.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>
#include <nori/ray.h>

NORI_NAMESPACE_BEGIN

class AVIntegrator : public Integrator {
public:
    AVIntegrator(const PropertyList &props) {
        /* No parameters this time */
        len=props.getFloat("length",5.0f);
    }
    
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(1.0f);
        
        Normal3f sh_normal= its.shFrame.n;
        
        Vector3f  v = Warp::sampleUniformHemisphere(sampler, sh_normal);
        Ray3f ray_he(its.p, v, Epsilon,len);
        
        Intersection its2;
        if(!scene->rayIntersect(ray_he,its2))
            return Color3f(1.0f);
    
        return Color3f(0.0f);
        
    }
    
    std::string toString() const {
        return "AVIntegrator[]";
    }
private:
    float len;
};

NORI_REGISTER_CLASS(AVIntegrator, "av");
NORI_NAMESPACE_END