//
//  direct_mis.cpp
//  nori
//
//  Created by Yuan Tian on 4/27/16.
//
//

#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

class Direct_Mis : public Integrator {
public:
    Direct_Mis (const PropertyList &props) {
        // no parameters this time
    }
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        

        Color3f ERadiance(0.f);   //Emitter Radiance
        Color3f LRadiance(0.f);   //Light Radiance
        Color3f RRadiance_em(0.f);   //Reflected Radiance of emitter sampling
        Color3f RRadiance_mat(0.f);   //Reflected Radiance of brdf sampling
        Color3f BRadiance(0.f);   //BRDF Radiance;
        
        const float EPSILON = 0.00001;
      
        float rnd = float(rand())/RAND_MAX;
        auto Light = scene->getRandomEmitter(rnd);  //choose a random emitter
        
     
        Intersection its;
        if (!scene->rayIntersect(ray, its))            // if the emitter is infinity and camera ray is in the subtanded angle,
        {                                                 //   Lradiance should be returned
            EmitterQueryRecord emitInf(ray.o);
            emitInf.wi=ray.d;
            
            ERadiance = Light->eval(emitInf);
            
            return ERadiance;
            
        }
       
        else if (its.mesh->isEmitter())    // if the intersection is on an emitter mesh
        {
            
            const Emitter* emitter = its.mesh->getEmitter();
            EmitterQueryRecord mesh_emit(emitter,its.p,ray.o,its.shFrame.n);
            
            
            ERadiance = emitter->ERadiance(mesh_emit);
            
        }
        
        
        
        //pdfs for weighting samples later
        float pdf_em_em= 0,pdf_em_mat = 0,
        pdf_mat_em = 0,pdf_mat_mat = 0;
        
        /**************************** emitter sampling *************************/
        
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
            
            const BSDF *bsdf = its.mesh->getBSDF();
            Vector3f Global_wo = -ray.d;     // output light direction in world ordinate
            Global_wo = Global_wo/Global_wo.norm();
            
            
            Vector3f Local_wi = its.toLocal(emit.wi);    //Transform to local ordinate
            Vector3f Local_wo = its.toLocal(Global_wo);
            
            
            BSDFQueryRecord diffuse(Local_wi,Local_wo,ESolidAngle);
            
            pdf_em_mat= bsdf->pdf(diffuse);       //pdfmat(wi,e)
            Color3f BRadiance = bsdf->eval(diffuse);    //fr()
           
            pdf_em_em = Light->pdf(emit);                        //pdfem(wi,e)
            if(pdf_em_em > 0)
                 RRadiance_em = LRadiance*BRadiance*cosi/pdf_em_em;
          
            //cout<<"pdf_em_em = "<<pdf_em_em<<endl;
        
        }
        
        LRadiance = 0.f;
        /**************************** bsdf sampling *************************/
        
        //sample BSDF
        BSDFQueryRecord Sam_bsdf(its.toLocal(-ray.d));
        
        Point2f sample2D2(float(rand())/RAND_MAX, float(rand())/RAND_MAX);
        
        
        BRadiance = its.mesh->getBSDF()->sample(Sam_bsdf,sample2D2);
        
        pdf_mat_mat = its.mesh->getBSDF()->pdf(Sam_bsdf);              //pdfmat(wi,m)
        
        //eval incident light
        Ray3f reflected_ray = Ray3f(its.p,its.toWorld(Sam_bsdf.wo));
        Intersection reflected_its;
  
        if (!scene->rayIntersect(reflected_ray, reflected_its)){
            
            EmitterQueryRecord emitUnk(reflected_ray.o);
            emitUnk.wi=reflected_ray.d;
            LRadiance = Light->eval(emitUnk);
            
            float cosi = its.shFrame.n.dot(emitUnk.wi);
            cosi = (cosi > 0)? cosi:0;
          
            pdf_mat_em = Light->pdf(emitUnk);          //pdfem(wi,m)
            
            //RRadiance_mat = LRadiance*BRadiance*cosi;
            RRadiance_mat = LRadiance*BRadiance;  //Piazza
        }
        else if (reflected_its.mesh->isEmitter()) {
            const Emitter *emitter2 = reflected_its.mesh->getEmitter();
            EmitterQueryRecord mesh_emit2(emitter2,reflected_its.p,reflected_ray.o,reflected_its.shFrame.n);
            
           // float cosi = mesh_emit2.n.dot(mesh_emit2.wi);
            LRadiance = emitter2->ERadiance(mesh_emit2);
            pdf_mat_em = Light->pdf(mesh_emit2);       //pdfem(wi,m)
            
            RRadiance_mat = LRadiance*BRadiance;
            
        }

        
       //*********************Combination******************************

        float w_em = 0, w_mat = 0;
        if(pdf_em_em> 0 && pdf_em_mat>= 0)
            w_em = pdf_em_em/(pdf_em_em+pdf_em_mat);
        
        if(pdf_mat_mat>0 && pdf_mat_em>= 0)
            w_mat = pdf_mat_mat/(pdf_mat_mat+pdf_mat_em);

        return ERadiance + w_em * RRadiance_em + w_mat * RRadiance_mat;
       
    }
    std::string toString() const {
        return "DirectMis Integrator";
    }
};

NORI_REGISTER_CLASS(Direct_Mis , "direct_mis");
NORI_NAMESPACE_END

