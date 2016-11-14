/*
 This file is part of Nori, a simple educational ray tracer
 
 Copyright (c) 2015 by Wenzel Jakob
 
 Nori is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License Version 3
 as published by the Free Software Foundation.
 
 Nori is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;
    
    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }
    
    
    
    
    virtual void preprocess(const Scene *scene) {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();
        
        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
                                                  NoriObjectFactory::createInstance("independent", PropertyList()));
        
        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);
        
        /* Estimate a default photon radius */
        if (m_photonRadius == 0)
            m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;
        
        
        /* How to add a photon?
         * m_photonMap->push_back(Photon(
         *	Point3f(0, 0, 0),  // Position
         *	Vector3f(0, 0, 1), // Direction
         *	Color3f(1, 2, 3)   // Power
         * ));
         */
        
        // put your code to trace photons here
        
        for(int i = 0; i < m_photonCount; i++){
            
            Color3f power(0.f);
            Ray3f Sample_ray;
            
            float rnd = sampler->next1D();
            auto Light = scene->getRandomEmitter(rnd);  //choose a random emitter
            
            power = Light->samplePhoton(Sample_ray,sampler->next2D(),sampler->next2D());
            
            tracePhoton(scene,Sample_ray,power/m_photonCount,sampler);
            
            
        }
        
        
        
        /* Build the photon map */
        m_photonMap->build();
        
        cout<<"Preprocessing Finished!"<<endl;
    }
    
    
    
    
    void tracePhoton(const Scene *scene, const Ray3f &S_ray, const Color3f &P_power, Sampler* sampler){
        
        
        const float EPSILON = 0.00001;
        const float p = 0.2f; //termination probability
        
        
        
        Color3f ARadiance(0.f);   //All Radiance
        Color3f ERadiance(0.f);   //Emitter Radiance
        Color3f BRadiance(0.f);   //BSDF sampling Radiance
        Color3f IRadiance(0.f);   //Indirect Radiance;
        
        
        Intersection its;
        if (!scene->rayIntersect(S_ray, its) || its.t < EPSILON )
            return;     //No photon will be recorded.
        
        
       
            
            //sample BSDF
            BSDFQueryRecord Sam_bsdf(its.toLocal(-S_ray.d));
            BRadiance = its.mesh->getBSDF()->sample(Sam_bsdf,sampler->next2D());
            
            Photon InPhoton(its.p,S_ray.d,P_power);
            Ray3f Ray_out(its.p,its.toWorld(Sam_bsdf.wo));
            //accord this Photon
            if(its.mesh->getBSDF()->isDiffuse()){   //Each time a diffuse surface is intersected, a photon record is created in the photon map.
                     m_photonMap->push_back(InPhoton);
            }
            //check for terminating the recursion
            if(sampler->next1D() > p){
                tracePhoton(scene, Ray_out,P_power*BRadiance/(1.f-p), sampler);
                
            }
        
        
    }
    
  
    

    float getGaussianFilterWeight(const float &dist_sqr, const float radius_sqr) const{
        
        // Gaussian filter constants
        const float ALPHA = 0.918, BETA = 1.953, E = 2.718, ONE_MInus_E_TO_MInus_BETA = 0.858;
        
        const float power = -(BETA * 0.5F * (dist_sqr/radius_sqr));
        const float numerator = 1.0F - pow(E, power);
        const float denominator = ONE_MInus_E_TO_MInus_BETA;
        return ALPHA * (1.0F - (numerator/denominator));
    }
    
    
    
    Color3f ComputePhotonInScope(const Point3f &p, const Normal3f &n,
                             float searchRadius) const {
        
        Color3f PhotonInScope(0.0f);
        
        std::vector<Photon::IndexType> scope_photons;
        m_photonMap->search(p, searchRadius, scope_photons);
        const float A = M_PI*pow(searchRadius,2);
        
        
        
        for (size_t i=0; i<scope_photons.size(); i++) {
            
            const Photon &photon = (*m_photonMap)[scope_photons[i]];
            const Vector3f disVector = photon.getPosition()-p;
            const float distance = disVector.norm();
            
            if (photon.getDirection().dot(n) <= 0) { //incident from correct side
                PhotonInScope += photon.getPower()* getGaussianFilterWeight(pow(distance,2),pow(searchRadius,2));
            }
        }
        
    
        return PhotonInScope / A;
        
    }
    
    
    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const {
        
        /* How to find photons?
         * std::vector<uint32_t> results;
         * m_photonMap->search(Point3f(0, 0, 0), // lookup position
         *                     m_photonRadius,   // search radius
         *                     results);
         *
         * for (uint32_t i : results) {
         *    const Photon &photon = (*m_photonMap)[i];
         *    cout << "Found photon!" << endl;
         *    cout << " Position  : " << photon.getPosition().toString() << endl;
         *    cout << " Power     : " << photon.getPower().toString() << endl;
         *    cout << " Direction : " << photon.getDirection().toString() << endl;
         * }
         */
        // put your code for path tracing with photon gathering here
        
        const float EPSILON = 0.00001;
        const float p = 0.2;    //termination probability
        const float SearchRadius = m_photonRadius;
        
        
        Color3f ARadiance(0.f);   //All Radiance
        Color3f ERadiance(0.f);   //Emitter Radiance
        Color3f BRadiance(0.f);        //BSDF sampling Radiance
        Color3f PRadiance(0.f);   //Photon Estimate Radiance;
        Color3f IRadiance(0.f);   //Indirect Radiance;
        
        //if no intersection, return emittance immediately
        
        
        float rnd = sampler->next1D();
        auto Light = scene->getRandomEmitter(rnd);  //choose a random emitter
        int Light_num = scene->getLights().size();
        float pdf_light = 1.f/Light_num;

        Intersection its;
        if (!scene->rayIntersect(_ray, its)||its.t < EPSILON)            // if the emitter is infinity and camera ray is in the subtanded angle,
        {
            //   Lradiance should be returned
            if(Light->IsDistantLight())
            {
                EmitterQueryRecord emitInf(_ray.o);
                emitInf.wi=_ray.d;
                
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
            EmitterQueryRecord mesh_emit(emitter,_ray.o,its.p,its.shFrame.n);
            
            
            ERadiance = emitter->ERadiance(mesh_emit);
            
        }
        
        
        
        //sample BSDF
        BSDFQueryRecord Sam_bsdf(its.toLocal(-_ray.d));
        
        Point2f sample2D=sampler->next2D();
        
        BRadiance = its.mesh->getBSDF()->sample(Sam_bsdf,sample2D);
        
        //sample outgoint light
        Ray3f reflected_ray = Ray3f(its.p,its.toWorld(Sam_bsdf.wo));
        
        //if intersection is diffuse surface -> photon mapping
        if(its.mesh->getBSDF()->isDiffuse()){
            PRadiance = ComputePhotonInScope(its.p,its.shFrame.n,SearchRadius);
            IRadiance = BRadiance*PRadiance;
            
        }
        else if(sampler->next1D() > p){   //check for terminating the recursion
            
            IRadiance = Li(scene,sampler,reflected_ray)*BRadiance/(1.f-p);
            
        }
        
        
        return ERadiance + IRadiance;
        
    }
    
    virtual std::string toString() const {
        return tfm::format(
                           "PhotonMapper[\n"
                           "  photonCount = %i,\n"
                           "  photonRadius = %f\n"
                           "]",
                           m_photonCount,
                           m_photonRadius
                           );
    }
private:
    int m_photonCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END

