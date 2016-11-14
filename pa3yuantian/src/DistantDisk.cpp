//
//  Mydisk.cpp
//  nori
//
//  Created by Yuan Tian on 4/23/16.
//
//

#include <stdio.h>

#include <stdio.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/mesh.h>
#include <limits>

NORI_NAMESPACE_BEGIN

class DistantDisk : public Emitter {
public:
    DistantDisk(const PropertyList &props) {
        
        light_radiance=props.getColor("radiance");
        subtended_angle = props.getFloat("thetaA")*M_PI/180;
        MaxCosTheta = cos(subtended_angle);
        
        
        /* Read a toWorld transform from the XML scene
         specification. If not present, use the identity */
        m_emitterToWorld = props.getTransform("toWorld", Transform());
        
        /* Invert the toWorld transformation */
        m_worldToEmitter = m_emitterToWorld.inverse();
    }
    DistantDisk(){}
    
    virtual std::string toString() const {
        return tfm::format(
                           "DistantDiskLight[\n"
                           "  radiance = %s,\n"
                           "]",
                           light_radiance.toString());
    }
    
    virtual Color3f eval(const EmitterQueryRecord & lRec) const {
        Vector3f p_local = m_worldToEmitter*lRec.wi;
        p_local.normalize();
        float z = p_local.z();
        
        if(z > MaxCosTheta)
        {
            return light_radiance;
        }
        else
            return Color3f (0.0f);
       
        
    }
    
    virtual Color3f ERadiance(const EmitterQueryRecord &lRec) const{
       
       
        
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const {
        
        
        
        lRec.wi = Warp::squareToUniformSphereCap(sample,MaxCosTheta);
        
        lRec.dist=std::numeric_limits<float>::max();
        
        lRec.wi = m_emitterToWorld *lRec.wi;        // Sample point in the subtended angle with the world coordinate
     

        lRec.pdf = pdf(lRec);
        
        return light_radiance;
        
    }
    
    virtual float pdf(const EmitterQueryRecord &lRec) const {
        
        
        float pdf = Warp::squareToUniformSphereCapPdf(m_worldToEmitter*lRec.wi,MaxCosTheta);
        return pdf;
        
        
    }
    
    
    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const {
        throw NoriException("To implement...");
    }
    
    
    
private:
    
    Color3f light_radiance;
    float subtended_angle;
    Transform m_emitterToWorld;
    Transform m_worldToEmitter;
    float MaxCosTheta;
    
    
};

NORI_REGISTER_CLASS(DistantDisk, "distantdisk")
NORI_NAMESPACE_END

