//
//  PointLight.cpp
//  nori
//
//  Created by Yuan Tian on 4/6/16.
//
//

#include <stdio.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter {
public:
    PointLight(const PropertyList &props) {
        light_position = props.getPoint3("position");
        light_power=props.getColor("power");
    }
    
    virtual std::string toString() const {
        return tfm::format(
                           "PointLight[\n"
                           "  position = %s,\n"
                           "  power = %s,\n"
                           "]",
                           light_position.toString(),light_power.toString());
    }
    
    virtual Color3f eval(const EmitterQueryRecord & lRec) const {

        if(!m_mesh)
            throw NoriException("There is no shape attached to this Point light!");
        
        throw NoriException("To implement...");
    }
    
    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const {
        lRec.p = light_position;
        lRec.wi = lRec.ref - lRec.p;
        lRec.dist = lRec.wi.norm();
        lRec.wi /= lRec.dist;
        
        return light_power/(lRec.dist*lRec.dist*4*M_PI);
        
        if(!m_mesh)
            throw NoriException("There is no shape attached to this Point light!");
        
        //throw NoriException("To implement...");
    }
    
    virtual float pdf(const EmitterQueryRecord &lRec) const {
        if(!m_mesh)
            throw NoriException("There is no shape attached to this Point light!");
        
        throw NoriException("To implement...");
    }
    
    
    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const {
        throw NoriException("To implement...");
    }
    
private:
    Point3f light_position;
    Color3f light_power;
    
};

NORI_REGISTER_CLASS(PointLight, "point")
NORI_NAMESPACE_END