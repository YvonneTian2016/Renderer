/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/mesh.h>
#include <nori/object.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
    
    
    
public:
    void setMesh(Mesh * mesh) {
        
        m_mesh = mesh;
    
    }
    
    void setParent(Mesh * parent) {
        m_parent = parent;
    }
    
    
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const {
        return Color3f(0.f);
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const {
       
   
           m_mesh->samplePosition(sample,lRec.p,lRec.n);  //need edition
        
            lRec.wi = lRec.p - lRec.ref;
            lRec.dist = lRec.wi.norm();
            lRec.wi.normalize();
        
            if(-1*lRec.wi.dot(lRec.n)>0)
        
                return m_radiance;

            else
            
                return Color3f(0.f);
        
        
    }
    
    
    virtual Color3f ERadiance(const EmitterQueryRecord &lRec) const{
    
        if(lRec.wi.dot(lRec.n) > 0.f)
            return m_radiance;
        else
            return Color3f(0.f);
    
    }
    

    virtual float pdf(const EmitterQueryRecord &lRec) const {
        Vector3f wo = lRec.ref-lRec.p;

        if(wo.dot(lRec.n) > 0)
        {
            float  distance = wo.squaredNorm();
            wo.normalize();
            
            float unit_pdf = m_mesh->PDF_get();
          
          //  cout<<"unit_pdf"<<unit_pdf<<endl;
            
            return unit_pdf * distance/wo.dot(lRec.n);
            //return  unit_pdf;
        }
        
       return 0;
    
    
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const {
        throw NoriException("To implement...");
    }



protected:
    Color3f m_radiance;
    Mesh * m_parent;
    
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END