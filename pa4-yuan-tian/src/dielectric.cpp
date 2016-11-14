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

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
        
        m_albedo = propList.getColor("albedo", Color3f(1.f));
    }

    virtual Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }


    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
      
        bRec.measure = EDiscrete;
        float cosTheta = Frame::cosTheta(bRec.wi);
        bool In;    // mark the direction this light come from  Ture:eta1->eta2/False: eta2 ->eta1
        
        if(cosTheta > 0){
            In = true;
             bRec.eta = m_extIOR/m_intIOR;
           
        }
        else{
            In = false;
            bRec.eta = m_intIOR/m_extIOR;
        }
        
        float Fr = fresnel(cosTheta,m_extIOR,m_intIOR);
      
        
        if(sample.x()< Fr){
            //reflection
            bRec.wo = Vector3f(-bRec.wi.x(),-bRec.wi.y(),bRec.wi.z());
        }
        else
        {   //refractioon
            
            if(In){
            //Snell’s law
                Vector3f n = Vector3f(0.f,0.f,1.f);
                float WdotN =  bRec.wi.dot(n);
                Vector3f WTerm = bRec.wi-WdotN*n;
                float c = -bRec.eta;
                Vector3f SqrTerm = n*sqrt(1.f -  bRec.eta*bRec.eta*(1.f-WdotN*WdotN));
            
                bRec.wo = c*WTerm - SqrTerm;
            
            }
            else{
                Vector3f n_r = Vector3f(0.f,0.f,-1.f); //reverse normal vector
                float WdotN =  bRec.wi.dot(n_r);
                Vector3f WTerm = bRec.wi-WdotN*n_r;
                float c = -bRec.eta;
                Vector3f SqrTerm = n_r*sqrt(1.f -  bRec.eta*bRec.eta*(1.f-WdotN*WdotN));
                
                bRec.wo = c*WTerm - SqrTerm;
            }
            
        }
        return m_albedo;
        
     
    }
        
    

    virtual std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
    Color3f m_albedo;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
