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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
 
    Vector2f unif_sample;
    
    float theta = 2*M_PI*(sample.x());
    float r = sqrt(sample.y());
   
    unif_sample.x() = r*cos(theta);
    unif_sample.y() = r*sin(theta);
    
    
    return unif_sample;
    
    throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    
    
    
      if ((powf(p.x(),2.0f)+powf(p.y(),2.0f)) <1.0f)
         return 1.0f/M_PI;
      else
         return 0.0f;
    
    
    throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformCylinder(const Point2f &sample){
    //unit Cylinder radius = 1
    
    Vector3f unif_sample;
    
    unif_sample.z() = 2.f*sample.x()-1.f;
    
    float phi = 2.0f*M_PI*sample.y();
    unif_sample.x() = cosf(phi);
    unif_sample.y() = sinf(phi);
    
    return unif_sample;
    
    
    throw NoriException("Warp::squareToUniformCylinder() is not yet implemented!");

}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
  
    //Using Cylinder Sampling
    
    /*
     Vector3f Cylinder_Sample = squareToUniformCylinder(sample);
     Vector3f unif_sample;
    
     unif_sample.z() = cosThetaMax+sample.x()*(1.f-cosThetaMax);
    
     float r = sqrtf(fmax(0.f, 1.f-powf(unif_sample.z(),2.f)));
     
     unif_sample.x() = r * Cylinder_Sample.x();
     unif_sample.y() = r * Cylinder_Sample.y();

     
     return unif_sample;
     */
 
    
    // Not using Cylinder Sampling
    Vector3f unif_sample;
    unif_sample.z() = cosThetaMax+sample.y()*(1.f-cosThetaMax);
    
    float r = sqrtf(fmax(0.f,1.0f-powf(unif_sample.z(),2.f)));
    
    float phi = 2.0f*M_PI*sample.x();
    unif_sample.x() = r * cosf(phi);
    unif_sample.y() = r * sinf(phi);
    
    return unif_sample;
    
    
    throw NoriException("Warp::squareToUniformSphereCap() is not yet implemented!");
}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {

    if (v.z() >= cosThetaMax)
        return INV_TWOPI/(1.f-cosThetaMax);
    else
        return 0.f;
    
    
    throw NoriException("Warp::squareToUniformSphereCapPdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
   
    //Using Cylinder Sampling
   
    /*
    Vector3f Cylinder_Sample = squareToUniformCylinder(sample);
    Vector3f unif_sample;
    
    float r = sqrtf(fmax(0.f, 1.f-powf(Cylinder_Sample.z(),2.f)));
    
    unif_sample.x() = r * Cylinder_Sample.x();
    unif_sample.y() = r * Cylinder_Sample.y();
    unif_sample.z() = Cylinder_Sample.z();
    
    return unif_sample;

    */
    
     // Not using Cylinder Sampling
    Vector3f unif_sample;
    unif_sample.z() = 2.0f*sample.y()-1.0f;
  
    float r = sqrtf(fmax(0.f,1.0f-powf(unif_sample.z(),2.f)));
    
    float phi = 2.0f*M_PI*sample.x();
    unif_sample.x() = r * cosf(phi);
    unif_sample.y() = r * sinf(phi);
    
    return unif_sample;
    
    
    throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
 
    return 1.f/(4.f*M_PI);
   
    
    throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
}


Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    
    //Using Cylinder Sampling
    /*
    Vector3f Cylinder_Sample = squareToUniformCylinder(sample);
    Vector3f unif_sample;
    
    unif_sample.z() = sample.x();
    
    float r = sqrtf(fmax(0.f, 1.f-powf(unif_sample.z(),2.f)));
    
    unif_sample.x() = r * Cylinder_Sample.x();
    unif_sample.y() = r * Cylinder_Sample.y();
    
    
    return unif_sample;
    
    */
    
    // Not Using Cylinder Sampling
   
    Vector3f unif_sample;
    unif_sample.z() = sample.y();
    
    
    float r = sqrtf(fmax(0.f,1.0f-powf(unif_sample.z(),2.f)));
    
    float phi = 2.0f*M_PI*sample.x();
    unif_sample.x() = r * sinf(phi);
    unif_sample.y() = r * cosf(phi);
    
    return unif_sample;

    
    throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    
    if (v.z() >= 0.f)
        return INV_TWOPI;
    else
        return 0.f;
    
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
   
    Vector3f unif_sample;
    Point2f unif_disk;
    unif_disk = squareToUniformDisk(sample);
    unif_sample.x() = unif_disk.x();
    unif_sample.y() = unif_disk.y();
    unif_sample.z() = sqrtf(fmax(0.f, 1.f - unif_sample.x()*unif_sample.x() - unif_sample.y()*unif_sample.y()));
    return unif_sample;
    
    throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    
    if(v.z()>=0.f)
    {
        Vector3f normal(0,0,1);
        float CosTheta = v.dot(normal)/v.norm();
    
        return CosTheta*INV_PI;
    }
    else
        return 0.f;
    throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    
    Vector3f Beckmann;

   // cout<<"sample_alpha="<<alpha<<endl;
    float theta = atanf(sqrtf(-(alpha*alpha)*log(1-sample.y())));
 
    float phi = 2*M_PI*sample.x();
    
    Beckmann.x() = sinf(theta)*cosf(phi);
    Beckmann.y() = sinf(theta)*sinf(phi);
    Beckmann.z() = cosf(theta);
    
    if(Beckmann.z()<0.f)
        cout<<"Beckmann.z()"<<Beckmann.z()<<endl;
    
    return Beckmann;
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {

    
   // cout<<"pdf_alpha="<<alpha<<endl;
    if(m.z() >0.f)
    {
        Vector3f normal(0,0,1);
        
        float CosTheta = m.dot(normal)/m.norm();
        float SinTheta = sqrtf(1.f-powf(CosTheta,2.f));
        float TanTheta = SinTheta/CosTheta;
        
        return INV_PI*(1.f/(alpha*alpha))*(1.f/powf(CosTheta,3.f))*exp(-1.f*powf(TanTheta,2.f)/(alpha*alpha));
    }
    
    else
        return 0.f;
    
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
