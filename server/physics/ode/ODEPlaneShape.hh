#ifndef ODEPLANESHAPE_HH
#define ODEPLANESHAPE_HH

#include "Body.hh"
#include "Mass.hh"
#include "ODEPhysics.hh"
#include "PlaneShape.hh"

namespace gazebo
{
  class ODEPlaneShape : public PlaneShape
  {
    public: ODEPlaneShape(Geom *parent) : PlaneShape(parent) {}
    public: virtual ~ODEPlaneShape() {}
  
    /// \brief Create the plane
    public: void CreatePlane()
            {
              PlaneShape::CreatePlane();
              ODEGeom *odeParent = (ODEGeom*)(this->parent);
  
              double altitude = 0;

              odeParent->SetGeom(dCreatePlane(odeParent->GetSpaceId(), 
                                 (**normalP).x, (**normalP).y, (**normalP).z, 
                                 altitude), false);
            }
  
    /// Set the altitude of the plane
    public: void SetAltitude(const Vector3 &pos)
            {
              PlaneShape::SetAltitude(pos);
              ODEGeom *odeParent = (ODEGeom*)(this->parent);

              dVector4 vec4;
  
              dGeomPlaneGetParams(odeParent->GetGeomId(), vec4);
  
              // Compute "altitude": scalar product of position and normal
              vec4[3] = vec4[0] * pos.x + vec4[1] * pos.y + vec4[2] * pos.z;
              dGeomPlaneSetParams(odeParent->GetGeomId(), vec4[0], vec4[1], 
                                  vec4[2], vec4[3]);
            }
  };
}
#endif
