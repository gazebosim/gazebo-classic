#ifndef ODEMULTIRAYSHAPE_HH
#define ODEMULTIRAYSHAPE_HH

#include "ODEPhysics.hh"
#include "MultiRayShape.hh"

namespace gazebo
{
  /// \brief ODE specific version of MultiRayShape
  class ODEMultiRayShape : public MultiRayShape
  {
    /// \brief Constructor
    public: ODEMultiRayShape(Geom *parent);
  
    /// \brief Destructor
    public: virtual ~ODEMultiRayShape();
 
    /// \brief Update the rays 
    public: virtual void UpdateRays();

    /// \brief Ray-intersection callback
    private: static void UpdateCallback( void *data, dGeomID o1, dGeomID o2 );

    /// \brief Add a ray to the geom
    protected: void AddRay(const Vector3 &start, const Vector3 &end );

    /// Ray space for collision detector
    private: dSpaceID superSpaceId;
    private: dSpaceID raySpaceId; 
  };
}
#endif
