#ifndef PLANE_HH
#define PLANE_HH

#include "Vector3.hh"
#include "Vector2.hh"

namespace gazebo
{
  class Plane
  {
    public: Plane();
    public: virtual ~Plane();

    public: void Set(Vector3 normal, Vector2<double> size, double offset);

    /// \brief Equal operator
    public: const Plane &operator=(const Plane & p);

    public: Vector3 normal;
    public: Vector2<double> size;
    public: double d;
  };
}

#endif
