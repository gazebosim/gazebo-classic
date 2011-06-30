#ifndef INERTIAL_HH
#define INERTIAL_HH

#include "sdf/interface/sdf.h"
#include "physics/Mass.hh"

namespace gazebo
{
  namespace physics
  {
    class Inertial
    {
      public: void Load( sdf::ElementPtr _sdf);

      public: double GetLinearDamping();
      public: double GetAngularDamping();

      public: const Mass &GetMass() const;

      /// Mass properties of the object
      private: Mass mass;
      private: sdf::ElementPtr sdf;
    };
  }
}

#endif
