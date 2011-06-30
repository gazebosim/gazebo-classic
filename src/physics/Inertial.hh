/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef INERTIAL_HH
#define INERTIAL_HH

#include "sdf/sdf.h"
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
