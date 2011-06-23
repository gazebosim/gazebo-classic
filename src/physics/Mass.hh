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
/* Desc: Mass class
 * Author: Nate Koenig
 * Date: 18 May 2009
 */

#ifndef MASS_HH
#define MASS_HH

#include "math/Quaternion.hh"
#include "math/Vector3.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \brief Mass class
    class Mass
    {
      /// \brief Default Constructor
      public: Mass();
  
      /// \brief Constructor
      public: Mass(double mass);
  
      /// \brief Copy constructor
      public: Mass(const Mass &mass);
  
      /// \brief Destructor
      public: virtual ~Mass();
  
      /// \brief Reset all the mass properties
      public: void Reset();
  
      /// \brief Set the mass
      public: void SetMass(double m);
  
      /// \brief Get the mass value
      public: double GetAsDouble() const;
  
      /// \brief Set the mass matrix
      public: void SetInertiaMatrix( double ixx, double iyy, double izz,
                                     double ixy, double ixz, double iyz);
  
      /// \brief Set the center of gravity
      public: void SetCoG(double cx, double cy, double cz);
  
      /// \brief Set the center of gravity
      public: void SetCoG(const math::Vector3 &c);
  
      /// \brief Get the center of gravity
      public: math::Vector3 GetCoG() const;
  
      /// \brief Get the prinicpal moments of inertia (Ixx, Iyy, Izz)
      public: math::Vector3 GetPrincipalMoments() const;
  
      /// \brief Get the products of inertia (Ixy, Ixy, Iyz)
      public: math::Vector3 GetProductsofInertia() const;
  
      /// \brief Rotate this mass
      public: void Rotate(const math::Quaternion &rot);
  
      /// \brief Equal operator
      public: void operator=(const Mass &mass);
  
      public: Mass operator+(const Mass &_mass ) const;
      public: const Mass &operator+=(const Mass &_mass );
  
      public: friend std::ostream &operator<<(std::ostream &out, 
                                              const gazebo::physics::Mass &mass)
              {
                out << "Mass[" << mass.mass << "] CoG[" << mass.cog << "]";
                return out;
              }
  
      private: double mass;
      private: math::Vector3 cog;
      private: math::Vector3 principals;
      private: math::Vector3 products;
    };
  }
}
#endif
