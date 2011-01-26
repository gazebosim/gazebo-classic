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
 * SVN: $Id:$
 */

#ifndef MASS_HH
#define MASS_HH

#include "Quatern.hh"
#include "Vector3.hh"

namespace gazebo
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
    public: void SetCoG(const Vector3 &c);

    /// \brief Get the center of gravity
    public: Vector3 GetCoG() const;

    /// \brief Get the prinicpal moments of inertia (Ixx, Iyy, Izz)
    public: Vector3 GetPrincipalMoments() const;

    /// \brief Get the products of inertia (Ixy, Ixy, Iyz)
    public: Vector3 GetProductsofInertia() const;

    /// \brief Rotate this mass
    public: void Rotate(const Quatern &rot);

    /// \brief Equal operator
    public: void operator=(const Mass &mass);

    public: Mass operator+(const Mass &_mass ) const;
    public: const Mass &operator+=(const Mass &_mass );

    public: friend std::ostream &operator<<(std::ostream &out, 
                                            const gazebo::Mass &mass)
            {
              out << "Mass[" << mass.mass << "] CoG[" << mass.cog << "]";
              return out;
            }

    private: double mass;
    private: Vector3 cog;
    private: Vector3 principals;
    private: Vector3 products;
  };
}
#endif
