/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
