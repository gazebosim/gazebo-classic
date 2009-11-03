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

#include "Mass.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Default Constructor
Mass::Mass()
{
  this->mass = 0;
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Mass::Mass(double m)
{
  this->mass = m;
  this->cog.Set(0,0,0);
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Copy constructor
Mass::Mass(const Mass &mass)
{
  (*this) = mass;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Mass::~Mass()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Reset all the mass properties
void Mass::Reset()
{
  this->mass = 0;
  this->cog.Set(0,0,0);
  this->principals.Set(0,0,0);
  this->products.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Mass::SetMass(double m)
{
  this->mass = m;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mass value
double Mass::GetAsDouble() const
{
  return this->mass;
}

////////////////////////////////////////////////////////////////////////////////
// Set the center of gravity
void Mass::SetCoG(double cx, double cy, double cz)
{
  this->cog.Set(cx, cy, cz);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the center of gravity
void Mass::SetCoG(const Vector3 &c)
{
  this->cog = c;
}

////////////////////////////////////////////////////////////////////////////////
// Get the center of gravity
Vector3 Mass::GetCoG() const
{
  return this->cog;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass matrix
void Mass::SetInertiaMatrix(double ixx, double iyy, double izz,
                            double ixy, double ixz, double iyz)
{
  this->principals.Set( ixx, iyy, izz );
  this->products.Set( ixy, ixz, iyz );
}


////////////////////////////////////////////////////////////////////////////////
/// Get the prinicpal moments of inertia (Ixx, Iyy, Izz)
Vector3 Mass::GetPrincipalMoments() const
{
  return this->principals;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the products of inertia (Ixy, Ixy, Iyz)
Vector3 Mass::GetProductsofInertia() const
{
  return this->products;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
void Mass::operator=(const Mass &mass)
{
  this->mass = mass.mass;
  this->cog = mass.cog;
  this->principals = mass.principals;
  this->products = mass.products;
}

Mass Mass::operator+(const Mass &_mass ) const
{
  Mass result;
  result.mass = this->mass + _mass.mass;

  result.cog = (this->cog*this->mass + _mass.cog * _mass.mass) / result.mass;

  result.principals = this->principals + _mass.principals;
  result.products = this->products + _mass.products;
  return result;
}

const Mass &Mass::operator+=(const Mass &_mass )
{
  *this = *this + _mass;
  return *this;
}

