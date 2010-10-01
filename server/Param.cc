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
/* Desc: Parameter class
 * Author: Nate Koenig
 * Date: 14 Aug 2008
 * SVN: $Id$
 */

#include "GazeboError.hh"
#include "Param.hh"
#include "Quatern.hh"
#include "Pose3d.hh"
#include "Vector3.hh"
#include "Vector4.hh"

using namespace gazebo;

std::vector<Param*> *Param::params = NULL;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Param::Param(Param *newParam) 
{
  if (params == NULL)
    gzthrow("Param vector is NULL\n");
  params->push_back(newParam);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Param::~Param() 
{
}

////////////////////////////////////////////////////////////////////////////////
//  Begin a block of "new ParamT<>"
void Param::Begin(std::vector<Param*> *_params)
{
  if (params != NULL)
    gzthrow("Calling Begin before an End\n");
  params = _params;
}

////////////////////////////////////////////////////////////////////////////////
//  End a block of "new ParamT<>"
void Param::End()
{
  if (params == NULL)
    gzthrow("Calling End before a Begin\n");

  params = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// The name of the key
std::string Param::GetKey() const
{
  return this->key;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the param's data type
std::string Param::GetTypename() const
{
  return this->typeName;
}

bool Param::IsBool() const
{
  return this->GetTypename() == typeid(bool).name();
}

bool Param::IsInt() const
{
  return this->GetTypename() == typeid(int).name();
}

bool Param::IsUInt() const
{
  return this->GetTypename() == typeid(unsigned int).name();
}

bool Param::IsFloat() const
{
  return this->GetTypename() == typeid(float).name();
}

bool Param::IsDouble() const
{
  return this->GetTypename() == typeid(double).name();
}

bool Param::IsChar() const
{
  return this->GetTypename() == typeid(char).name();
}

bool Param::IsStr() const
{
  return this->GetTypename() == typeid(std::string).name();
}

bool Param::IsVector3() const
{
  return this->GetTypename() == typeid(Vector3).name();
}
bool Param::IsVector4() const
{
  return this->GetTypename() == typeid(Vector4).name();
}
bool Param::IsQuatern() const
{
  return this->GetTypename() == typeid(Quatern).name();
}
bool Param::IsPose3d() const
{
  return this->GetTypename() == typeid(Pose3d).name();
}
