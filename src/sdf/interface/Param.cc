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
/* Desc: Parameter class
 * Author: Nate Koenig
 * Date: 14 Aug 2008
 */

#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Vector3.hh"
#include "math/Pose.hh"

#include "sdf/interface/Param.hh"

using namespace sdf;

//std::vector<Param*> *Param::params = NULL;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Param::Param(Param * /*_newParam*/) 
{
/*  if (params == NULL)
    gzthrow("Param vector is NULL\n");
  params->push_back(_newParam);
  */
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Param::~Param() 
{
}

////////////////////////////////////////////////////////////////////////////////
//  Begin a block of "new ParamT<>"
void Param::Begin(std::vector<Param*> * /*_params*/)
{
/*  if (params != NULL)
    gzthrow("Calling Begin before an End\n");
  params = _params;
  */
}

////////////////////////////////////////////////////////////////////////////////
//  End a block of "new ParamT<>"
void Param::End()
{
  /*if (params == NULL)
    gzthrow("Calling End before a Begin\n");

  params = NULL;
  */
}

////////////////////////////////////////////////////////////////////////////////
std::string Param::GetTypeName() const
{
  return this->typeName;
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsBool() const
{
  return this->GetTypeName() == typeid(bool).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsInt() const
{
  return this->GetTypeName() == typeid(int).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsUInt() const
{
  return this->GetTypeName() == typeid(unsigned int).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsFloat() const
{
  return this->GetTypeName() == typeid(float).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsDouble() const
{
  return this->GetTypeName() == typeid(double).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsChar() const
{
  return this->GetTypeName() == typeid(char).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsStr() const
{
  return this->GetTypeName() == typeid(std::string).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsVector3() const
{
  return this->GetTypeName() == typeid(gazebo::math::Vector3).name();
}

////////////////////////////////////////////////////////////////////////////////
bool Param::IsRotation() const
{
  return this->GetTypeName() == typeid(gazebo::math::Quatern).name();
}
