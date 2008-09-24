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

/* Desc: Base class shared by all classes in Gazebo.
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * SVN: $Id$
 */

#include "Common.hh"
#include "GazeboMessage.hh"

using namespace gazebo;

unsigned int Common::idCounter = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Common::Common()
{
  this->id = ++idCounter;

  Param::Begin(&this->parameters);
  this->nameP = new ParamT<std::string>("name","noname",1);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Common::~Common()
{
  delete this->nameP;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the name of the entity
void Common::SetName(const std::string &name)
{
  this->nameP->SetValue( name );
}
  
////////////////////////////////////////////////////////////////////////////////
/// Return the name of the entity
std::string Common::GetName() const
{
  return this->nameP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the parameters 
std::vector<Param*> *Common::GetParams()
{
  return &this->parameters;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a parameter by name
Param *Common::GetParam(const std::string &key) const
{
  std::vector<Param*>::const_iterator iter;
  Param *result = NULL;

  for (iter = this->parameters.begin(); iter != this->parameters.end(); iter++)
  {
    if ((*iter)->GetKey() == key)
    {
      result = *iter;
      break;
    }
  }

  if (result == NULL)
    gzerr(0) << "Unable to find Param using key[" << key << "]\n";

  return result;
}
   
////////////////////////////////////////////////////////////////////////////////
/// Return the ID of this entity. This id is unique
int Common::GetId() const
{
  return this->id;
}
