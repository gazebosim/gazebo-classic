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
