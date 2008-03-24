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
 */
/* Desc: A helpfull StringValue class, mostly a converter
 * Author: Jordi Polo
 * Date: 3 Jan 2008
 * SVN: $Id:$
 */

#include "Vector3.hh"
#include "Vector2.hh"
#include "Quatern.hh"
#include <OgreColourValue.h>

#include "StringValue.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Copy Constructor
StringValue::StringValue(const StringValue& data)
{
  stream << data.GetStr();
}

////////////////////////////////////////////////////////////////////////////////
/// Convert Vector3 to string
StringValue::StringValue(Vector3 data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert Vector2<int> to string
StringValue::StringValue(Vector2<int> data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert Vector2<double> to string
StringValue::StringValue(Vector2<double> data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert Quatern to string
StringValue::StringValue(Quatern data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert bool to string
StringValue::StringValue(bool data)
{
  if (data)
    stream << "true";
  else
    stream << "false";
}


////////////////////////////////////////////////////////////////////////////////
/// Convert int to string
StringValue::StringValue(int data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert double to string
StringValue::StringValue(double data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert float to string
StringValue::StringValue(float data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert float to string
StringValue::StringValue(const char* data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// Ogre colour value to string
StringValue::StringValue(std::string data)
{
  stream << data;
}

////////////////////////////////////////////////////////////////////////////////
/// A std string
StringValue::StringValue(Ogre::ColourValue* data)
{
  stream << data->r << " " <<data->g <<" " << data->b << " " << data->a;
}

////////////////////////////////////////////////////////////////////////////////
/// destructor
StringValue::~StringValue()
{}

////////////////////////////////////////////////////////////////////////////////
/// True if the string is null
bool StringValue::IsNull() const
{
  bool null=false;
  if (stream.str()==std::string())
    null=true;
  return null;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the value as a std string
std::string StringValue::GetStr() const
{
  return stream.str();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the value as a char string
const char* StringValue::GetCharStr() const
{
  return stream.str().c_str();
}
