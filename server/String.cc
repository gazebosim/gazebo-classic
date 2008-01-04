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
/* Desc: A helpfull String class, mostly a converter
 * Author: Jordi Polo
 * Date: 3 Jan 2008
 */

#include "String.hh"
#include "Vector3.hh"
#include "Vector2.hh"
#include "Quatern.hh"
#include <OgreColourValue.h>

using namespace gazebo;

String::String(const String& data)
{
  stream << data.GetStr();
}
    
String::String(Vector3 data)
{
  stream << data;
  //if (!(stream << data.x << " " << data.y << " " <<data.z))
  //  gztrow();  
}

String::String(Vector2<int> data)
{
  stream << data;
}

String::String(Vector2<double> data)
{
  stream << data;
}

String::String(Quatern data)
{
  stream << data;
}

String::String(bool data)
{
  if (data) 
    stream << "true";
  else
    stream << "false";
}


String::String(int data)
{
  stream << data;
}

String::String(double data)
{
  stream << data;
}

String::String(float data)
{
  stream << data;
}

String::String(const char* data)
{
  stream << data;
}

String::String(std::string data)
{
  stream << data;
}

String::String(Ogre::ColourValue* data)
{
  stream << data->r << " " <<data->g <<" " << data->b << " " << data->a;
}

String::~String()
{}

bool String::IsNull() const
{
  bool null=false;
  if (stream.str()==std::string())
    null=true;
  return null;
}
    
std::string String::GetStr() const
{
  return stream.str();
}

const char* String::GetCharStr() const
{
  return stream.str().c_str();
}
