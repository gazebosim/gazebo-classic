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

#ifndef STRING_HH
#define STRING_HH

#include <iostream>
#include <sstream>
#include <string>

#include "Vector3.hh"
#include "Vector2.hh"
#include "Quatern.hh"

namespace Ogre
{
 class ColourValue;
}

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief String conversions and tools
/// \{

  class String
  {
    //our types
    public: String(const String& data);
    public: String(Vector3 data);
    public: String(Vector2<int> data);
    public: String(Vector2<double> data);
    public: String(Quatern data);
    //common types
    public: String(bool data);
    public: String(int data);
    public: String(double data);
    public: String(float data);
    public: String(const char* data);
    //Ogre types
    public: String(Ogre::ColourValue* data);
    //libstd types
    public: String(std::string data);
    
    public: ~String();
    public: bool IsNull() const;
    public: std::string GetStr() const;
    public: const char* GetCharStr() const;
  
    //private: std::string str;
    private: std::ostringstream stream;
  };


/// \}
}

#endif
