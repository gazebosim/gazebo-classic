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
/// \brief StringValue conversions and tools
/// \{

  class StringValue
  {
    /// \brief Copy Constructor
    public: StringValue(const StringValue& data);

    /// \brief Convert Vector3 to string
    public: StringValue(Vector3 data);

    /// \brief Convert Vector2<int> to string
    public: StringValue(Vector2<int> data);

    /// \brief Convert Vector2<double> to string
    public: StringValue(Vector2<double> data);

    /// \brief Convert Quatern to string
    public: StringValue(Quatern data);

    /// \brief Convert bool to string
    public: StringValue(bool data);

    /// \brief Convert int to string
    public: StringValue(int data);

    /// \brief Convert double to string
    public: StringValue(double data);

    /// \brief Convert float to string
    public: StringValue(float data);

    /// \brief Convert float to string
    public: StringValue(const char* data);

    /// \brief Ogre colour value to string
    public: StringValue(Ogre::ColourValue* data);

    /// \brief A std string
    public: StringValue(std::string data);
    
    /// \brief destructor
    public: ~StringValue();

    /// \brief True if the string is null
    public: bool IsNull() const;

    /// \brief Get the value as a std string
    public: std::string GetStr() const;

    /// \brief Get the value as a char string
    public: const char* GetCharStr() const;
  
    //private: std::string str;
    private: std::ostringstream stream;
  };


/// \}
}

#endif
