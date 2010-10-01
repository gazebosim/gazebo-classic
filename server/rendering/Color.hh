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
/* Desc: Color class
 * Author: Nate Koenig
 * Date: 08 May 2009
 * CVS: $Id$
 */

#ifndef COLOR_HH
#define COLOR_HH

#include <iostream>
#include "Vector3.hh"

#include <OGRE/OgreColourValue.h>

namespace gazebo
{

  class Color
  {
    /// \brief Constructor
    public: Color();

    /// \brief Constructor
    public: Color( float r, float g, float b, float a=1.0 );

    /// \brief Copy Constructor
    public: Color( const Color &clr );
  
    /// \brief Destructor
    public: virtual ~Color();

    /// \brief Reset the color to default values
    public: void Reset();
  
    /// \brief Set the contents of the vector
    public: void Set(float r = 1, float g =1 , float b = 1, float a = 1);

    /// \brief Get the color in HSV colorspace
    public: Vector3 GetAsHSV() const;
 
    /// \brief Set a color based on HSV values
    public: void SetFromHSV(float h, float s, float v);

    /// \brief Get the color in YUV colorspace
    public: Vector3 GetAsYUV() const;

    /// \brief Set from yuv
    public: void SetFromYUV(float y, float u, float v);

    /// \brief Equal operator
    public: const Color &operator=( const Color &pt );

    /// \brief Array index operator
    public: float operator[](unsigned int index);

    /// \brief Get the red color
    public: float R() const;

    /// \brief Get the green color
    public: float G() const;

    /// \brief Get the blue color
    public: float B() const;

    /// \brief Get the alpha color
    public: float A() const;

    /// \brief Return the equivalent ogre color
    public: Ogre::ColourValue GetOgreColor();
  
    // Addition operators
    public: Color operator+( const Color &pt ) const;
    public: Color operator+( const float &v ) const;
    public: const Color &operator+=( const Color &pt );
  
    // Subtraction operators 
    public: Color operator-( const Color &pt ) const;
    public: Color operator-( const float &pt ) const;
    public: const Color &operator-=( const Color &pt );
  
    // Division operators
    public: const Color operator/( const float &i ) const;
    public: const Color operator/( const Color &pt ) const;
    public: const Color &operator/=( const Color &pt );
  
    // Multiplication operators
    public: const Color operator*(const float &i) const;
    public: const Color operator*( const Color &pt ) const;
    public: const Color &operator*=( const Color &pt );
  
    // Equality operators
    public: bool operator==( const Color &pt ) const;
    public: bool operator!=( const Color &pt ) const;

    /// Clamp the color values
    private: void Clamp();

    // Ostream operator
    public: friend std::ostream &operator<< (std::ostream &out, const Color &pt)    {
      out << pt.r << " " << pt.g << " " << pt.b << " " << pt.a;
      return out; 
    }

    // Istream operator
    public: friend std::istream &operator>> (std::istream &in, Color &pt)
    { 
      // Skip white spaces
      in.setf( std::ios_base::skipws );
      in >> pt.r >> pt.g >> pt.b >> pt.a;
      return in; 
    }

    // The values
    private: float r, g, b, a;
  };
}

#endif
