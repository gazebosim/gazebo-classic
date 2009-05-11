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
//#include <math.h>
//#include <fstream>
#include "Vector3.hh"

namespace gazebo
{

  class Color
  {
    // Constructors
    public: Color();
    public: Color( const float &r, const float &g, const float &b );
    public: Color( const float &r, const float &g, const float &b, 
                   const float &a );
    public: Color( const Color &clr );
  
    // Destructor
    public: virtual ~Color();

    public: void Reset();
  
    /// \brief Set the contents of the vector
    public: void Set(float r = 0, float g =0 , float b = 0, float a = 0);

    /// \brief Get the color in HSV colorspace
    public: Vector3 GetAsHSV() const;
 
    /// \brief Set a color based on HSV values
    public: void SetFromHSV(float h, float s, float v);

    /// \brief Get the color in YUV colorspace
    public: Vector3 GetAsYUV() const;

    /// \brief Set from yuv
    public: void SetFromYUV(float y, float u, float v);

    // Equal operator
    public: const Color &operator=( const Color &pt );

    /// \brief Array index operator
    public: float operator[](unsigned int index);

    /// \brief Get the red color
    public: float R();

    /// \brief Get the green color
    public: float G();

    /// \brief Get the blue color
    public: float B();

    /// \brief Get the alpha color
    public: float A();
  
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
    public: friend std::ostream &operator<< (std::ostream &out, const Color &pt) {
      out << pt.r << " " << pt.g << " " << pt.b << " " << pt.a;
      return out; 
    }

    // The values
    private: float r, g, b, a;
  };
}

#endif
