/*
 * Copyright 2011 Nate Koenig
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
/* Desc: Color class
 * Author: Nate Koenig
 * Date: 08 May 2009
 */

#ifndef GAZEBO_COLOR_HH
#define GAZEBO_COLOR_HH

#include <iostream>
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/math/Vector3.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \brief Defines a color
    class Color
    {
      /// \brief (1, 1, 1)
      public: static const Color White;
      /// \brief (0, 0, 0)
      public: static const Color Black;
      /// \brief (1, 0, 0)
      public: static const Color Red;
      /// \brief (0, 1, 0)
      public: static const Color Green;
      /// \brief (0, 0, 1)
      public: static const Color Blue;
      /// \brief (1, 1, 0)
      public: static const Color Yellow;
      /// \brief (1, 0, 1)
      public: static const Color Purple;

      public: typedef unsigned int RGBA;
      public: typedef unsigned int BGRA;
      public: typedef unsigned int ARGB;
      public: typedef unsigned int ABGR;

      /// \brief Constructor
      public: Color();

      /// \brief Constructor
      /// \param _r Red value
      /// \param _g Green value
      /// \param _b Blue value
      /// \param _a Alpha value (1=opaque)
      public: Color(float _r, float _g, float _b, float _a = 1.0);

      /// \brief Copy Constructor
      /// \param _clr Color to copy
      public: Color(const Color &_clr);

      /// \brief Destructor
      public: virtual ~Color();

      /// \brief Reset the color to default values
      public: void Reset();

      /// \brief Set the contents of the vector
      /// \param _r Red value
      /// \param _g Green value
      /// \param _b Blue value
      /// \param _a Alpha value (1=opaque)
      public: void Set(float _r = 1, float _g = 1 , float _b = 1, float _a = 1);

      /// \brief Get the color in HSV colorspace
      /// \return HSV values in a math::Vector3 format
      public: math::Vector3 GetAsHSV() const;

      /// \brief Set a color based on HSV values
      /// \param _h Hue(0..360)
      /// \param _s Saturation(0..1)
      /// \param _v Value(0..1)
      public: void SetFromHSV(float _h, float _s, float _v);

      /// \brief Get the color in YUV colorspace
      public: math::Vector3 GetAsYUV() const;

      /// \brief Set from yuv
      /// \param _y value
      /// \param _u value
      /// \param _v value
      public: void SetFromYUV(float _y, float _u, float _v);

      /// \brief Equal operator
      /// \param _pt Color to copy
      /// \return Reference to this color
      public: Color &operator =(const Color &_pt);

      /// \brief Array index operator
      /// \param _index Color component index(0=red, 1=green, 2=blue)
      public: float operator[](unsigned int _index);

      /// \brief Get the red color
      /// \return red color component
      public: float R() const GAZEBO_DEPRECATED;

      /// \brief Get the green color
      /// \return green color component
      public: float G() const GAZEBO_DEPRECATED;

      /// \brief Get the blue color
      /// \return blue color component
      public: float B() const GAZEBO_DEPRECATED;

      /// \brief Get the alpha color
      /// \return alpha value
      public: float A() const GAZEBO_DEPRECATED;

      /// \brief Set the red color
      /// \param _r Red color component
      public: void R(float _r) GAZEBO_DEPRECATED;

      /// \brief Set the green color
      /// \param _g Green color component
      public: void G(float _g) GAZEBO_DEPRECATED;

      /// \brief Set the blue color
      /// \param _b Blue color component
      public: void B(float _b) GAZEBO_DEPRECATED;

      /// \brief Set the alpha color
      /// \param _a Alpha value
      public: void A(float _a) GAZEBO_DEPRECATED;

      /// \brief Get as uint32 RGBA packed value
      public: RGBA GetAsRGBA() const;

      /// \brief Get as uint32 BGRA packed value
      public: BGRA GetAsBGRA() const;

      /// \brief Get as uint32 ARGB packed value
      public: ARGB GetAsARGB() const;

      /// \brief Get as uint32 ABGR packed value
      public: ABGR GetAsABGR() const;


      /// \brief Set from uint32 RGBA packed value
      public: void SetFromRGBA(const RGBA _v);

      /// \brief Set from uint32 BGRA packed value
      public: void SetFromBGRA(const BGRA _v);

      /// \brief Set from uint32 ARGB packed value
      public: void SetFromARGB(const ARGB _v);

      /// \brief Set from uint32 ABGR packed value
      public: void SetFromABGR(const ABGR _v);

      /// \brief Addition operator (this + _pt)
      /// \param _pt Color to add
      /// \return The resulting color
      public: Color operator+(const Color &_pt) const;

      /// \brief Add _v to all color components
      /// \param _v Value to add to each color component
      /// \return The resulting color
      public: Color operator+(const float &_v) const;

      /// \brief Addition equal operator
      /// \param _pt Color to add
      /// \return The resulting color
      public: const Color &operator+=(const Color &_pt);

      /// \brief Subtraction operator
      /// \param _pt The color to substract
      /// \return The resulting color
      public: Color operator-(const Color &_pt) const;

      /// \brief Subtract _v from all color components
      /// \param _v Value to subtract
      /// \return The resulting color
      public: Color operator-(const float &_v) const;

      /// \brief Subtraction equal operator
      /// \param _pt Color to subtract
      /// \return The resulting color
      public: const Color &operator-=(const Color &_pt);

      /// \brief Division operator
      /// \param _pt Color to divide by
      /// \return The resulting color
      public: const Color operator/(const Color &_pt) const;

      /// \brief Divide all color component by _v
      /// \param _v The value to divide by
      /// \return The resulting color
      public: const Color operator/(const float &_v) const;

      /// \brief Division equal operator
      /// \param _pt Color to divide by
      /// \return The resulting color
      public: const Color &operator/=(const Color &_pt);

      /// \brief Multiplication operator
      /// \param _pt The color to muliply by
      /// \return The resulting color
      public: const Color operator*(const Color &_pt) const;

      /// \brief Multiply all color components by _v
      /// \param _v The value to multiply by
      /// \return The resulting color
      public: const Color operator*(const float &_v) const;

      /// \brief Multiplication equal operator
      /// \param _pt The color to muliply by
      /// \return The resulting color
      public: const Color &operator*=(const Color &_pt);

      /// \brief Equality operator
      /// \param _pt The color to check for equality
      /// \return True if the this color equals _pt
      public: bool operator ==(const Color &_pt) const;

      /// \brief Inequality operator
      /// \param _pt The color to check for inequality
      /// \return True if the this color does not equal _pt
      public: bool operator!=(const Color &_pt) const;

      /// \brief Clamp the color values to valid ranges
      private: void Clamp();

      /// \brief Ostream operator
      public: friend std::ostream &operator<< (std::ostream &_out,
                                               const Color &_pt)
              {
                _out << _pt.r << " " << _pt.g << " " << _pt.b << " " << _pt.a;
                return _out;
              }

      /// \brief Istream operator
      public: friend std::istream &operator>> (std::istream &in, Color &pt)
      {
        // Skip white spaces
        in.setf(std::ios_base::skipws);
        in >> pt.r >> pt.g >> pt.b >> pt.a;
        return in;
      }

      // The values
      public: float r, g, b, a;
    };
    /// \}
  }
}
#endif
