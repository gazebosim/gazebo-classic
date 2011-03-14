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
/* Desc: Color class
 * Author: Nate Koenig
 * Date: 08 May 2009
 * CVS: $Id$
 */

#include <algorithm>
#include <math.h>

#include "GazeboMessage.hh"
#include "Color.hh"

using namespace gazebo;
using namespace common;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Color::Color()
  : r(0), g(0), b(0), a(0)
{
  this->Clamp();
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Color::Color( float r, float g, float b, const float a)
  : r(r), g(g), b(b), a(a)
{
  this->Clamp();
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
Color::Color( const Color &pt )
  : r(pt.r), g(pt.g), b(pt.b), a(pt.a)
{
  this->Clamp();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Color::~Color()
{
}

////////////////////////////////////////////////////////////////////////////////
// Reset the vector
void Color::Reset()
{
  this->r = this->g = this->b = this->a = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the contents of the vector
void Color::Set(float r, float g, float b, float a)
{
  this->r = r;
  this->g = g;
  this->b = b;
  this->a = a;

  this->Clamp();
}

////////////////////////////////////////////////////////////////////////////////
/// Set a color based on HSV values
void Color::SetFromHSV(float h, float s, float v)
{
  int i;
  float f, p , q, t;

  h = (int)(h) % 360;

  if (s==0)
  {
    //acromatic (grey)
    this->r = this->g = this->b = v;
    return;
  }

  h /= 60; // sector 0 - 5
  i = (int)floor(h);

  f = h - i;

  p = v * (1-s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1-f));

  switch(i)
  {
    case 0:
      this->r = v;
      this->g = t;
      this->b = p;
      break;
    case 1:
      this->r = q;
      this->g = v;
      this->b = p;
      break;
    case 2:
      this->r = p;
      this->g = v;
      this->b = t;
      break;
    case 3:
      this->r = p;
      this->g = q;
      this->b = v;
      break;
    case 4:
      this->r = t;
      this->g = p;
      this->b = v;
      break;
    case 5:
      this->r = v;
      this->g = p;
      this->b = q;
      break;
  }

  this->Clamp();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the color in HSV colorspace
Vector3 Color::GetAsHSV() const
{
  Vector3 hsv;
  float x,v, f, i;

  x = std::min(this->r, std::min(this->g, this->b));
  v = std::max(this->r, std::max(this->g, this->b));

  if(v == x) 
  {
    gzerr(0) << "rgb to hsv undefined\n";
    return hsv;
  }

  if (r == x)
    f = this->g - this->b;
  else if (this->g == x)
    f = this->b - this->r;
  else 
    f = this->r - this->g;

  if (this->r == x)
    i = 3;
  else if (this->g == x)
    i = 5;
  else
    i = 1;

  hsv.x = i - f /(v - x);
  hsv.y = (v - x)/v;
  hsv.z = v; 

  return hsv;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the color in YUV colorspace
Vector3 Color::GetAsYUV() const
{
  Vector3 yuv;

  yuv.x =  0.299*this->r + 0.587*this->g + 0.114*this->b;
  yuv.y = -0.1679*this->r - 0.332*this->g + 0.5*this->b + 0.5;
  yuv.z =  0.5*this->r - 0.4189*this->g - 0.08105*this->b + 0.5;

  yuv.x = yuv.x < 0 ? 0: yuv.x;
  yuv.x = yuv.x > 255 ? 255.0: yuv.x;

  yuv.y = yuv.y < 0 ? 0: yuv.y;
  yuv.y = yuv.y > 255 ? 255.0: yuv.y;

  yuv.z = yuv.z < 0 ? 0: yuv.z;
  yuv.z = yuv.z > 255 ? 255.0: yuv.z;



  /*if (yuv.x > 255)
    yuv.x = 255;
  if (yuv.x < 0)
    yuv.x = 0;

  if (yuv.y > 255)
    yuv.y = 255;
  if (yuv.y < 0)
    yuv.y = 0;

  if (yuv.z > 255)
    yuv.z = 255;
  if (yuv.z < 0)
    yuv.z = 0;
    */

  return yuv;
}

////////////////////////////////////////////////////////////////////////////////
// Set from yuv
void Color::SetFromYUV(float y, float u, float v)
{
  this->r = y + 1.140*v;
  this->g = y - 0.395*u - 0.581*v;
  this->b = y + 2.032*u;
  this->Clamp();
}

////////////////////////////////////////////////////////////////////////////////
// Array index operator
float Color::operator[](unsigned int index)
{
  switch(index)
  {
    case 0: return this->R();
    case 1: return this->G();
    case 2: return this->B();
    case 3: return this->A();
    default: gzerr(0) << "Invalid color index[" << index << "]\n";
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Get the red color
float Color::R() const
{
  return this->r;
}

////////////////////////////////////////////////////////////////////////////////
// Get the green color
float Color::G() const
{
  return this->g;
}

////////////////////////////////////////////////////////////////////////////////
// Get the blue color
float Color::B() const
{
  return this->b;
}

////////////////////////////////////////////////////////////////////////////////
// Get the alpha color
float Color::A() const
{
  return this->a;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the red color
void Color::R(float v)
{
  this->r = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the green color
void Color::G(float v)
{
  this->g = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the blue color
void Color::B(float v)
{
  this->b = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the alpha color
void Color::A(float v)
{
  this->a = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the equivalent ogre color
Ogre::ColourValue Color::GetOgreColor() const
{
  return Ogre::ColourValue(this->r, this->g, this->b, this->a);
}
 
////////////////////////////////////////////////////////////////////////////////
// Equals operator
const Color &Color::operator=( const Color &pt )
{
  this->r = pt.r;
  this->g = pt.g;
  this->b = pt.b;
  this->a = pt.a;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Addition operator
Color Color::operator+( const Color &pt ) const
{
  return Color(this->r + pt.r, this->g + pt.g, this->b + pt.b, this->a + pt.a);
}

Color Color::operator+( const float &v ) const
{
  return Color(this->r + v, this->g + v, this->b + v, this->a + v);
}

const Color &Color::operator+=( const Color &pt )
{
  this->r += pt.r;
  this->g += pt.g;
  this->b += pt.b;
  this->a += pt.a;

  this->Clamp();

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Subtraction operators
Color Color::operator-( const Color &pt ) const
{
  return Color(this->r - pt.r, this->g - pt.g, this->b - pt.b, this->a - pt.a);
}

Color Color::operator-( const float &v ) const
{
  return Color(this->r - v, this->g - v, this->b - v, this->a - v);
}

const Color &Color::operator-=( const Color &pt )
{
  this->r -= pt.r;
  this->g -= pt.g;
  this->b -= pt.b;
  this->a -= pt.a;

  this->Clamp();

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Division operators
const Color Color::operator/( const float &i ) const
{
  return Color(this->r / i, this->g / i, this->b / i, this->a / i);
}

const Color Color::operator/( const Color &pt ) const
{
  return Color(this->r / pt.r, this->g / pt.g, this->b / pt.b, this->a / pt.a);
}

const Color &Color::operator/=( const Color &pt )
{
  this->r /= pt.r;
  this->g /= pt.g;
  this->b /= pt.b;
  this->a /= pt.a;

  this->Clamp();

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Mulitplication operators
const Color Color::operator*(const float &i) const
{
  return Color(this->r * i, this->g * i, this->b * i, this->a * i);
}

const Color Color::operator*( const Color &pt ) const
{
  return Color(this->r * pt.r, this->g * pt.g, this->b * pt.b, this->a * pt.a);
}

const Color &Color::operator*=( const Color &pt )
{
  this->r *= pt.r;
  this->g *= pt.g;
  this->b *= pt.b;
  this->a *= pt.a;

  this->Clamp();

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Equality operator
bool Color::operator==( const Color &pt ) const
{
  return this->r == pt.r && this->g == pt.g && this->b == pt.b && this->a == pt.a;
}

////////////////////////////////////////////////////////////////////////////////
// Inequality operator
bool Color::operator!=( const Color &pt ) const
{
  return !(*this == pt);
}

////////////////////////////////////////////////////////////////////////////////
/// Clamp the color values
void Color::Clamp()
{
  this->r = this->r < 0 ? 0: this->r;
  this->r = this->r > 1 ? this->r/255.0: this->r;

  this->g = this->g < 0 ? 0: this->g;
  this->g = this->g > 1 ? this->g/255.0: this->g;

  this->b = this->b < 0 ? 0: this->b;
  this->b = this->b > 1 ? this->b/255.0: this->b;
}


