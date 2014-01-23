/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/rendering/Conversions.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
Ogre::ColourValue Conversions::Convert(const common::Color &_color)
{
  return Ogre::ColourValue(_color.r, _color.g, _color.b, _color.a);
}

//////////////////////////////////////////////////
common::Color Conversions::Convert(const Ogre::ColourValue &_clr)
{
  return common::Color(_clr.r, _clr.g, _clr.b, _clr.a);
}

//////////////////////////////////////////////////
Ogre::Vector3 Conversions::Convert(const math::Vector3 &v)
{
  return Ogre::Vector3(v.x, v.y, v.z);
}

//////////////////////////////////////////////////
math::Vector3 Conversions::Convert(const Ogre::Vector3 &v)
{
  return math::Vector3(v.x, v.y, v.z);
}

//////////////////////////////////////////////////
Ogre::Quaternion Conversions::Convert(const math::Quaternion &_v)
{
  return Ogre::Quaternion(_v.w, _v.x, _v.y, _v.z);
}

//////////////////////////////////////////////////
math::Quaternion Conversions::Convert(const Ogre::Quaternion &_v)
{
  return math::Quaternion(_v.w, _v.x, _v.y, _v.z);
}



