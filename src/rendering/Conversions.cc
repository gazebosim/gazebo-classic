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
#include "rendering/Conversions.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
/// Return the equivalent ogre color
Ogre::ColourValue Conversions::Convert(const common::Color &color)
{
  return Ogre::ColourValue(color.R(), color.G(), color.B(), color.A());
}

//////////////////////////////////////////////////
// Ogre Vector from gazebo Vector3
Ogre::Vector3 Conversions::Convert(const math::Vector3 &v)
{
  return Ogre::Vector3(v.x, v.y, v.z);
}

//////////////////////////////////////////////////
/// \brief return gazebo Vector from ogre Vector3
math::Vector3 Conversions::Convert(const Ogre::Vector3 &v)
{
  return math::Vector3(v.x, v.y, v.z);
}

//////////////////////////////////////////////////
/// \brief Gazebo quaternion to Ogre quaternion
Ogre::Quaternion Conversions::Convert(const math::Quaternion &v)
{
  return Ogre::Quaternion(v.w, v.x, v.y, v.z);
}

//////////////////////////////////////////////////
/// \brief Ogre quaternion to Gazebo quaternion
math::Quaternion Conversions::Convert(const Ogre::Quaternion &v)
{
  return math::Quaternion(v.w, v.x, v.y, v.z);
}


