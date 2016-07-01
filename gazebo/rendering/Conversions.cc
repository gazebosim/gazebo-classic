/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
Ogre::Vector3 Conversions::Convert(const math::Vector3 &_v)
{
  return Ogre::Vector3(_v.x, _v.y, _v.z);
}

//////////////////////////////////////////////////
math::Vector3 Conversions::Convert(const Ogre::Vector3 &_v)
{
  return math::Vector3(_v.x, _v.y, _v.z);
}

//////////////////////////////////////////////////
ignition::math::Vector3d Conversions::ConvertIgn(const Ogre::Vector3 &_v)
{
  return ignition::math::Vector3d(_v.x, _v.y, _v.z);
}

//////////////////////////////////////////////////
Ogre::Vector3 Conversions::Convert(const ignition::math::Vector3d &_v)
{
  return Ogre::Vector3(_v.X(), _v.Y(), _v.Z());
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

//////////////////////////////////////////////////
ignition::math::Quaterniond Conversions::ConvertIgn(const Ogre::Quaternion &_v)
{
  return ignition::math::Quaterniond(_v.w, _v.x, _v.y, _v.z);
}

//////////////////////////////////////////////////
Ogre::Quaternion Conversions::Convert(const ignition::math::Quaterniond &_q)
{
  return Ogre::Quaternion(_q.W(), _q.X(), _q.Y(), _q.Z());
}

//////////////////////////////////////////////////
ignition::math::Matrix4d Conversions::ConvertIgn(const Ogre::Matrix4 &_m)
{
  return ignition::math::Matrix4d(_m[0][0], _m[0][1], _m[0][2], _m[0][3],
                                  _m[1][0], _m[1][1], _m[1][2], _m[1][3],
                                  _m[2][0], _m[2][1], _m[2][2], _m[2][3],
                                  _m[3][0], _m[3][1], _m[3][2], _m[3][3]);
}

//////////////////////////////////////////////////
Ogre::Matrix4 Conversions::Convert(const ignition::math::Matrix4d &_m)
{
  return Ogre::Matrix4(_m(0, 0), _m(0, 1), _m(0, 2), _m(0, 3),
                       _m(1, 0), _m(1, 1), _m(1, 2), _m(1, 3),
                       _m(2, 0), _m(2, 1), _m(2, 2), _m(2, 3),
                       _m(3, 0), _m(3, 1), _m(3, 2), _m(3, 3));
}

//////////////////////////////////////////////////
ReferenceFrame Conversions::Convert(const Ogre::Node::TransformSpace &_ts)
{
  switch (_ts)
  {
    case Ogre::Node::TS_LOCAL:
      return RF_LOCAL;
    case Ogre::Node::TS_PARENT:
      return RF_PARENT;
    case Ogre::Node::TS_WORLD:
      return RF_WORLD;
    default:
      return RF_LOCAL;
  }
}

//////////////////////////////////////////////////
Ogre::Node::TransformSpace Conversions::Convert(const ReferenceFrame &_rf)
{
  switch (_rf)
  {
    case RF_LOCAL:
      return Ogre::Node::TS_LOCAL;
    case RF_PARENT:
      return Ogre::Node::TS_PARENT;
    case RF_WORLD:
      return Ogre::Node::TS_WORLD;
    default:
      return Ogre::Node::TS_LOCAL;
  }
}
