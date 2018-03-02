/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_CONVERSIONS_HH_
#define GAZEBO_RENDERING_CONVERSIONS_HH_

#include <ignition/math/Color.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Color.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \brief Conversions Conversions.hh rendering/Conversions.hh
    /// \brief A set of utility function to convert between Gazebo and Ogre
    /// data types
    class GZ_RENDERING_VISIBLE Conversions
    {
      /// \brief Return the equivalent ogre color
      /// \param[in] _clr Gazebo color to convert
      /// \return Ogre color value
      /// \deprecated use ignition::math::Color
      public: static Ogre::ColourValue Convert(const common::Color &_clr)
              GAZEBO_DEPRECATED(9.0);

      /// \brief Return the equivalent ogre color
      /// \param[in] _clr color to convert
      /// \return Ogre color value
      public: static Ogre::ColourValue Convert(
                  const ignition::math::Color &_clr);

      /// \brief Return the equivalent color
      /// \param[in] _clr Ogre color to convert
      /// \return igntion math color value
      public: static ignition::math::Color Convert(
                  const Ogre::ColourValue &_clr);

      /// \brief Return ignition::math::Vector3d from Ogre Vector3.
      /// \param[in] _v Ogre Vector3
      /// \return Ignition math Vector3d
      public: static ignition::math::Vector3d ConvertIgn(
          const Ogre::Vector3 &_v);

      /// \brief Return Ogre Vector3 from ignition::math::Vector3d
      /// \param[in] _v Ignition math Vector3d
      /// \return Ogre Vector3
      public: static Ogre::Vector3 Convert(const ignition::math::Vector3d &_v);

      /// \brief Ogre quaternion to ignition::math::Quaterniond
      /// \param[in] _q Ogre quaternion
      /// \return Ignition math quaternion
      public: static ignition::math::Quaterniond ConvertIgn(
                  const Ogre::Quaternion &_q);

      /// \brief ignition::math::Quaterniond to Ogre quaternion
      /// \param[in] _q Ignition math quaternion
      /// \return Ogre quaternion
      public: static Ogre::Quaternion Convert(
                  const ignition::math::Quaterniond &_q);

      /// \brief Ogre Matrix4 to ignition math Matrix4d
      /// \param[in] _m Ogre Matrix4
      /// \return ignition math Matrix4d
      public: static ignition::math::Matrix4d ConvertIgn(
          const Ogre::Matrix4 &_m);

      /// \brief Ignition math Matrix4d to Ogre Matrix4
      /// \param[in] _m ignition math Matrix4d
      /// \return Ogre Matrix4
      public: static Ogre::Matrix4 Convert(const ignition::math::Matrix4d &_m);

      /// \brief Return the equivalent ogre transform space
      /// \param[in] _rf gazebo reference frame to convert
      /// \return Ogre node transform space
      public: static Ogre::Node::TransformSpace Convert(
          const ReferenceFrame &_rf);

      /// \brief Return the equivalent gazebo reference frame
      /// \param[in] _ts Ogre node transform space to convert
      /// \return Gazebo reference frame
      public: static ReferenceFrame Convert(
          const Ogre::Node::TransformSpace &_ts);
    };
    /// \}
  }
}
#endif
