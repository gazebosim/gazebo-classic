/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _CONVERSIONS_HH_
#define _CONVERSIONS_HH_

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Color.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
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
      public: static Ogre::ColourValue Convert(const common::Color &_clr);

      /// \brief Return the equivalent gazebo color
      /// \param[in] _clr Ogre color to convert
      /// \return Gazebo color value
      public: static common::Color Convert(const Ogre::ColourValue &_clr);

      /// \brief return Ogre Vector from Gazebo Vector3
      /// \param[in] _v Gazebo vector
      /// \return Ogre vector
      public: static Ogre::Vector3 Convert(const math::Vector3 &_v);

      /// \brief return gazebo Vector from ogre Vector3
      /// \param[in] _v Ogre vector
      /// \return Gazebo vector
      public: static math::Vector3 Convert(const Ogre::Vector3 &_v);

      /// \brief Gazebo quaternion to Ogre quaternion
      /// \param[in] _v Gazebo quaternion
      /// \return Ogre quaternion
      public: static Ogre::Quaternion Convert(const math::Quaternion &_v);

      /// \brief Ogre quaternion to Gazebo quaternion
      /// \param[in] _v Ogre quaternion
      /// return Gazebo quaternion
      public: static math::Quaternion Convert(const Ogre::Quaternion &_v);
    };
    /// \}
  }
}
#endif
