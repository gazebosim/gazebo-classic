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
#ifndef _KEYFRAME_HH_
#define _KEYFRAME_HH_

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class KeyFrame KeyFrame.hh common/common.hh
    /// \brief A key frame in an animation
    class GZ_COMMON_VISIBLE KeyFrame
    {
      /// \brief Constructor
      /// \param[in] _time Time of the keyframe in seconds
      public: KeyFrame(double _time);

      /// \brief Destructor
      public: virtual ~KeyFrame();

      /// \brief Get the time of the keyframe
      /// \return the time
      public: double GetTime() const;

      /// \brief time of key frame
      protected: double time;
    };

    /// \brief A keyframe for a PoseAnimation
    class GZ_COMMON_VISIBLE PoseKeyFrame : public KeyFrame
    {
      /// \brief Constructor
      /// \param[in] _time of the keyframe
      public: PoseKeyFrame(double _time);

      /// \brief Destructor
      public: virtual ~PoseKeyFrame();

      /// \brief Set the translation for the keyframe
      /// \param[in] _trans Translation amount
      public: void SetTranslation(const math::Vector3 &_trans);

      /// \brief Get the translation of the keyframe
      /// \return The translation amount
      public: const math::Vector3 &GetTranslation() const;

      /// \brief Set the rotation for the keyframe
      /// \param[in] _rot Rotation amount
      public: void SetRotation(const math::Quaternion &_rot);

      /// \brief Get the rotation of the keyframe
      /// \return The rotation amount
      public: const math::Quaternion &GetRotation() const;

      /// \brief the translation vector
      protected: math::Vector3 translate;

      /// \brief the rotation quaternion
      protected: math::Quaternion rotate;
    };

    /// \brief A keyframe for a NumericAnimation
    class GZ_COMMON_VISIBLE NumericKeyFrame : public KeyFrame
    {
      /// \brief Constructor
      /// \param[in] _time Time of the keyframe
      public: NumericKeyFrame(double _time);

      /// \brief Destructor
      public: virtual ~NumericKeyFrame();

      /// \brief Set the value of the keyframe
      /// \param[in] _value The new value
      public: void SetValue(const double &_value);

      /// \brief Get the value of the keyframe
      /// \return the value of the keyframe
      public: const double &GetValue() const;

      /// \brief numeric value
      protected: double value;
    };
    /// \}
  }
}
#endif
