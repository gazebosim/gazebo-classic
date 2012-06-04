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
#ifndef __KEYFRAME_HH__
#define __KEYFRAME_HH__

#include "math/Vector3.hh"
#include "math/Quaternion.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief A key frame in an animation
    class KeyFrame
    {
      /// \brief Constructor
      /// \param _time Time of the keyframe in seconds
      public: KeyFrame(double _time);

      /// \brief Destructor
      public: virtual ~KeyFrame();

      /// \brief Get the time of the keyframe
      public: double GetTime() const;

      protected: double time;
    };

    /// \brief A keyframe for a PoseAnimation
    class PoseKeyFrame : public KeyFrame
    {
      /// \brief Constructor
      /// \param Time of the keyframe
      public: PoseKeyFrame(double _time);

      /// \brief Destructor
      public: virtual ~PoseKeyFrame();

      /// \brief Set the translation for the keyframe
      /// \param _trans Translation amount
      public: void SetTranslation(const math::Vector3 &_trans);

      /// \brief Get the translation of the keyframe
      /// \return The translation amount
      public: const math::Vector3 &GetTranslation() const;

      /// \brief Set the rotation for the keyframe
      /// \param _trans Rotation amount
      public: void SetRotation(const math::Quaternion &_rot);

      /// \brief Get the rotation of the keyframe
      /// \return The rotation amount
      public: const math::Quaternion &GetRotation() const;

      protected: math::Vector3 translate;
      protected: math::Quaternion rotate;
    };

    /// \brief A keyframe for a NumericAnimation
    class NumericKeyFrame : public KeyFrame
    {
      /// \brief Constructor
      /// \param Time of the keyframe
      public: NumericKeyFrame(double _time);

      /// \brief Destructor
      public: virtual ~NumericKeyFrame();

      /// \brief Set the value of the keyframe
      /// \param _value The new value
      public: void SetValue(const double &_value);

      /// \brief Get the value of the keyframe
      /// \return the value of the keyframe
      public: const double &GetValue() const;

      protected: double value;
    };
    /// \}
  }
}
#endif
