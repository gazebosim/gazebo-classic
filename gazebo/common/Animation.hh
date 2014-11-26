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
#ifndef _ANIMATION_HH_
#define _ANIMATION_HH_

#include <string>
#include <vector>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    class Spline;
    class RotationSpline;
  }

  /// \ingroup gazebo_common
  /// \brief Common namespace

  namespace common
  {
    class KeyFrame;
    class PoseKeyFrame;
    class NumericKeyFrame;

    /// \addtogroup gazebo_common
    /// \{

    /// \class Animation Animation.hh common/common.hh
    /// \brief Manages an animation, which is a collection of keyframes and
    /// the ability to interpolate between the keyframes
    class GZ_COMMON_VISIBLE Animation
    {
      /// \brief Constructor
      /// \param[in] _name Name of the animation, should be unique
      /// \param[in] _length Duration of the animation in seconds
      /// \param[in] _loop Set to true if the animation should repeat
      public: Animation(const std::string &_name, double _length, bool _loop);

      /// \brief Destructor
      public: virtual ~Animation();

      /// \brief Return the duration of the animation
      /// \return Duration of the animation in seconds
      public: double GetLength() const;

      /// \brief Set the duration of the animation
      /// \param[in] _len The length of the animation in seconds
      public: void SetLength(double _len);

      /// \brief Set the current time position of the animation
      /// \param[in] _time The time position in seconds
      public: void SetTime(double _time);

      /// \brief Add time to the animation
      /// \param[in] _time The amount of time to add in seconds
      public: void AddTime(double _time);

      /// \brief Return the current time position
      /// \return The time position in seconds
      public: double GetTime() const;

      /// \brief Return the number of key frames in the animation
      /// \return The number of keyframes
      public: unsigned int GetKeyFrameCount() const;

      /// \brief Get a key frame using an index value
      /// \param[in] _index The index of the key frame
      /// \return A pointer the keyframe, NULL if the _index is invalid
      public: KeyFrame* GetKeyFrame(unsigned int _index) const;

      /// \brief Get the two key frames that bound a time value
      /// \param[in] _time The time in seconds
      /// \param[out] _kf1 Lower bound keyframe that is returned
      /// \param[out] _kf2 Upper bound keyframe that is returned
      /// \param[out] _firstKeyIndex Index of the lower bound key frame
      /// \return The time between the two keyframe
      protected: double GetKeyFramesAtTime(double _time, KeyFrame **_kf1,
                                           KeyFrame **_kf2,
                                           unsigned int &_firstKeyIndex) const;


      /// \brief animation name
      protected: std::string name;

      /// \brief animation duration
      protected: double length;

      /// \brief current time position
      protected: double timePos;

      /// \brief determines if the interpolation splines need building
      protected: mutable bool build;

      /// \brief true if animation repeats
      protected: bool loop;

      /// \brief array of keyframe type alias
      protected: typedef std::vector<KeyFrame*> KeyFrame_V;

      /// \brief array of key frames
      protected: KeyFrame_V keyFrames;
    };
    /// \}

    /// \addtogroup gazebo_common
    /// \{

    /// \brief A pose animation.
    class GZ_COMMON_VISIBLE PoseAnimation : public Animation
    {
      /// \brief Constructor
      /// \param[in] _name String name of the animation. This should be unique.
      /// \param[in] _length Length of the animation in seconds
      /// \param[in] _loop True == loop the animation
      public: PoseAnimation(const std::string &_name,
                            double _length, bool _loop);

      /// \brief Destructor
      public: virtual ~PoseAnimation();

      /// \brief Create a pose keyframe at the given time
      /// \param[in] _time Time at which to create the keyframe
      /// \return Pointer to the new keyframe
      public: PoseKeyFrame *CreateKeyFrame(double _time);

      /// \brief Get a keyframe using the animation's current time.
      /// \param[out] _kf PoseKeyFrame reference to hold the interpolated result
      public: void GetInterpolatedKeyFrame(PoseKeyFrame &_kf) const;

      /// \brief Get a keyframe using a passed in time.
      /// \param[in] _time Time in seconds
      /// \param[out] _kf PoseKeyFrame reference to hold the interpolated result
      protected: void GetInterpolatedKeyFrame(double _time,
                                              PoseKeyFrame &_kf) const;

      /// \brief Update the pose splines
      protected: void BuildInterpolationSplines() const;

      /// \brief smooth interpolation for position
      private: mutable math::Spline *positionSpline;

      /// \brief smooth interpolation for rotation
      private: mutable math::RotationSpline *rotationSpline;
    };
    /// \}

    /// \addtogroup gazebo_common
    /// \{

    /// \brief A numeric animation.
    class GZ_COMMON_VISIBLE NumericAnimation : public Animation
    {
      /// \brief Constructor
      /// \param[in] _name String name of the animation. This should be unique.
      /// \param[in] _length Length of the animation in seconds
      /// \param[in] _loop True == loop the animation
      public: NumericAnimation(const std::string &_name,
                               double _length, bool _loop);

      /// \brief Destructor
      public: virtual ~NumericAnimation();

      /// \brief Create a numeric keyframe at the given time
      /// \param[in] _time Time at which to create the keyframe
      /// \return Pointer to the new keyframe
      public: NumericKeyFrame *CreateKeyFrame(double _time);

      /// \brief Get a keyframe using the animation's current time.
      /// \param[out] _kf NumericKeyFrame reference to hold the
      /// interpolated result
      public: void GetInterpolatedKeyFrame(NumericKeyFrame &_kf) const;
    };
    /// \}
  }
}

#endif
