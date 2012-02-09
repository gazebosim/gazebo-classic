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
#ifndef KEYFRAME_HH
#define KEYFRAME_HH

#include "math/Vector3.hh"
#include "math/Quaternion.hh"

namespace gazebo
{
  namespace common
  {
    class KeyFrame
    {
      public: KeyFrame(double _time);
      public: virtual ~KeyFrame();

      public: double GetTime() const;

      protected: double time;
    };

    class PoseKeyFrame : public KeyFrame
    {
      public: PoseKeyFrame(double _time);
      public: virtual ~PoseKeyFrame();

      public: void SetTranslation(const math::Vector3 &_trans);
      public: const math::Vector3 &GetTranslation() const;

      public: void SetRotation(const math::Quaternion &_rot);
      public: const math::Quaternion &GetRotation() const;

      protected: math::Vector3 translate;
      protected: math::Quaternion rotate;
    };

    class NumericKeyFrame : public KeyFrame
    {
      public: NumericKeyFrame(double _time);
      public: virtual ~NumericKeyFrame();


      public: void SetValue(const double &_value);
      public: const double &GetValue() const;

      protected: double value;
    };
  }
}
#endif


