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

      public: void SetTranslate(const math::Vector3 &_trans);
      public: const math::Vector3 &GetTranslate() const;

      public: void SetRotation(const math::Quaternion &_rot);
      public: const math::Quaternion &GetRotation() const;

      protected: double time;
      protected: math::Vector3 translate;
      protected: math::Quaternion rotate;
    };
  }
}
#endif
