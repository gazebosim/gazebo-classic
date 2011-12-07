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

      public: void SetTranslate(const math::Vector3 &_trans);
      public: const math::Vector3 &GetTranslate() const;

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
