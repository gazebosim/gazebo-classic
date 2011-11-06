#ifndef ANIMATION_HH
#define ANIMATION_HH

#include <string>
#include <vector>

namespace gazebo
{
  namespace math
  {
    class Spline;
    class RotationSpline;
  }

  namespace common
  {
    class KeyFrame;

    class Animation
    {
      public: Animation(const std::string _name, double _length, bool _loop);
      public: virtual ~Animation();

      public: double GetLength() const;
      public: void SetLength(double _len);

      public: KeyFrame *CreateKeyFrame(double _time);

      public: void SetTime(double _time);
      public: void AddTime(double _time);

      public: void GetInterpolatedKeyFrame(KeyFrame &_kf) const;

      protected: void GetInterpolatedKeyFrame(double _time, 
                                              KeyFrame &_kf) const;

      protected: void BuildInterpolationSplines() const;

      protected: double GetKeyFramesAtTime(double _time, KeyFrame **_kf1,
                                       KeyFrame **_kf2, 
                                       unsigned int &_firstKeyIndex) const;

      protected: std::string name;
      protected: double length;
      protected: double timePos;
      protected: mutable bool build;
      protected: bool loop;

      private: typedef std::vector<KeyFrame*> KeyFrame_V;
      private: KeyFrame_V keyFrames;
      private: mutable math::Spline *positionSpline;
      private: mutable math::RotationSpline *rotationSpline;
    };
  }
}

#endif
