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
    class PoseKeyFrame;
    class NumericKeyFrame;

    class Animation
    {
      public: Animation(const std::string _name, double _length, bool _loop);
      public: virtual ~Animation();

      public: double GetLength() const;
      public: void SetLength(double _len);

      public: void SetTime(double _time);
      public: void AddTime(double _time);
      public: double GetTime() const;

      protected: double GetKeyFramesAtTime(double _time, KeyFrame **_kf1,
                                           KeyFrame **_kf2, 
                                           unsigned int &_firstKeyIndex) const;


      protected: std::string name;
      protected: double length;
      protected: double timePos;
      protected: mutable bool build;
      protected: bool loop;

      protected: typedef std::vector<KeyFrame*> KeyFrame_V;
      protected: KeyFrame_V keyFrames;
    };

    class PoseAnimation : public Animation
    {
      public: PoseAnimation(const std::string _name, 
                            double _length, bool _loop);
      public: virtual ~PoseAnimation();

      public: PoseKeyFrame *CreateKeyFrame(double _time);

      public: void GetInterpolatedKeyFrame(PoseKeyFrame &_kf) const;

      protected: void GetInterpolatedKeyFrame(double _time, 
                                              PoseKeyFrame &_kf) const;

      protected: void BuildInterpolationSplines() const;

      private: mutable math::Spline *positionSpline;
      private: mutable math::RotationSpline *rotationSpline;
    };

    class NumericAnimation : public Animation
    {
      public: NumericAnimation(const std::string _name, 
                               double _length, bool _loop);
      public: virtual ~NumericAnimation();

      public: NumericKeyFrame *CreateKeyFrame(double _time);

      public: void GetInterpolatedKeyFrame(NumericKeyFrame &_kf) const;
    };
  }
}

#endif
