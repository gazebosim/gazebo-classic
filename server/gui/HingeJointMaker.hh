#ifndef HINGEJOINTMAKER_HH
#define HINGEJOINTMAKER_HH

#include "Vector2.hh"

namespace gazebo
{
  class Entity;

  class HingeJointMaker
  {
    public: HingeJointMaker();
    public: virtual ~HingeJointMaker();

    public: void Start();
    public: void Stop();
    public: bool IsActive() const;

    public: void MousePushCB(Vector2<int> mousePos);
    public: void MouseReleaseCB(Vector2<int> mousePos);
    public: void MouseDragCB(Vector2<int> mousePos);
  
    private: void CreateTheHingeJoint();
    private: int state;
    private: bool leftMousePressed;
    private: Vector2<int> mousePushPos;
    private: std::string jointName;

    private: Entity *first, *second;
  };
}

#endif
