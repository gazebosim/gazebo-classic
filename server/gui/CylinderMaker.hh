#ifndef CYLINDERMAKER_HH
#define CYLINDERMAKER_HH

#include "Vector2.hh"

namespace gazebo
{
  class CylinderMaker
  {
    public: CylinderMaker();
    public: virtual ~CylinderMaker();
  
    public: void Start();
    public: void Stop();
    public: bool IsActive() const;

    public: void MousePushCB(Vector2<int> mousePos);
    public: void MouseReleaseCB(Vector2<int> mousePos);
    public: void MouseDragCB(Vector2<int> mousePos);
  
    private: void CreateTheCylinder();
    private: int state;
    private: bool leftMousePressed;
    private: Vector2<int> mousePushPos;
    private: std::string visualName;
    private: int index;
  };
}
#endif
