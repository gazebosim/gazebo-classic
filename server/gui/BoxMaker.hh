#ifndef BOXMAKER_HH
#define BOXMAKER_HH

#include "Vector2.hh"

namespace gazebo
{
  class MouseEvent;

  class BoxMaker
  {
    public: BoxMaker();
    public: virtual ~BoxMaker();
  
    public: void Start();
    public: void Stop();
    public: bool IsActive() const;

    public: void MousePushCB(const MouseEvent &event);
    public: void MouseReleaseCB(const MouseEvent &event);
    public: void MouseDragCB(const MouseEvent &event);
  
    private: void CreateTheBox();
    private: int state;
    private: bool leftMousePressed;
    private: Vector2<int> mousePushPos;
    private: std::string visualName;
    private: int index;
  };
}

#endif
