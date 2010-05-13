#ifndef BOXMAKER_HH
#define BOXMAKER_HH

#include "Vector2.hh"

namespace gazebo
{
  class BoxMaker
  {
    public: BoxMaker();
    public: virtual ~BoxMaker();
  
    public: void Start();
    public: void Stop();
    public: bool IsActive() const;

    public: void MousePushCB(Vector2<int> mousePos);
    public: void MouseReleaseCB(Vector2<int> mousePos);
    public: void MouseDragCB(Vector2<int> mousePos);
  
    private: void CreateTheBox();
    private: int state;
    private: bool leftMousePressed;
    private: Vector2<int> mousePushPos;
    private: std::string visualName;
    private: int index;
  };
}

#endif
