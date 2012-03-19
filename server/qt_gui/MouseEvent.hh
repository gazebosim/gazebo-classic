#ifndef MOUSEEVENT_HH
#define MOUSEEVENT_HH

#include "Vector2.hh"

namespace gazebo
{
  class MouseEvent
  {
    public: enum ButtonState {DOWN, UP, SCROLL};

    public: MouseEvent()
            : pos(0,0), prevPos(0,0), pressPos(0,0), scroll(0,0),
              moveScale(0.01),dragging(false), left(UP), right(UP), middle(UP)
            {}

    public: Vector2<int> pos; 
    public: Vector2<int> prevPos;
    public: Vector2<int> pressPos; 
    public: Vector2<int> scroll; 

    public: float moveScale;

    public: bool dragging;

    public: ButtonState left;
    public: ButtonState right;
    public: ButtonState middle;
  };
};
#endif
