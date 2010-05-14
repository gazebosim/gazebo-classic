#include "Simulator.hh"
#include "Global.hh"
#include "OgreCamera.hh"
#include "Vector2.hh"
#include "MouseEvent.hh"
#include "FPSViewController.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
FPSViewController::FPSViewController(OgreCamera *camera)
  : ViewController(camera)
{
  this->directionVec.x = 0;
  this->directionVec.y = 0;
  this->directionVec.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
FPSViewController::~FPSViewController()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update
void FPSViewController::Update()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse event
void FPSViewController::HandleMouseEvent(const MouseEvent &event)
{
  if (!this->camera->GetUserMovable())
    return;

  Vector2<int> drag = event.pos - event.prevPos;

  this->directionVec.Set(0,0,0);

  if (event.left == MouseEvent::DOWN)
  {
    this->camera->RotateYaw(DTOR(drag.x * 0.1));
    this->camera->RotatePitch(DTOR(-drag.y * 0.1));
  }
  else if (event.right == MouseEvent::DOWN)
  {
    // interactively pan view
    this->directionVec.x = 0;
    this->directionVec.y =  drag.x * 0.01;//this->moveAmount;
    this->directionVec.z =  drag.y * 0.01;//this->moveAmount;
  }
  else if (event.middle == MouseEvent::DOWN)
  {
    this->directionVec.x =  drag.y * 0.01;//this->moveAmount;
    this->directionVec.y =  0;
    this->directionVec.z =  0;
  }
  else if (event.middle == MouseEvent::SCROLL)
  {
    this->directionVec.x -=  50.0 * event.scroll.y * 0.01;//this->moveAmount;
    this->directionVec.y =  0;
    this->directionVec.z =  0;
  }

  this->camera->Translate(directionVec);
  this->directionVec.Set(0,0,0);
}
