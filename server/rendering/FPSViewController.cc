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

  Vector3 directionVec(0,0,0);

  if (event.left == MouseEvent::DOWN)
  {
    this->camera->RotateYaw(DTOR(drag.x * 0.1));
    this->camera->RotatePitch(DTOR(-drag.y * 0.1));
  }
  else if (event.right == MouseEvent::DOWN)
  {
    // interactively pan view
    directionVec.x = 0;
    directionVec.y =  drag.x * event.moveScale;
    directionVec.z =  drag.y * event.moveScale;
  }
  else if (event.middle == MouseEvent::DOWN)
  {
    directionVec.x =  drag.y * event.moveScale;
    directionVec.y =  0;
    directionVec.z =  0;
  }
  else if (event.middle == MouseEvent::SCROLL)
  {
    directionVec.x -=  50.0 * event.scroll.y * event.moveScale;
    directionVec.y =  0;
    directionVec.z =  0;
  }

  this->camera->Translate(directionVec);
}
