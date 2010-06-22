#include <Ogre.h>
#include "Simulator.hh"
#include "Global.hh"
#include "OgreCamera.hh"
#include "Vector2.hh"
#include "MouseEvent.hh"
#include "OrbitViewController.hh"

using namespace gazebo;

static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = M_PI - 0.001;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
OrbitViewController::OrbitViewController(OgreCamera *camera)
  : ViewController(camera), yaw(M_PI), pitch(M_PI*.5), distance(5.0f)
{
  this->focalPoint.Set(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OrbitViewController::~OrbitViewController()
{
}

////////////////////////////////////////////////////////////////////////////////
// Activate the controller
//void OrbitViewController::Activate()
//{
//}

////////////////////////////////////////////////////////////////////////////////
// Update
void OrbitViewController::Update()
{
  Vector3 pos;
  pos.x = this->distance * cos( this->yaw ) * sin( this->pitch ) + this->focalPoint.x;
  pos.z = this->distance * cos( this->pitch ) + this->focalPoint.z;
  pos.y = this->distance * sin( this->yaw ) * sin( this->pitch ) + this->focalPoint.y;

  this->camera->SetPosition(pos);
  Pose3d pose = this->camera->GetCameraWorldPose();

  Vector3 vec = pose.rot * (this->focalPoint - pos);

  this->camera->SetDirection(vec);
  //this->camera->GetOgreCamera()->setDirection(Ogre::Vector3(vec.x, vec.y, vec.z));

  //camera_->setFixedYawAxis(true, reference_node_->getOrientation() * Ogre::Vector3::UNIT_Y);
  //camera_->setDirection(reference_node_->getOrientation() * (focal_point_ - pos));

  //focal_shape_->setPosition(focal_point_);
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse event
void OrbitViewController::HandleMouseEvent(const MouseEvent &event)
{
  if (!this->camera->GetUserMovable())
    return;

  Vector2<int> drag = event.pos - event.prevPos;

  Vector3 directionVec(0,0,0);

  if (event.left == MouseEvent::DOWN)
  {
    this->yaw += drag.x * event.moveScale * 0.1;
    this->pitch += drag.y * event.moveScale *0.1;

    this->yaw = fmod(this->yaw, M_PI*2);
    if (this->yaw < 0.0f)
      this->yaw = M_PI * 2 + this->yaw;

    if (this->pitch < PITCH_LIMIT_LOW)
      this->pitch = PITCH_LIMIT_LOW;
    else if (this->pitch > PITCH_LIMIT_HIGH)
      this->pitch = PITCH_LIMIT_HIGH;

    //this->camera->RotateYaw(DTOR(drag.x * 0.1));
    //this->camera->RotatePitch(DTOR(-drag.y * 0.1));
  }
  /*else if (event.right == MouseEvent::DOWN)
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
  this->directionVec.Set(0,0,0);
  */
}
