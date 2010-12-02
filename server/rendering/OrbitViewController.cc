#include "Global.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
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
  : ViewController(camera), distance(5.0f)
{
  Vector3 rpy = this->camera->GetWorldPose().rot.GetAsEuler();

  this->yaw = rpy.z;
  this->pitch = rpy.y;

  this->focalPoint.Set(0,0,0);
  this->distance = this->camera->GetWorldPosition().Distance(this->focalPoint);
  this->refVisual = OgreCreator::Instance()->CreateVisual("", NULL, NULL, this->camera->GetScene());
  this->refVisual->AttachMesh("unit_sphere");
  this->refVisual->SetScale(Vector3(0.2,0.2,0.1));
  this->refVisual->SetCastShadows(false);
  this->refVisual->SetMaterial("Gazebo/YellowTransparent");
  this->refVisual->SetVisible(false);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OrbitViewController::~OrbitViewController()
{
  if (this->refVisual)
    OgreCreator::Instance()->DeleteVisual(this->refVisual);
}

////////////////////////////////////////////////////////////////////////////////
// Update
void OrbitViewController::Update()
{
  Vector3 pos;
  pos.x = this->distance * cos( this->yaw ) * sin( this->pitch );
  pos.z = this->distance * cos( this->pitch );
  pos.y = this->distance * sin( this->yaw ) * sin( this->pitch );

  pos += this->focalPoint;

  this->camera->SetWorldPosition(pos);

  Quatern rot;
  rot.SetFromEuler( Vector3(0, M_PI*0.5 - this->pitch, this->yaw - M_PI) );
  this->camera->SetWorldRotation(rot);
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
    this->refVisual->SetVisible(true);
    this->yaw += drag.x * event.moveScale * -0.1;
    this->pitch += drag.y * event.moveScale * -0.1;

    this->NormalizeYaw(this->yaw);
    this->NormalizePitch(this->pitch);
  }
  else if (event.middle == MouseEvent::SCROLL)
  {
    this->refVisual->SetVisible(true);
    distance +=  50.0 * event.scroll.y * event.moveScale;
  }
  else if (event.right == MouseEvent::DOWN)
  {
    this->refVisual->SetVisible(true);
    this->Translate(Vector3(0, drag.x * event.moveScale, drag.y * event.moveScale));
  }
  else if (event.middle == MouseEvent::DOWN)
  {
    this->refVisual->SetVisible(true);
    this->Translate(Vector3(drag.y * event.moveScale,0,0));
  }
  else
    this->refVisual->SetVisible(false);
}

////////////////////////////////////////////////////////////////////////////////
// Translate the focal point
void OrbitViewController::Translate(Vector3 vec)
{
  this->focalPoint += this->camera->GetWorldPose().rot * vec;
  this->refVisual->SetPosition(this->focalPoint);
}

////////////////////////////////////////////////////////////////////////////////
// Normalize yaw value
void OrbitViewController::NormalizeYaw(float &v)
{
  v = fmod(v, M_PI*2);
  if (v < 0.0f)
  {
    v = M_PI * 2 + v;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Normalize pitch value
void OrbitViewController::NormalizePitch(float &v)
{
  if (v < PITCH_LIMIT_LOW)
    v = PITCH_LIMIT_LOW;
  else if (v > PITCH_LIMIT_HIGH)
    v = PITCH_LIMIT_HIGH;
}
