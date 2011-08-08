#include <QtGui>
#include <QX11Info>

#include <math.h>

#include "common/Exception.hh"
#include "math/gzmath.h"

#include "transport/transport.h"

#include "rendering/Rendering.hh"
#include "rendering/Visual.hh"
#include "rendering/WindowManager.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/OrbitViewController.hh"

#include "gui/Gui.hh"
#include "gui/GuiEvents.hh"
#include "gui/GLWidget.hh"

using namespace gazebo;
using namespace gui;

GLWidget::GLWidget( QWidget *parent )
  : QWidget(parent)
{
  this->windowId = -1;

  setAttribute(Qt::WA_OpaquePaintEvent,true);
  setAttribute(Qt::WA_PaintOnScreen,true);
//  setMinimumSize(320,240);

  this->renderFrame = new QFrame;
  this->renderFrame->setLineWidth(1);
  this->renderFrame->setFrameShadow(QFrame::Sunken);
  this->renderFrame->setFrameShape(QFrame::Box);
  this->renderFrame->show();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->renderFrame);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  this->connections.push_back( 
      gui::Events::ConnectMoveModeSignal( 
        boost::bind(&GLWidget::OnMoveMode, this, _1) ) );

  this->connections.push_back( 
      gui::Events::ConnectCreateEntitySignal( 
        boost::bind(&GLWidget::OnCreateEntity, this, _1) ) );


  this->entityMaker = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->posePub = this->node->Advertise<msgs::Pose>("~/set_pose");
  this->selectionSub = this->node->Subscribe("~/selection", &GLWidget::OnSelectionMsg, this);
}

GLWidget::~GLWidget()
{
}

void GLWidget::showEvent(QShowEvent *event)
{
  QApplication::flush();
  this->windowId = rendering::WindowManager::Instance()->CreateWindow(
      this->GetOgreHandle(), this->width(), this->height());

  QWidget::showEvent(event);

  if (this->userCamera)
    rendering::WindowManager::Instance()->SetCamera(this->windowId, 
                                                    this->userCamera);
}

void GLWidget::moveEvent(QMoveEvent *e)
{
  QWidget::moveEvent(e);

  if(e->isAccepted() && this->windowId >= 0)
  {
    rendering::WindowManager::Instance()->Moved(this->windowId);
  }
}

void GLWidget::paintEvent(QPaintEvent *e)
{
  if (this->userCamera)
  {
    event::Events::preRenderSignal();

    // Tell all the cameras to render
    event::Events::renderSignal();

    event::Events::postRenderSignal();

    //this->userCamera->Render();
    //this->userCamera->PostRender();
  }
  e->accept();
}

void GLWidget::resizeEvent(QResizeEvent *e)
{
  if (this->windowId >= 0)
  {
    rendering::WindowManager::Instance()->Resize( this->windowId, 
        e->size().width(), e->size().height());
    this->userCamera->Resize(e->size().width(), e->size().height());
  }
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
  this->mouseEvent.pressPos.Set( event->pos().x(), event->pos().y() );
  this->mouseEvent.prevPos = this->mouseEvent.pressPos;

  this->mouseEvent.left = event->buttons() & Qt::LeftButton ? common::MouseEvent::DOWN : common::MouseEvent::UP;
  this->mouseEvent.right = event->buttons() & Qt::RightButton ? common::MouseEvent::DOWN : common::MouseEvent::UP;
  this->mouseEvent.middle = event->buttons() & Qt::MidButton ? common::MouseEvent::DOWN : common::MouseEvent::UP;
  this->mouseEvent.dragging = false;

  if (this->entityMaker)
    this->entityMaker->OnMousePush(this->mouseEvent);
  else if (this->selection)
  {
    this->scene->GetVisualAt(this->userCamera, this->mouseEvent.pressPos, 
                             this->selectionMod);
  }
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
  this->mouseEvent.scroll.y = event->delta() > 0 ? -1 : 1;
  this->mouseEvent.middle = common::MouseEvent::SCROLL;
  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
  this->mouseEvent.pos.Set( event->pos().x(), event->pos().y() );
  this->mouseEvent.dragging = true;

  if (this->entityMaker)
    this->entityMaker->OnMouseDrag(this->mouseEvent);
  else if (this->selection && !this->selectionMod.empty())
  {
    if (this->selectionMod.substr(0,3) == "rot")
      this->RotateEntity(this->selection);
    else
      this->TranslateEntity(this->selection);
  }
  else
    this->userCamera->HandleMouseEvent(this->mouseEvent);

  this->mouseEvent.prevPos = this->mouseEvent.pos;
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
  this->mouseEvent.pos.Set( event->pos().x(), event->pos().y() );
  this->mouseEvent.prevPos = this->mouseEvent.pos;

  this->mouseEvent.left = event->buttons() & Qt::LeftButton ? common::MouseEvent::DOWN : common::MouseEvent::UP;
  this->mouseEvent.right = event->buttons() & Qt::RightButton ? common::MouseEvent::DOWN : common::MouseEvent::UP;
  this->mouseEvent.middle = event->buttons() & Qt::MidButton ? common::MouseEvent::DOWN : common::MouseEvent::UP;

  emit clicked();

  if (this->entityMaker)
    this->entityMaker->OnMouseRelease(this->mouseEvent);
  else if (this->mouseEvent.dragging == false && 
           event->button() & Qt::RightButton)
  {
    //this->selection = this->scene->SelectVisualAt(this->userCamera, this->mouseEvent.pos);
    std::string mod;
    this->selection = this->scene->GetVisualAt(this->userCamera, 
                                               this->mouseEvent.pos, mod);

    // Get the model associated with the visual
    this->selection = this->selection->GetParent()->GetParent();
    this->scene->SelectVisual(this->selection->GetName());
  }
  else if ( !this->selectionMod.empty() && this->selection)
  {
    msgs::Pose msg;

    msgs::Init(msg, this->selection->GetName());
    msgs::Set( &msg, this->selection->GetWorldPose());

    this->posePub->Publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Create the camera
void GLWidget::ViewScene(rendering::ScenePtr _scene)
{
  if (_scene->GetUserCameraCount() == 0)
    this->userCamera = _scene->CreateUserCamera("rc_camera");
  else
    this->userCamera = _scene->GetUserCamera(0);

  gui::set_active_camera(this->userCamera);
  this->scene =_scene;

  this->userCamera->SetWorldPosition( math::Vector3(-5,0,5) );
  this->userCamera->SetWorldRotation( math::Quaternion::EulerToQuaternion(0, DTOR(15), 0) );

  if (this->windowId >= 0)
    rendering::WindowManager::Instance()->SetCamera(this->windowId, this->userCamera);
}


////////////////////////////////////////////////////////////////////////////////
rendering::UserCameraPtr GLWidget::GetCamera() const
{
  return this->userCamera;
}

////////////////////////////////////////////////////////////////////////////////
std::string GLWidget::GetOgreHandle() const
{
  std::string handle;

#ifdef WIN32
  handle = boost::lexical_cast<std::string>(this->winId());
#else
  QX11Info info = x11Info();
  QWidget *q_parent = dynamic_cast<QWidget*>(this->renderFrame);
  handle = boost::lexical_cast<std::string>((unsigned long)info.display());
  handle += ":";
  handle += boost::lexical_cast<std::string>((unsigned int)info.screen());
  handle += ":";
  assert(q_parent);
  handle += boost::lexical_cast<std::string>((unsigned long)q_parent->winId());
#endif

  return handle;
}

void GLWidget::CreateEntity(const std::string &name)
                            //const EntityMaker::CreateCallback &cb)
{
  if (this->entityMaker)
    this->entityMaker->Stop();

  if (name == "box")
    this->entityMaker = &this->boxMaker;
  else if (name == "sphere")
    this->entityMaker = &this->sphereMaker;
  else if (name == "cylinder")
    this->entityMaker = &this->cylinderMaker;
  else
    this->entityMaker = NULL;

  if (this->entityMaker)
  {
    // TODO: change the cursor to a cross
    this->entityMaker->Start(this->userCamera);
  }
  else
  {
    // TODO: make sure cursor state stays at the default
  }
}

void GLWidget::OnMoveMode(bool mode)
{
  if (mode)
  {
    // TODO: set cursor to default state
    this->entityMaker = NULL;
  }
}

void GLWidget::OnCreateEntity( const std::string &_type )
{
  this->CreateEntity(_type);
}

void GLWidget::RotateEntity( rendering::VisualPtr &_vis )
{
  math::Vector3 planeNorm, planeNorm2;
  math::Vector3 p1, p2;
  math::Vector3 a,b;
  math::Vector3 ray(0,0,0);

  math::Pose pose = _vis->GetPose();

  // Figure out which axis to rotate around
  if (this->selectionMod == "rotx")
    ray.x = 1.0;
  else if (this->selectionMod == "roty")
    ray.y = 1.0;
  else if (this->selectionMod == "rotz")
    ray.z = 1.0;

  // Compute the normal to the plane on which to rotate
  planeNorm = pose.rot.RotateVector(ray);
  double d = -pose.pos.GetDotProd(planeNorm);

  p1 = this->userCamera->GetWorldPointOnPlane( this->mouseEvent.pos.x,
       this->mouseEvent.pos.y, planeNorm, d);

  p2 = this->userCamera->GetWorldPointOnPlane( this->mouseEvent.prevPos.x,
       this->mouseEvent.prevPos.y, planeNorm, d);

  // Get point vectors relative to the entity's pose
  a = p1 - _vis->GetWorldPose().pos;
  b = p2 - _vis->GetWorldPose().pos;

  a.Normalize();
  b.Normalize();

  // Get the angle between the two vectors. This is the amount to
  // rotate the entity 
  float angle = acos(a.GetDotProd(b));
  if (isnan(angle))
    angle = 0;

  // Compute the normal to the plane which is defined by the
  // direction of rotation
  planeNorm2 = a.GetCrossProd(b);
  planeNorm2.Normalize();

  // Switch rotation direction if the two normals don't line up
  if ( planeNorm.GetDotProd(planeNorm2) > 0)
    angle *= -1;

    math::Quaternion delta;
    delta.SetFromAxis( ray.x, ray.y, ray.z,angle);

    _vis->SetRotation(pose.rot * delta);
   
    //TODO: send message

/*  if (entity->GetType() == Entity::MODEL)
  {
    Quatern delta;
    delta.SetFromAxis( ray.x, ray.y, ray.z,angle);

    pose.rot = pose.rot * delta;
    entity->SetWorldPose(pose);
  }
  else
  {
    ((Body*)entity)->SetTorque(planeNorm * angle * Gui::forceMultiplier);
  }
*/
}

void GLWidget::TranslateEntity( rendering::VisualPtr &_vis )
{
  math::Pose pose = _vis->GetPose();

  math::Vector3 origin1, dir1, p1;
  math::Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.pos.x,
      this->mouseEvent.pos.y, origin1, dir1);
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.prevPos.x,
      this->mouseEvent.prevPos.y, origin2, dir2);

  math::Vector3 moveVector(0,0,0);
  math::Vector3 planeNorm(0,0,1);
  if (this->selectionMod == "transx")
    moveVector.x = 1;
  else if (this->selectionMod == "transy")
    moveVector.y = 1;
  else if (this->selectionMod == "transz")
  {
    moveVector.z = 1;
    planeNorm.Set(1,0,0);
  }

  // Compute the distance from the camera to plane of translation
  double d = -pose.pos.GetDotProd(planeNorm);
  double dist1 = origin1.GetDistToPlane(dir1, planeNorm, d);
  double dist2 = origin2.GetDistToPlane(dir2, planeNorm, d);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  p2 = origin2 + dir2 * dist2;

  moveVector *= p1 - p2;
  pose.pos += moveVector;

  _vis->SetPose(pose);

  /*if (entity->GetType() == Entity::MODEL)
  {
    pose.pos += moveVector;
    entity->SetRelativePose(pose);
  }
  else if (entity->GetType() == Entity::BODY)
  {
    Body *body = (Body*)(entity);
    moveVector *= Gui::forceMultiplier;
    body->SetForce(moveVector);
  }
*/

}

void GLWidget::OnSelectionMsg(const boost::shared_ptr<msgs::Selection const> &_msg)
{
  this->selection = this->scene->GetVisual(_msg->header().str_id() );
}
