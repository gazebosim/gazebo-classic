#include <QtGui>
#include <QX11Info>

#include <math.h>

#include "common/Exception.hh"

#include "rendering/Rendering.hh"
#include "rendering/WindowManager.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/OrbitViewController.hh"

#include "GLWidget.hh"

using namespace gazebo;
using namespace gui;

GLWidget::GLWidget( QWidget *parent )
  : QWidget(parent)
{
  this->windowId = -1;

  setAttribute(Qt::WA_OpaquePaintEvent,true);
  setAttribute(Qt::WA_PaintOnScreen,true);
  setMinimumSize(320,240);

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
      event::Events::ConnectMoveModeSignal( 
        boost::bind(&GLWidget::OnMoveMode, this, _1) ) );

  this->entityMaker = NULL;
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
}

////////////////////////////////////////////////////////////////////////////////
/// Create the camera
void GLWidget::ViewScene(rendering::ScenePtr scene)
{
  if (scene->GetUserCameraCount() == 0)
    this->userCamera = scene->CreateUserCamera("rc_camera");
  else
    this->userCamera = scene->GetUserCamera(0);

  this->userCamera->SetWorldPosition( common::Vector3(-5,0,5) );
  this->userCamera->SetWorldRotation( common::Quatern::EulerToQuatern(0, DTOR(15), 0) );

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
