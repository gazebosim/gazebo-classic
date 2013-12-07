
#include "gazebo/gui/OculusWindow.hh"
#include "gazebo/rendering/OculusCamera.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/WindowManager.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
OculusWindow::OculusWindow(QWidget *_parent)
  : QWidget(_parent)
{
  setAttribute(Qt::WA_NativeWindow, true);
  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_PaintOnScreen, true);

  this->windowId = -1;

  this->setObjectName("oculusWindow");

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Oculus"));

  this->renderFrame = new QFrame;
  this->renderFrame->setFrameShape(QFrame::NoFrame);
  this->renderFrame->setSizePolicy(QSizePolicy::Expanding,
                                   QSizePolicy::Expanding);
  this->renderFrame->setContentsMargins(0, 0, 0, 0);
  this->renderFrame->show();

  //mainFrame->setFrameShape(QFrame::NoFrame);
  //mainFrame->show();

  //this->glWidget = new GLWidget(mainFrame);

  QVBoxLayout *renderLayout = new QVBoxLayout;
  renderLayout->addWidget(this->renderFrame);
  renderLayout->setContentsMargins(0, 0, 0, 0);
  // renderLayout->setSpacing(0);

  // QFrame *renderFrame = new QFrame;
  //renderFrame->setLayout(renderLayout);

  //mainFrame->setLayout(renderLayout);

  //QVBoxLayout *mainLayout = new QVBoxLayout;
  //mainLayout->addWidget(mainFrame);
  //mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(renderLayout);
  this->isFullScreen = false;
}

/////////////////////////////////////////////////
OculusWindow::~OculusWindow()
{
}

/////////////////////////////////////////////////
void OculusWindow::keyPressEvent(QKeyEvent *_event)
{
  // Toggle full screen
  if (_event->key() == Qt::Key_F11)
  {
    if (this->isFullScreen)
      this->showFullScreen();
    else
      this->showNormal();

    this->isFullScreen = !this->isFullScreen;
  }
}

/////////////////////////////////////////////////
void OculusWindow::resizeEvent(QResizeEvent *_e)
{
  if (!this->scene)
    return;

  if (this->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Resize(
        this->windowId, _e->size().width(), _e->size().height());
    this->oculusCamera->Resize(_e->size().width(), _e->size().height());
  }
}

/////////////////////////////////////////////////
void OculusWindow::showEvent(QShowEvent *_event)
{
  this->scene = rendering::get_scene();

  this->oculusCamera = this->scene->CreateOculusCamera("gzclient_camera");
  this->oculusCamera->AttachToVisual("atlas::head", true);

  math::Vector3 camPos(0.1, 0, 0);
  math::Vector3 lookAt(0, 0, 0);
  math::Vector3 delta = lookAt - camPos;

  double yaw = atan2(delta.y, delta.x);

  double pitch = atan2(-delta.z, sqrt(delta.x*delta.x + delta.y*delta.y));
  this->oculusCamera->SetWorldPose(math::Pose(camPos,
        math::Vector3(0, pitch, yaw)));

  std::cout << "Create Oculus renderw window[" << this->GetOgreHandle() << "] Width[" << this->width() << "] height[" << this->height() << "]\n";
  this->windowId = rendering::RenderEngine::Instance()->GetWindowManager()->
    CreateWindow(this->GetOgreHandle(), this->width(), this->height());

  QWidget::showEvent(_event);

  if (this->oculusCamera)
    rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
        this->windowId, this->oculusCamera);

  this->setFocus();

  QSize winSize;
  winSize.setWidth(1280);
  winSize.setHeight(800);
  this->resize(winSize);
}

//////////////////////////////////////////////////
std::string OculusWindow::GetOgreHandle() const
{
  std::string ogreHandle;

#if defined(WIN32) || defined(__APPLE__)
  ogreHandle = boost::lexical_cast<std::string>(this->winId());
#else
  QX11Info info = x11Info();
  QWidget *q_parent = dynamic_cast<QWidget*>(this->renderFrame);
  ogreHandle = boost::lexical_cast<std::string>(
      reinterpret_cast<uint64_t>(info.display()));
  ogreHandle += ":";
  ogreHandle += boost::lexical_cast<std::string>(
      static_cast<uint32_t>(info.screen()));
  ogreHandle += ":";
  assert(q_parent);
  ogreHandle += boost::lexical_cast<std::string>(
      static_cast<uint64_t>(q_parent->winId()));
#endif

  return ogreHandle;
}
