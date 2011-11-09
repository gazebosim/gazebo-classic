#include <QtGui>
#include <iomanip>

#include "rendering/UserCamera.hh"
#include "rendering/Rendering.hh"

#include "gui/Gui.hh"
#include "gui/GLWidget.hh"
#include "gui/GuiEvents.hh"
#include "gui/RenderWidget.hh"

using namespace gazebo;
using namespace gui;

RenderWidget::RenderWidget( QWidget *parent )
  : QWidget(parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->mainFrame = new QFrame;
  this->mainFrame->setLineWidth(1);
  this->mainFrame->setFrameShadow(QFrame::Sunken);
  this->mainFrame->setFrameShape(QFrame::Box);
  this->mainFrame->show();

  QVBoxLayout *frameLayout = new QVBoxLayout;

  this->glWidget = new GLWidget(this->mainFrame);
  rendering::ScenePtr scene = rendering::create_scene(gui::get_world(), true);
  this->glWidget->ViewScene( scene );

  this->xPosEdit = new QLineEdit;
  this->xPosEdit->setValidator(new QDoubleValidator(this->xPosEdit) );
  this->xPosEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->xPosEdit->setFixedWidth(100);

  this->yPosEdit = new QLineEdit;
  this->yPosEdit->setValidator(new QDoubleValidator(this->yPosEdit) );
  this->yPosEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->yPosEdit->setFixedWidth(100);

  this->zPosEdit = new QLineEdit;
  this->zPosEdit->setValidator(new QDoubleValidator(this->zPosEdit) );
  this->zPosEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->zPosEdit->setFixedWidth(100);

  this->rollEdit = new QLineEdit;
  this->rollEdit->setValidator(new QDoubleValidator(this->rollEdit) );
  this->rollEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->rollEdit->setFixedWidth(100);

  this->pitchEdit = new QLineEdit;
  this->pitchEdit->setValidator(new QDoubleValidator(this->pitchEdit) );
  this->pitchEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->pitchEdit->setFixedWidth(100);

  this->yawEdit = new QLineEdit;
  this->yawEdit->setValidator(new QDoubleValidator(this->yawEdit) );
  this->yawEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->yawEdit->setFixedWidth(100);

  /*this->fpsEdit = new QLineEdit;
  this->fpsEdit->setReadOnly(true);
  this->fpsEdit->setFixedWidth(50);

  this->trianglesEdit = new QLineEdit;
  this->trianglesEdit->setReadOnly(true);
  this->trianglesEdit->setFixedWidth(80);
  */ 

  this->xyzLabel = new QLabel(tr("XYZ:"));
  this->rpyLabel = new QLabel(tr("RPY:"));
  //QLabel *fpsLabel = new QLabel(tr("FPS:"));
  //QLabel *trianglesLabel = new QLabel(tr("Triangles:"));

  bottomBarLayout = new QHBoxLayout;
  bottomBarLayout->addWidget(this->xyzLabel);
  bottomBarLayout->addWidget(this->xPosEdit);
  bottomBarLayout->addWidget(this->yPosEdit);
  bottomBarLayout->addWidget(this->zPosEdit);

  bottomBarLayout->addItem(new QSpacerItem(10,20));
  bottomBarLayout->addWidget(this->rpyLabel);
  bottomBarLayout->addWidget(this->rollEdit);
  bottomBarLayout->addWidget(this->pitchEdit);
  bottomBarLayout->addWidget(this->yawEdit);

  bottomBarLayout->addItem(new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum));
  //bottomBarLayout->addWidget(fpsLabel);
  //bottomBarLayout->addWidget(this->fpsEdit);
  //bottomBarLayout->addWidget(trianglesLabel);
  //bottomBarLayout->addWidget(this->trianglesEdit);
  bottomBarLayout->addSpacing(10);

  frameLayout->addWidget(this->glWidget);
  frameLayout->addLayout(bottomBarLayout);

  this->mainFrame->setLayout(frameLayout);
  this->mainFrame->layout()->setContentsMargins(4,4,4,4);

  mainLayout->addWidget(this->mainFrame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  QTimer *timer = new QTimer(this);
  connect( timer, SIGNAL(timeout()), this, SLOT(update()) );
  timer->start(33);

  this->connections.push_back( 
      gui::Events::ConnectFullScreen( 
        boost::bind(&RenderWidget::OnFullScreen, this, _1) ) );
}

RenderWidget::~RenderWidget()
{
  delete this->glWidget;
}

void RenderWidget::OnFullScreen(bool &_value)
{
  if (_value)
  {
    this->mainFrame->layout()->removeItem(this->bottomBarLayout);
    this->mainFrame->setLineWidth(0);
    this->mainFrame->layout()->setContentsMargins(0,0,0,0);
    this->glWidget->layout()->setContentsMargins(0,0,0,0);
    this->layout()->setContentsMargins(0,0,0,0);
    this->xyzLabel->hide();
    this->rpyLabel->hide();

    this->xPosEdit->hide();
    this->yPosEdit->hide();
    this->zPosEdit->hide();

    this->rollEdit->hide();
    this->pitchEdit->hide();
    this->yawEdit->hide();
    //this->fpsEdit->hide();
    //this->trianglesEdit->hide();
  }
  else
  {
    this->mainFrame->layout()->addItem(this->bottomBarLayout);
    this->mainFrame->setLineWidth(1);
    this->mainFrame->layout()->setContentsMargins(4,4,4,4);
    this->xyzLabel->show();
    this->rpyLabel->show();

    this->xPosEdit->show();
    this->yPosEdit->show();
    this->zPosEdit->show();

    this->rollEdit->show();
    this->pitchEdit->show();
    this->yawEdit->show();
    //this->fpsEdit->show();
    //this->trianglesEdit->show();
  }
}

void RenderWidget::update()
{
  rendering::UserCameraPtr cam = this->glWidget->GetCamera();

  if (!cam)
    return;

  //float fps = cam->GetAvgFPS();
  //int triangleCount = cam->GetTriangleCount();
  math::Pose pose = cam->GetWorldPose();

  std::ostringstream stream;

  stream << std::fixed << std::setprecision(2) << pose.pos.x;
  this->xPosEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  stream << std::fixed << std::setprecision(2) << pose.pos.y;
  this->yPosEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  stream << std::fixed << std::setprecision(2) << pose.pos.z;
  this->zPosEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  stream << std::fixed << std::setprecision(2) << RTOD(pose.rot.GetAsEuler().x);
  this->rollEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  stream << std::fixed << std::setprecision(2) << RTOD(pose.rot.GetAsEuler().y);
  this->pitchEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  stream << std::fixed << std::setprecision(2) << RTOD(pose.rot.GetAsEuler().z);
  this->yawEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  /*stream << std::fixed << std::setprecision(1) << fps;
  this->fpsEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  stream << std::fixed << std::setprecision(2) << triangleCount;
  this->trianglesEdit->setText( tr(stream.str().c_str()) );
  */

  this->glWidget->update();
}
