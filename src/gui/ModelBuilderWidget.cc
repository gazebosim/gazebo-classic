#include <QtGui>

#include "rendering/Rendering.hh"
#include "rendering/UserCamera.hh"

#include "physics/World.hh"
#include "physics/Physics.hh"

#include "gui/GLWidget.hh"

#include "gui/ModelBuilderWidget.hh"

using namespace gazebo;
using namespace gui;

ModelBuilderWidget::ModelBuilderWidget( QWidget *parent )
  : QWidget(parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *renderFrame = new QFrame;
  renderFrame->setLineWidth(1);
  renderFrame->setFrameShadow(QFrame::Sunken);
  renderFrame->setFrameShape(QFrame::Box);
  renderFrame->show();

  QVBoxLayout *frameLayout = new QVBoxLayout;

  this->glWidget = new GLWidget(renderFrame);
  rendering::ScenePtr scene = rendering::create_scene("model_builder");
  this->glWidget->ViewScene( scene );
  //this->glWidget->hide();

  frameLayout->addWidget(this->glWidget);
  renderFrame->setLayout(frameLayout);
  renderFrame->layout()->setContentsMargins(4,4,4,4);

  QToolBar *toolbar = new QToolBar(this);

  this->boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  this->boxCreateAct->setStatusTip(tr("Create a box"));
  connect(this->boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));
  toolbar->addAction(boxCreateAct);

  this->sphereCreateAct = new QAction(QIcon(":/images/sphere.png"), tr("Sphere"), this);
  this->sphereCreateAct->setStatusTip(tr("Create a sphere"));
  connect(this->sphereCreateAct, SIGNAL(triggered()), this, SLOT(CreateSphere()));
  toolbar->addAction(sphereCreateAct);

  this->cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"), tr("Cylinder"), this);
  this->cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  connect(this->cylinderCreateAct, SIGNAL(triggered()), this, SLOT(CreateCylinder()));
  toolbar->addAction(cylinderCreateAct);

  mainLayout->addWidget(toolbar);
  mainLayout->addWidget(renderFrame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0,0,0,0);

  physics::init();
  this->world = physics::create_world("model_builder");
  this->world->Load(NULL);
  this->world->Init();
  this->world->SetPaused(true);
}

ModelBuilderWidget::~ModelBuilderWidget()
{
  delete glWidget;
}

void ModelBuilderWidget::Init()
{
  this->glWidget->show();
}

void ModelBuilderWidget::CreateBox()
{
  this->glWidget->CreateEntity("box");
}

void ModelBuilderWidget::CreateSphere()
{
  this->glWidget->CreateEntity("sphere");
}

void ModelBuilderWidget::CreateCylinder()
{
  this->glWidget->CreateEntity("cylinder");
}
