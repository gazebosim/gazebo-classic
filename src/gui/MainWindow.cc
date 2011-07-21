#include <QtGui>

#include "common/Console.hh"
#include "common/Exception.hh"

#include "transport/Node.hh"
#include "transport/Transport.hh"

#include "gui/Gui.hh"
#include "gui/InsertModelWidget.hh"
#include "gui/WorldPropertiesWidget.hh"
#include "gui/TimePanel.hh"
#include "gui/RenderWidget.hh"
#include "gui/GLWidget.hh"
#include "gui/MainWindow.hh"

using namespace gazebo;
using namespace gui;


MainWindow::MainWindow()
  : glWidget(0)
{
  (void) new QShortcut(Qt::CTRL + Qt::Key_Q, this, SLOT(close()));
  this->CreateActions();
  this->CreateMenus();
  this->CreateToolbars();

  QSplitter *splitter = new QSplitter;  
  QWidget *mainWidget = new QWidget;
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainWidget->show();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->worldControlPub = this->node->Advertise<msgs::WorldControl>("~/world_control");

  this->glWidget = new RenderWidget(mainWidget);
  this->glWidget->hide();

  this->timePanel = new TimePanel(mainWidget);

  //InsertModelWidget *insertModel = new InsertModelWidget();

  //QFrame *leftFrame = new QFrame;
  //leftFrame->setLineWidth(1);
  //leftFrame->setFrameShadow(QFrame::Sunken);
  //leftFrame->setFrameShape(QFrame::Box);

//  splitter->addWidget(insertModel);
  splitter->addWidget(this->glWidget);
  //mainLayout->addWidget( this->glWidget );
  mainLayout->addWidget(splitter);
  mainLayout->addWidget( this->timePanel );
  mainWidget->setLayout(mainLayout);

  this->setCentralWidget(mainWidget);
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowIconText(tr("Gazebo"));
  this->setWindowTitle(tr("Gazebo"));

  this->worldPropertiesWidget = NULL;
}

MainWindow::~MainWindow()
{
}

void MainWindow::Load()
{
}

void MainWindow::Init()
{
  this->glWidget->show();
}

void MainWindow::closeEvent(QCloseEvent * /*_event*/)
{
  delete this->glWidget;
  delete this->timePanel;
}

void MainWindow::New()
{
  gzdbg << "MainWindow::New world\n";
}

void MainWindow::Open()
{
  gzdbg << "MainWindow::Open file\n";
}

void MainWindow::Save()
{
  gzdbg << "MainWindow::Save to file\n";
}

void MainWindow::About()
{
  QMessageBox::about(this, tr("About Gazebo"),
      tr("The <b>Gazebo</b> is awesome."));
}

void MainWindow::Play()
{
  msgs::WorldControl msg;
  msgs::Init(msg,"world_control");
  msg.set_pause(false);

  this->worldControlPub->Publish(msg);
}

void MainWindow::Pause()
{
  msgs::WorldControl msg;
  msgs::Init(msg,"world_control");
  msg.set_pause(true);

  this->worldControlPub->Publish(msg);
}

void MainWindow::Step()
{
  msgs::WorldControl msg;
  msgs::Init(msg,"world_control");
  msg.set_step(true);

  this->worldControlPub->Publish(msg);
}

void MainWindow::NewModel()
{
  /*ModelBuilderWidget *modelBuilder = new ModelBuilderWidget();
  modelBuilder->Init();
  modelBuilder->show();
  modelBuilder->resize(800,600);
  */
}

void MainWindow::EditScene()
{
  if (!this->worldPropertiesWidget)
    this->worldPropertiesWidget = new WorldPropertiesWidget();

  this->worldPropertiesWidget->show();
}

void MainWindow::CreateBox()
{
}

void MainWindow::CreateSphere()
{
}

void MainWindow::CreateCylinder()
{
}

void MainWindow::CreatePointLight()
{
}

void MainWindow::CreateSpotLight()
{
}

void MainWindow::CreateDirectionalLight()
{
}

void MainWindow::InsertModel()
{
}

void MainWindow::CreateActions()
{
  this->newAct = new QAction(tr("&New"), this);
  this->newAct->setShortcut(tr("Ctrl+N"));
  this->newAct->setStatusTip(tr("Create a new world"));
  connect(this->newAct, SIGNAL(triggered()), this, SLOT(New()));

  this->openAct = new QAction(tr("&Open"), this);
  this->openAct->setShortcut(tr("Ctrl+O"));
  this->openAct->setStatusTip(tr("Open an world file"));
  connect(this->openAct, SIGNAL(triggered()), this, SLOT(Open()));

  this->saveAct = new QAction(tr("&Save"), this);
  this->saveAct->setShortcut(tr("Ctrl+S"));
  this->saveAct->setStatusTip(tr("Save to a world file"));
  connect(this->saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  this->aboutAct = new QAction(tr("&About"), this);
  this->aboutAct->setStatusTip(tr("Show the about info"));
  connect(this->aboutAct, SIGNAL(triggered()), this, SLOT(About()));

  this->quitAct = new QAction(tr("&Quit"), this);
  this->quitAct->setStatusTip(tr("Quit"));
  connect(this->quitAct, SIGNAL(triggered()), this, SLOT(close()));

  this->newModelAct = new QAction(tr("New &Model"), this);
  this->newModelAct->setShortcut(tr("Ctrl+M"));
  this->newModelAct->setStatusTip(tr("Create a new model"));
  connect(this->newModelAct, SIGNAL(triggered()), this, SLOT(NewModel()));

  this->editSceneAct = new QAction(tr("&Scene"), this);
  this->editSceneAct->setShortcut(tr("Ctrl+S"));
  this->editSceneAct->setStatusTip(tr("Edit Scene Properties"));
  connect(this->editSceneAct, SIGNAL(triggered()), this, SLOT(EditScene()));


  this->playAct = new QAction(QIcon(":/images/play.png"), tr("Play"), this);
  this->playAct->setStatusTip(tr("Run the world"));
  connect(this->playAct, SIGNAL(triggered()), this, SLOT(Play()));

  this->pauseAct = new QAction(QIcon(":/images/pause.png"), tr("Pause"), this);
  this->pauseAct->setStatusTip(tr("Pause the world"));
  connect(this->pauseAct, SIGNAL(triggered()), this, SLOT(Pause()));

  this->stepAct = new QAction(QIcon(":/images/end.png"), tr("Step"), this);
  this->stepAct->setStatusTip(tr("Step the world"));
  connect(this->stepAct, SIGNAL(triggered()), this, SLOT(Step()));

  this->boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  this->boxCreateAct->setStatusTip(tr("Create a box"));
  connect(this->boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));

  this->sphereCreateAct = new QAction(QIcon(":/images/sphere.png"), tr("Sphere"), this);
  this->sphereCreateAct->setStatusTip(tr("Create a sphere"));
  connect(this->sphereCreateAct, SIGNAL(triggered()), this, SLOT(CreateSphere()));

  this->cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"), tr("Cylinder"), this);
  this->cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  connect(this->cylinderCreateAct, SIGNAL(triggered()), this, SLOT(CreateCylinder()));

  this->pointLghtCreateAct = new QAction(QIcon(":/images/pointlight.png"), tr("Point Light"), this);
  this->pointLghtCreateAct->setStatusTip(tr("Create a point light"));
  connect(this->pointLghtCreateAct, SIGNAL(triggered()), this, SLOT(CreatePointLight()));

  this->spotLghtCreateAct = new QAction(QIcon(":/images/spotlight.png"), tr("Spot Light"), this);
  this->spotLghtCreateAct->setStatusTip(tr("Create a spot light"));
  connect(this->spotLghtCreateAct, SIGNAL(triggered()), this, SLOT(CreateSpotLight()));

  this->dirLghtCreateAct = new QAction(QIcon(":/images/directionallight.png"), tr("Directional Light"), this);
  this->dirLghtCreateAct->setStatusTip(tr("Create a directional light"));
  connect(this->dirLghtCreateAct, SIGNAL(triggered()), this, SLOT(CreateDirectionalLight()));

  this->insertModelAct = new QAction(QIcon(":/images/insertModel.png"), tr("Insert Model"), this);
  this->insertModelAct->setStatusTip(tr("Insert a model"));
  connect(this->insertModelAct, SIGNAL(triggered()), this, SLOT(InsertModel()));

}

void MainWindow::CreateMenus()
{
  this->fileMenu = this->menuBar()->addMenu(tr("&File"));
  this->fileMenu->addAction(this->openAct);
  this->fileMenu->addAction(this->newAct);
  this->fileMenu->addAction(this->saveAct);
  this->fileMenu->addSeparator();
  this->fileMenu->addAction(this->quitAct);

  this->editMenu = this->menuBar()->addMenu(tr("&Edit"));
  this->editMenu->addAction(this->newModelAct);
  this->editMenu->addAction(this->editSceneAct);

  this->viewMenu = this->menuBar()->addMenu(tr("&View"));

  this->menuBar()->addSeparator();

  this->helpMenu = this->menuBar()->addMenu(tr("&Help"));
  this->helpMenu->addAction(this->aboutAct);
}

void MainWindow::CreateToolbars()
{
  this->playToolbar = this->addToolBar(tr("Play"));
  this->playToolbar->addAction(this->playAct);
  this->playToolbar->addAction(this->pauseAct);
  this->playToolbar->addAction(this->stepAct);

  this->editToolbar = this->addToolBar(tr("Edit"));
  this->editToolbar->addAction(this->boxCreateAct);
  this->editToolbar->addAction(this->sphereCreateAct);
  this->editToolbar->addAction(this->cylinderCreateAct);
  this->editToolbar->addSeparator();
  this->editToolbar->addAction(this->pointLghtCreateAct);
  this->editToolbar->addAction(this->spotLghtCreateAct);
  this->editToolbar->addAction(this->dirLghtCreateAct);
  this->editToolbar->addSeparator();
  this->editToolbar->addAction(this->insertModelAct);
}
