#include <QtGui>

#include "common/Console.hh"
#include "common/Exception.hh"

#include "transport/Node.hh"
#include "transport/Transport.hh"

#include "gui/Gui.hh"
#include "gui/InsertModelWidget.hh"
#include "gui/ModelListWidget.hh"
#include "gui/WorldPropertiesWidget.hh"
#include "gui/TimePanel.hh"
#include "gui/RenderWidget.hh"
#include "gui/GLWidget.hh"
#include "gui/MainWindow.hh"
#include "gui/GuiEvents.hh"

using namespace gazebo;
using namespace gui;

extern bool g_fullscreen;


MainWindow::MainWindow()
  : renderWidget(0)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  gui::set_world( this->node->GetTopicNamespace() );
  this->worldControlPub = this->node->Advertise<msgs::WorldControl>("~/world_control");

  (void) new QShortcut(Qt::CTRL + Qt::Key_Q, this, SLOT(close()));
  this->CreateActions();
  this->CreateMenus();
  this->CreateToolbars();

  QWidget *mainWidget = new QWidget;
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainWidget->show();
  this->setCentralWidget(mainWidget);

  this->setDockOptions(QMainWindow::ForceTabbedDocks |
//                       QMainWindow::AllowTabbedDocks |
//                       QMainWindow::AnimatedDocks |
                       QMainWindow::VerticalTabs);

  this->modelsDock = new QDockWidget(tr("Models"), this);
  this->modelsDock->setAllowedAreas(Qt::LeftDockWidgetArea);
  ModelListWidget *modelListWidget = new ModelListWidget();
  this->modelsDock->setWidget(modelListWidget);
  this->addDockWidget(Qt::LeftDockWidgetArea, this->modelsDock);

  this->insertModelsDock = new QDockWidget(tr("Insert Model"), this);
  this->insertModelsDock->setAllowedAreas(Qt::LeftDockWidgetArea);
  InsertModelWidget *insertModel = new InsertModelWidget();
  this->insertModelsDock->setWidget(insertModel);
  this->addDockWidget(Qt::LeftDockWidgetArea, this->insertModelsDock);

  this->renderWidget = new RenderWidget(mainWidget);
  this->renderWidget->hide();

  this->timePanel = new TimePanel(mainWidget);

  mainLayout->addWidget( this->renderWidget );
  mainLayout->addWidget( this->timePanel );
  mainWidget->setLayout(mainLayout);

  this->tabifyDockWidget(this->modelsDock, this->insertModelsDock);
  this->modelsDock->raise();

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));

  std::string title = "Gazebo : ";
  title += gui::get_world();
  this->setWindowIconText(tr(title.c_str()));
  this->setWindowTitle(tr(title.c_str()));

  this->worldPropertiesWidget = NULL;

  this->connections.push_back( 
      gui::Events::ConnectFullScreenSignal( 
        boost::bind(&MainWindow::OnFullScreen, this, _1) ) );
}

MainWindow::~MainWindow()
{
}

void MainWindow::Load()
{
}

void MainWindow::Init()
{
  this->renderWidget->show();
}

void MainWindow::closeEvent(QCloseEvent * /*_event*/)
{
  delete this->renderWidget;
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

void MainWindow::EditWorldProperties()
{
  if (!this->worldPropertiesWidget)
    this->worldPropertiesWidget = new WorldPropertiesWidget();

  this->worldPropertiesWidget->show();
}

void MainWindow::CreateBox()
{
  gui::Events::createEntitySignal("box");
}

void MainWindow::CreateSphere()
{
  gui::Events::createEntitySignal("sphere");
}

void MainWindow::CreateCylinder()
{
  gui::Events::createEntitySignal("cylinder");
}

void MainWindow::CreatePointLight()
{
  gui::Events::createEntitySignal("pointlight");
}

void MainWindow::CreateSpotLight()
{
  gui::Events::createEntitySignal("spotlight");
}

void MainWindow::CreateDirectionalLight()
{
  gui::Events::createEntitySignal("directionallight");
}

void MainWindow::InsertModel()
{
}

void MainWindow::OnFullScreen(bool _value)
{
  if (_value)
  {
    this->centralWidget()->layout()->setContentsMargins(0,0,0,0);
    this->showFullScreen();
    this->removeDockWidget(this->modelsDock);
    this->removeDockWidget(this->insertModelsDock);
    this->playToolbar->hide();
    this->editToolbar->hide();
    this->menuBar()->hide();
  }
  else
  {
    this->centralWidget()->layout()->setContentsMargins(4,4,4,4);
    this->showNormal();
    this->addDockWidget(Qt::LeftDockWidgetArea, this->modelsDock);
    this->addDockWidget(Qt::LeftDockWidgetArea, this->insertModelsDock);
    this->modelsDock->show();
    this->insertModelsDock->show();
    this->playToolbar->show();
    this->editToolbar->show();
    this->menuBar()->show();

    this->tabifyDockWidget(this->modelsDock, this->insertModelsDock);
  }
}

void MainWindow::ViewFullScreen()
{
  g_fullscreen = !g_fullscreen;
  gui::Events::fullScreenSignal(g_fullscreen);
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

  this->editWorldPropertiesAct = new QAction(tr("&World"), this);
  this->editWorldPropertiesAct->setShortcut(tr("Ctrl+W"));
  this->editWorldPropertiesAct->setStatusTip(tr("Edit World Properties"));
  connect(this->editWorldPropertiesAct, SIGNAL(triggered()), this, SLOT(EditWorldProperties()));


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

  this->viewFullScreenAct = new QAction(tr("Full Screen"), this);
  this->viewFullScreenAct->setStatusTip(tr("View Full Screen(F-11 to exit)"));
  connect(this->viewFullScreenAct, SIGNAL(triggered()), this, SLOT(ViewFullScreen()));

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
  this->editMenu->addAction(this->editWorldPropertiesAct);

  this->viewMenu = this->menuBar()->addMenu(tr("&View"));
  this->viewMenu->addAction(this->viewFullScreenAct);

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
