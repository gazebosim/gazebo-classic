#include <QtGui>

#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Events.hh"

#include "transport/Node.hh"
#include "transport/Transport.hh"

#include "rendering/UserCamera.hh"

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
  this->requestMsg = NULL;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  gui::set_world(this->node->GetTopicNamespace());
  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");
  this->serverControlPub =
    this->node->Advertise<msgs::ServerControl>("/gazebo/server/control");
  this->selectionPub =
    this->node->Advertise<msgs::Selection>("~/selection",1);

  this->newEntitySub = this->node->Subscribe("~/model/info", 
      &MainWindow::OnModel, this);

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response", 
      &MainWindow::OnResponse, this);

  this->worldModSub = this->node->Subscribe("/gazebo/world/modify",
                                            &MainWindow::OnWorldModify, this);

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
  ModelListWidget *modelListWidget = new ModelListWidget(this);
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
      gui::Events::ConnectFullScreen( 
        boost::bind(&MainWindow::OnFullScreen, this, _1) ) );

  this->connections.push_back( 
      gui::Events::ConnectMoveMode( 
        boost::bind(&MainWindow::OnMoveMode, this, _1) ) );

  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&MainWindow::OnSetSelectedEntity, this, _1)));

  this->requestMsg = msgs::CreateRequest("entity_list");
  this->requestPub->Publish(*this->requestMsg);
}

MainWindow::~MainWindow()
{
}

void MainWindow::Load()
{
  this->guiSub = this->node->Subscribe("~/gui", &MainWindow::OnGUI, this);
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
  std::string filename = QFileDialog::getOpenFileName(this,
      tr("Open World"), "",
      tr("SDF Files (*.xml *.sdf *.world)")).toStdString();

  if (!filename.empty())
  {
    msgs::ServerControl msg;
    msg.set_open_filename(filename);
    this->serverControlPub->Publish(msg);
  }
}

void MainWindow::Save()
{
  std::string filename = QFileDialog::getSaveFileName(this,
      tr("Save World"), "/home/nkoenig",
      tr("SDF Files (*.xml *.sdf *.world)")).toStdString();

  msgs::ServerControl msg;
  msg.set_save_world_name("default");
  msg.set_save_filename(filename);
  this->serverControlPub->Publish(msg);
}

void MainWindow::About()
{
  QMessageBox::about(this, tr("About Gazebo"),
      tr("The <b>Gazebo</b> is awesome."));
}

void MainWindow::Play()
{
  msgs::WorldControl msg;
  msg.set_pause(false);

  this->pauseAct->setChecked(false);
  this->worldControlPub->Publish(msg);
}

void MainWindow::Pause()
{
  msgs::WorldControl msg;
  msg.set_pause(true);

  this->playAct->setChecked(false);
  this->worldControlPub->Publish(msg);
}

void MainWindow::Step()
{
  msgs::WorldControl msg;
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

void MainWindow::OnResetWorld()
{
  msgs::WorldControl msg;
  msg.set_reset_world(true);
  this->worldControlPub->Publish(msg);
}

void MainWindow::EditWorldProperties()
{
  if (!this->worldPropertiesWidget)
    this->worldPropertiesWidget = new WorldPropertiesWidget();

  this->worldPropertiesWidget->show();
}

void MainWindow::CreateBox()
{
  gui::Events::createEntity("box");
}

void MainWindow::CreateSphere()
{
  gui::Events::createEntity("sphere");
}

void MainWindow::CreateCylinder()
{
  gui::Events::createEntity("cylinder");
}

void MainWindow::CreatePointLight()
{
  gui::Events::createEntity("pointlight");
}

void MainWindow::CreateSpotLight()
{
  gui::Events::createEntity("spotlight");
}

void MainWindow::CreateDirectionalLight()
{
  gui::Events::createEntity("directionallight");
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
  gui::Events::fullScreen(g_fullscreen);
}

void MainWindow::ViewFPS()
{
  gui::Events::fps();
}

void MainWindow::ViewOrbit()
{
  gui::Events::orbit();
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

  this->resetWorldAct = new QAction(tr("&Reset World"), this);
  this->resetWorldAct->setShortcut(tr("Ctrl+R"));
  this->resetWorldAct->setStatusTip(tr("Reset the world"));
  connect(this->resetWorldAct, SIGNAL(triggered()), this, SLOT(OnResetWorld()));


  this->editWorldPropertiesAct = new QAction(tr("&World"), this);
  this->editWorldPropertiesAct->setShortcut(tr("Ctrl+W"));
  this->editWorldPropertiesAct->setStatusTip(tr("Edit World Properties"));
  connect(this->editWorldPropertiesAct, SIGNAL(triggered()), this, SLOT(EditWorldProperties()));


  this->playAct = new QAction(QIcon(":/images/play.png"), tr("Play"), this);
  this->playAct->setStatusTip(tr("Run the world"));
  this->playAct->setCheckable(true);
  connect(this->playAct, SIGNAL(triggered()), this, SLOT(Play()));

  this->pauseAct = new QAction(QIcon(":/images/pause.png"), tr("Pause"), this);
  this->pauseAct->setStatusTip(tr("Pause the world"));
  this->pauseAct->setCheckable(true);
  connect(this->pauseAct, SIGNAL(triggered()), this, SLOT(Pause()));

  this->stepAct = new QAction(QIcon(":/images/end.png"), tr("Step"), this);
  this->stepAct->setStatusTip(tr("Step the world"));
  connect(this->stepAct, SIGNAL(triggered()), this, SLOT(Step()));

  this->boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  this->boxCreateAct->setStatusTip(tr("Create a box"));
  this->boxCreateAct->setCheckable(true);
  connect(this->boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));

  this->sphereCreateAct = new QAction(QIcon(":/images/sphere.png"), tr("Sphere"), this);
  this->sphereCreateAct->setStatusTip(tr("Create a sphere"));
  this->sphereCreateAct->setCheckable(true);
  connect(this->sphereCreateAct, SIGNAL(triggered()), this, SLOT(CreateSphere()));

  this->cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"), tr("Cylinder"), this);
  this->cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  this->cylinderCreateAct->setCheckable(true);
  connect(this->cylinderCreateAct, SIGNAL(triggered()), this, SLOT(CreateCylinder()));

  this->pointLghtCreateAct = new QAction(QIcon(":/images/pointlight.png"), tr("Point Light"), this);
  this->pointLghtCreateAct->setStatusTip(tr("Create a point light"));
  this->pointLghtCreateAct->setCheckable(true);
  connect(this->pointLghtCreateAct, SIGNAL(triggered()), this, SLOT(CreatePointLight()));

  this->spotLghtCreateAct = new QAction(QIcon(":/images/spotlight.png"), tr("Spot Light"), this);
  this->spotLghtCreateAct->setStatusTip(tr("Create a spot light"));
  this->spotLghtCreateAct->setCheckable(true);
  connect(this->spotLghtCreateAct, SIGNAL(triggered()), this, SLOT(CreateSpotLight()));

  this->dirLghtCreateAct = new QAction(QIcon(":/images/directionallight.png"), tr("Directional Light"), this);
  this->dirLghtCreateAct->setStatusTip(tr("Create a directional light"));
  this->dirLghtCreateAct->setCheckable(true);
  connect(this->dirLghtCreateAct, SIGNAL(triggered()), this, SLOT(CreateDirectionalLight()));

  /*this->insertModelAct = new QAction(QIcon(":/images/insertModel.png"), tr("Insert Model"), this);
  this->insertModelAct->setStatusTip(tr("Insert a model"));
  this->rLghtCreateAct->setCheckable(true);
  connect(this->insertModelAct, SIGNAL(triggered()), this, SLOT(InsertModel()));
  */

  this->viewFullScreenAct = new QAction(tr("Full Screen"), this);
  this->viewFullScreenAct->setStatusTip(tr("View Full Screen(F-11 to exit)"));
  connect(this->viewFullScreenAct, SIGNAL(triggered()), this, SLOT(ViewFullScreen()));

  this->viewFPSAct = new QAction(tr("FPS View Control"), this);
  this->viewFPSAct->setStatusTip(tr("First Person Shooter View Style"));
  connect(this->viewFPSAct, SIGNAL(triggered()), this, SLOT(ViewFPS()));

  this->viewOrbitAct = new QAction(tr("Orbit View Control"), this);
  this->viewOrbitAct->setStatusTip(tr("Orbit View Style"));
  connect(this->viewOrbitAct, SIGNAL(triggered()), this, SLOT(ViewOrbit()));
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
  //this->editMenu->addAction(this->newModelAct);
  this->editMenu->addAction(this->resetWorldAct);
  this->editMenu->addAction(this->editWorldPropertiesAct);

  this->viewMenu = this->menuBar()->addMenu(tr("&View"));
  this->viewMenu->addAction(this->viewFullScreenAct);
  this->viewMenu->addAction(this->viewFPSAct);
  this->viewMenu->addAction(this->viewOrbitAct);

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
  //this->editToolbar->addSeparator();
  //this->editToolbar->addAction(this->insertModelAct);
}

void MainWindow::OnMoveMode(bool mode)
{
  if (mode)
  {
    this->boxCreateAct->setChecked(false);
    this->sphereCreateAct->setChecked(false);
    this->cylinderCreateAct->setChecked(false);
    this->pointLghtCreateAct->setChecked(false);
    this->spotLghtCreateAct->setChecked(false);
    this->dirLghtCreateAct->setChecked(false);
  }
}

void MainWindow::OnGUI(ConstGUIPtr &_msg)
{
  if (_msg->has_fullscreen() && _msg->fullscreen())
  {
    ViewFullScreen();
  }

  if (_msg->has_camera())
  {
    rendering::UserCameraPtr cam = gui::get_active_camera();

    if(_msg->camera().has_origin())
    {
      const msgs::Pose &msg_origin = _msg->camera().origin();
      math::Vector3 cam_origin_pos = math::Vector3(\
        msg_origin.position().x(),\
        msg_origin.position().y(),\
        msg_origin.position().z()
      );
      math::Quaternion cam_origin_rot = math::Quaternion(\
        msg_origin.orientation().w(),\
        msg_origin.orientation().x(),\
        msg_origin.orientation().y(),\
        msg_origin.orientation().z()
      );
      math::Pose cam_origin(cam_origin_pos,cam_origin_rot);
      cam->SetWorldPose(cam_origin);
    }

    if (_msg->camera().has_track())
    {
      std::string name = _msg->camera().track().name();

      double minDist, maxDist;
      minDist = maxDist = 0;

      if ( _msg->camera().track().has_min_dist())
        minDist = _msg->camera().track().min_dist();
      if ( _msg->camera().track().has_max_dist())
        maxDist = _msg->camera().track().max_dist();

      std::cout << "HasTrack. Name[" << name << "] min[" << minDist << "] Max[" << maxDist << "]\n";

      cam->AttachToVisual( name, minDist, maxDist );
    }
  }
}

void MainWindow::OnModel(ConstModelPtr &_msg )
{
  this->entities[_msg->name()] = _msg->id();
  for (int i=0; i < _msg->link_size(); i++)
  {
    this->entities[_msg->link(i).name()] = _msg->link(i).id();

    for (int j=0; j < _msg->link(i).collision_size(); j++)
    {
      this->entities[_msg->link(i).collision(j).name()] = 
        _msg->link(i).collision(j).id();
    }
  }

  gui::Events::modelUpdate(*_msg);
}

void MainWindow::OnResponse(
    ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  msgs::Model_V modelVMsg;

  if (_msg->has_type() && _msg->type() == modelVMsg.GetTypeName())
  {
    modelVMsg.ParseFromString(_msg->serialized_data());

    for (int i=0; i < modelVMsg.models_size(); i++)
    {
      this->entities[modelVMsg.models(i).name()] = modelVMsg.models(i).id();

      for (int j=0; j < modelVMsg.models(i).link_size(); j++)
      {
        this->entities[modelVMsg.models(i).link(j).name()] = 
          modelVMsg.models(i).link(j).id();

        for (int k=0; k < modelVMsg.models(i).link(j).collision_size(); k++)
        {
          this->entities[modelVMsg.models(i).link(j).collision(k).name()] = 
            modelVMsg.models(i).link(j).collision(k).id();
        }
      }
      gui::Events::modelUpdate(modelVMsg.models(i));
    }
  }

  delete this->requestMsg;
  this->requestMsg = NULL;
}

void MainWindow::OnSetSelectedEntity(const std::string &_name)
{
  std::map<std::string, unsigned int>::iterator iter;
  std::string name = _name;
  boost::replace_first(name, gui::get_world()+"::","");

  msgs::Selection msg;
  msg.set_name(name);

  iter = this->entities.find(name);
  if (iter != this->entities.end())
    msg.set_id(iter->second);
  else
  {
    gzerr << "Unable to find model[" << _name << "]\n";
  }

  this->selectionPub->Publish(msg);
}

unsigned int MainWindow::GetEntityId(const std::string &_name)
{
  unsigned int result = 0;

  std::string name = _name;
  boost::replace_first(name, gui::get_world()+"::","");

  std::map<std::string, unsigned int>::iterator iter;
  iter = this->entities.find(name);
  if (iter != this->entities.end())
    result = iter->second;
  else
    gzerr << "Unable to find model[" << name << "]\n";

  return result;
}

bool MainWindow::HasEntityName(const std::string &_name)
{
  bool result = false;

  std::map<std::string, unsigned int>::iterator iter;
  iter = this->entities.find(_name);

  if (iter != this->entities.end())
    result = true;

  return result;
}

void MainWindow::OnWorldModify(
    ConstWorldModifyPtr &_msg)
{
  if (_msg->has_create() && _msg->create())
    this->renderWidget->CreateScene(_msg->world_name());
  else if (_msg->has_remove() && _msg->remove())
    this->renderWidget->RemoveScene(_msg->world_name());
  else if (_msg->has_create() && _msg->create())
    this->renderWidget->CreateScene(_msg->world_name());
}
