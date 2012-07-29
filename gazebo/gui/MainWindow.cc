/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Events.hh"

#include "transport/Node.hh"
#include "transport/Transport.hh"

#include "rendering/UserCamera.hh"

#include "gui/Gui.hh"
#include "gui/InsertModelWidget.hh"
#include "gui/SkyWidget.hh"
#include "gui/ModelListWidget.hh"
#include "gui/LightListWidget.hh"
#include "gui/WorldPropertiesWidget.hh"
#include "gui/TimePanel.hh"
#include "gui/RenderWidget.hh"
#include "gui/GLWidget.hh"
#include "gui/MainWindow.hh"
#include "gui/GuiEvents.hh"

using namespace gazebo;
using namespace gui;

extern bool g_fullscreen;

/////////////////////////////////////////////////
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
    this->node->Advertise<msgs::Selection>("~/selection", 1);

  this->newEntitySub = this->node->Subscribe("~/model/info",
      &MainWindow::OnModel, this);
  this->statsSub =
    this->node->Subscribe("~/world_stats", &MainWindow::OnStats, this);

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response",
      &MainWindow::OnResponse, this, true);

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
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setDockOptions(QMainWindow::AnimatedDocks);

  ModelListWidget *modelListWidget = new ModelListWidget(this);
  LightListWidget *lightListWidget = new LightListWidget(this);
  InsertModelWidget *insertModel = new InsertModelWidget(this);
  SkyWidget *skyWidget = new SkyWidget(this);


  this->treeWidget = new QTreeWidget();
  this->treeWidget->setColumnCount(1);
  this->treeWidget->setIndentation(0);
  this->treeWidget->setRootIsDecorated(true);
  this->treeWidget->setExpandsOnDoubleClick(true);
  this->treeWidget->setFocusPolicy(Qt::NoFocus);
  this->treeWidget->setAnimated(true);
  this->treeWidget->setObjectName("mainTree");

  this->treeWidget->header()->hide();
  this->treeWidget->header()->setResizeMode(QHeaderView::Stretch);
  this->treeWidget->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  this->treeWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  this->treeWidget->setItemDelegate(
      new TreeViewDelegate(this->treeWidget, this->treeWidget));

  connect(this->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
          this, SLOT(ItemSelected(QTreeWidgetItem *, int)));

  QTreeWidgetItem *topItem = new QTreeWidgetItem(this->treeWidget,
      QStringList("Models"));
  this->treeWidget->addTopLevelItem(topItem);
  QTreeWidgetItem *subItem = new QTreeWidgetItem(topItem);
  this->treeWidget->setItemWidget(subItem, 0, modelListWidget);

  topItem = new QTreeWidgetItem(this->treeWidget, QStringList("Lights"));
  this->treeWidget->addTopLevelItem(topItem);
  subItem = new QTreeWidgetItem(topItem);
  this->treeWidget->setItemWidget(subItem, 0, lightListWidget);

  topItem = new QTreeWidgetItem(this->treeWidget, QStringList("InsertModel"));
  this->treeWidget->addTopLevelItem(topItem);
  subItem = new QTreeWidgetItem(topItem);
  this->treeWidget->setItemWidget(subItem, 0, insertModel);

  topItem = new QTreeWidgetItem(this->treeWidget, QStringList("Sky"));
  this->treeWidget->addTopLevelItem(topItem);
  subItem = new QTreeWidgetItem(topItem);
  this->treeWidget->setItemWidget(subItem, 0, skyWidget);

  this->renderWidget = new RenderWidget(mainWidget);
  this->timePanel = new TimePanel(mainWidget);

  QHBoxLayout *centerLayout = new QHBoxLayout;

  this->collapseButton = new QPushButton("<");
  this->collapseButton->setObjectName("collapseButton");
  this->collapseButton->setSizePolicy(QSizePolicy::Fixed,
                                      QSizePolicy::Expanding);
  this->collapseButton->setFocusPolicy(Qt::NoFocus);
  connect(this->collapseButton, SIGNAL(clicked()), this, SLOT(OnCollapse()));

  centerLayout->addSpacing(10);
  centerLayout->addWidget(this->treeWidget, 0);
  centerLayout->addWidget(collapseButton, 0);
  centerLayout->addWidget(this->renderWidget, 1);
  centerLayout->setContentsMargins(0, 0, 0, 0);
  centerLayout->addSpacing(10);
  centerLayout->setSpacing(0);

  QHBoxLayout *timePanelLayout = new QHBoxLayout;
  timePanelLayout->addSpacing(5);
  timePanelLayout->addWidget(this->timePanel);
  timePanelLayout->addSpacing(5);

  mainLayout->addLayout(centerLayout);
  mainLayout->addLayout(timePanelLayout);
  mainWidget->setLayout(mainLayout);

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));

  std::string title = "Gazebo : ";
  title += gui::get_world();
  this->setWindowIconText(tr(title.c_str()));
  this->setWindowTitle(tr(title.c_str()));

  this->worldPropertiesWidget = NULL;

  this->connections.push_back(
      gui::Events::ConnectFullScreen(
        boost::bind(&MainWindow::OnFullScreen, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectMoveMode(
        boost::bind(&MainWindow::OnMoveMode, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectManipMode(
        boost::bind(&MainWindow::OnManipMode, this, _1)));
}

/////////////////////////////////////////////////
MainWindow::~MainWindow()
{
}

/////////////////////////////////////////////////
void MainWindow::Load()
{
  this->guiSub = this->node->Subscribe("~/gui", &MainWindow::OnGUI, this);
}

/////////////////////////////////////////////////
void MainWindow::Init()
{
  this->renderWidget->show();
  this->requestMsg = msgs::CreateRequest("entity_list");
  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void MainWindow::closeEvent(QCloseEvent * /*_event*/)
{
  delete this->worldPropertiesWidget;
  delete this->renderWidget;
  delete this->timePanel;
}

/////////////////////////////////////////////////
void MainWindow::New()
{
  msgs::ServerControl msg;
  msg.set_new_world(true);
  this->serverControlPub->Publish(msg);
}

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
void MainWindow::Import()
{
  std::string filename = QFileDialog::getOpenFileName(this,
      tr("Import Collada Mesh"), "",
      tr("SDF Files (*.dae *.zip)")).toStdString();

  if (!filename.empty())
  {
    if (filename.find(".dae") != std::string::npos)
    {
      gui::Events::createEntity("mesh", filename);
    }
    else
      gzerr << "Unable to import mesh[" << filename << "]\n";
  }
}

/////////////////////////////////////////////////
void MainWindow::Save()
{
  msgs::ServerControl msg;
  msg.set_save_world_name(get_world());
  this->serverControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::SaveAs()
{
  std::string filename = QFileDialog::getSaveFileName(this,
      tr("Save World"), QString(),
      tr("SDF Files (*.xml *.sdf *.world)")).toStdString();

  if (!filename.empty())
  {
    msgs::ServerControl msg;
    msg.set_save_world_name(get_world());
    msg.set_save_filename(filename);
    this->serverControlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void MainWindow::About()
{
  std::string helpTxt = "Gazebo is a 3D multi-robot simulator with dynamics. ";
  helpTxt += "It is capable of simulating articulated robot in complex and ";
  helpTxt += "realistic environments.\n Visit http://www.gazebosim.org for ";
  helpTxt += "more information.";
  QMessageBox::about(this, tr("About Gazebo"), tr(helpTxt.c_str()));
}

/////////////////////////////////////////////////
void MainWindow::Play()
{
  msgs::WorldControl msg;
  msg.set_pause(false);

  this->pauseAct->setChecked(false);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Pause()
{
  msgs::WorldControl msg;
  msg.set_pause(true);

  this->playAct->setChecked(false);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Step()
{
  msgs::WorldControl msg;
  msg.set_step(true);

  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::NewModel()
{
  /*ModelBuilderWidget *modelBuilder = new ModelBuilderWidget();
  modelBuilder->Init();
  modelBuilder->show();
  modelBuilder->resize(800, 600);
  */
}

/////////////////////////////////////////////////
void MainWindow::OnResetModelOnly()
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(false);
  msg.mutable_reset()->set_time_only(false);
  msg.mutable_reset()->set_model_only(true);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::OnResetWorld()
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::EditWorldProperties()
{
  if (!this->worldPropertiesWidget)
    this->worldPropertiesWidget = new WorldPropertiesWidget();

  this->worldPropertiesWidget->show();
}

/////////////////////////////////////////////////
void MainWindow::Arrow()
{
  gui::Events::manipMode("normal");
}

/////////////////////////////////////////////////
void MainWindow::RingPose()
{
  gui::Events::manipMode("ring");
}

/////////////////////////////////////////////////
void MainWindow::CreateBox()
{
  this->arrowAct->setChecked(true);
  gui::Events::createEntity("box", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateSphere()
{
  this->arrowAct->setChecked(true);
  gui::Events::createEntity("sphere", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateCylinder()
{
  this->arrowAct->setChecked(true);
  gui::Events::createEntity("cylinder", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateMesh()
{
  this->arrowAct->setChecked(true);
  gui::Events::createEntity("mesh", "mesh");
}

/////////////////////////////////////////////////
void MainWindow::CreatePointLight()
{
  this->arrowAct->setChecked(true);
  gui::Events::createEntity("pointlight", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateSpotLight()
{
  this->arrowAct->setChecked(true);
  gui::Events::createEntity("spotlight", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateDirectionalLight()
{
  this->arrowAct->setChecked(true);
  gui::Events::createEntity("directionallight", "");
}

/////////////////////////////////////////////////
void MainWindow::InsertModel()
{
}

/////////////////////////////////////////////////
void MainWindow::OnFullScreen(bool _value)
{
  if (_value)
  {
    this->centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);
    this->showFullScreen();
    /*this->removeDockWidget(this->modelsDock);
    this->removeDockWidget(this->lightsDock);
    this->removeDockWidget(this->insertModelsDock);
    */
    this->playToolbar->hide();
    this->editToolbar->hide();
    this->mouseToolbar->hide();
    this->menuBar()->hide();
  }
  else
  {
    this->centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);
    this->showNormal();
    /*this->addDockWidget(Qt::LeftDockWidgetArea, this->modelsDock);
    this->addDockWidget(Qt::LeftDockWidgetArea, this->lightsDock);
    this->addDockWidget(Qt::LeftDockWidgetArea, this->insertModelsDock);
    this->modelsDock->show();
    this->lightsDock->show();
    this->insertModelsDock->show();
    */
    this->playToolbar->show();
    this->editToolbar->show();
    this->mouseToolbar->show();
    this->menuBar()->show();
  }
}

/////////////////////////////////////////////////
void MainWindow::ViewReset()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->SetWorldPose(math::Pose(-5, 0, 1, 0, GZ_DTOR(11.31), 0));
}

/////////////////////////////////////////////////
void MainWindow::ViewFullScreen()
{
  g_fullscreen = !g_fullscreen;
  gui::Events::fullScreen(g_fullscreen);
}

/////////////////////////////////////////////////
void MainWindow::ViewFPS()
{
  gui::Events::fps();
}

/////////////////////////////////////////////////
void MainWindow::ViewOrbit()
{
  gui::Events::orbit();
}

/////////////////////////////////////////////////
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

  this->importAct = new QAction(tr("&Import Mesh"), this);
  this->importAct->setShortcut(tr("Ctrl+I"));
  this->importAct->setStatusTip(tr("Import a Collada mesh"));
  connect(this->importAct, SIGNAL(triggered()), this, SLOT(Import()));


  this->saveAct = new QAction(tr("&Save"), this);
  this->saveAct->setShortcut(tr("Ctrl+S"));
  this->saveAct->setStatusTip(tr("Save world"));
  connect(this->saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  this->saveAsAct = new QAction(tr("Save &As"), this);
  this->saveAsAct->setShortcut(tr("Ctrl+Shift+S"));
  this->saveAsAct->setStatusTip(tr("Save world to new file"));
  connect(this->saveAsAct, SIGNAL(triggered()), this, SLOT(SaveAs()));


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

  this->resetModelsAct = new QAction(tr("&Reset Model Poses"), this);
  this->resetModelsAct->setShortcut(tr("Ctrl+R"));
  this->resetModelsAct->setStatusTip(tr("Reset model poses"));
  connect(this->resetModelsAct, SIGNAL(triggered()), this,
    SLOT(OnResetModelOnly()));

  this->resetWorldAct = new QAction(tr("&Reset World"), this);
  this->resetWorldAct->setShortcut(tr("Ctrl+R"));
  this->resetWorldAct->setStatusTip(tr("Reset the world"));
  connect(this->resetWorldAct, SIGNAL(triggered()), this, SLOT(OnResetWorld()));


  this->editWorldPropertiesAct = new QAction(tr("&World"), this);
  this->editWorldPropertiesAct->setShortcut(tr("Ctrl+W"));
  this->editWorldPropertiesAct->setStatusTip(tr("Edit World Properties"));
  connect(this->editWorldPropertiesAct, SIGNAL(triggered()), this,
          SLOT(EditWorldProperties()));


  this->playAct = new QAction(QIcon(":/images/play.png"), tr("Play"), this);
  this->playAct->setStatusTip(tr("Run the world"));
  this->playAct->setCheckable(true);
  this->playAct->setChecked(true);
  connect(this->playAct, SIGNAL(triggered()), this, SLOT(Play()));

  this->pauseAct = new QAction(QIcon(":/images/pause.png"), tr("Pause"), this);
  this->pauseAct->setStatusTip(tr("Pause the world"));
  this->pauseAct->setCheckable(true);
  this->pauseAct->setChecked(false);
  connect(this->pauseAct, SIGNAL(triggered()), this, SLOT(Pause()));

  this->stepAct = new QAction(QIcon(":/images/end.png"), tr("Step"), this);
  this->stepAct->setStatusTip(tr("Step the world"));
  connect(this->stepAct, SIGNAL(triggered()), this, SLOT(Step()));


  this->arrowAct = new QAction(QIcon(":/images/arrow.png"),
      tr("Position object"), this);
  this->arrowAct->setStatusTip(tr("Move camera"));
  this->arrowAct->setCheckable(true);
  this->arrowAct->setChecked(true);
  connect(this->arrowAct, SIGNAL(triggered()), this, SLOT(Arrow()));

  this->ringPoseAct = new QAction(QIcon(":/images/translate.png"),
      tr("Position object"), this);
  this->ringPoseAct->setStatusTip(tr("Position object"));
  this->ringPoseAct->setCheckable(true);
  this->ringPoseAct->setChecked(false);
  connect(this->ringPoseAct, SIGNAL(triggered()), this, SLOT(RingPose()));


  this->boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  this->boxCreateAct->setStatusTip(tr("Create a box"));
  this->boxCreateAct->setCheckable(true);
  connect(this->boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));

  this->sphereCreateAct = new QAction(QIcon(":/images/sphere.png"),
      tr("Sphere"), this);
  this->sphereCreateAct->setStatusTip(tr("Create a sphere"));
  this->sphereCreateAct->setCheckable(true);
  connect(this->sphereCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSphere()));

  this->cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Cylinder"), this);
  this->cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  this->cylinderCreateAct->setCheckable(true);
  connect(this->cylinderCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateCylinder()));

  this->meshCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Mesh"), this);
  this->meshCreateAct->setStatusTip(tr("Create a mesh"));
  this->meshCreateAct->setCheckable(true);
  connect(this->meshCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateMesh()));


  this->pointLghtCreateAct = new QAction(QIcon(":/images/pointlight.png"),
      tr("Point Light"), this);
  this->pointLghtCreateAct->setStatusTip(tr("Create a point light"));
  this->pointLghtCreateAct->setCheckable(true);
  connect(this->pointLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreatePointLight()));

  this->spotLghtCreateAct = new QAction(QIcon(":/images/spotlight.png"),
      tr("Spot Light"), this);
  this->spotLghtCreateAct->setStatusTip(tr("Create a spot light"));
  this->spotLghtCreateAct->setCheckable(true);
  connect(this->spotLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSpotLight()));

  this->dirLghtCreateAct = new QAction(QIcon(":/images/directionallight.png"),
      tr("Directional Light"), this);
  this->dirLghtCreateAct->setStatusTip(tr("Create a directional light"));
  this->dirLghtCreateAct->setCheckable(true);
  connect(this->dirLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateDirectionalLight()));

  this->viewResetAct = new QAction(tr("Reset View"), this);
  this->viewResetAct->setStatusTip(tr("Move camera to origin"));
  connect(this->viewResetAct, SIGNAL(triggered()), this,
      SLOT(ViewReset()));

  this->viewFullScreenAct = new QAction(tr("Full Screen"), this);
  this->viewFullScreenAct->setStatusTip(tr("View Full Screen(F-11 to exit)"));
  connect(this->viewFullScreenAct, SIGNAL(triggered()), this,
      SLOT(ViewFullScreen()));

  this->viewFPSAct = new QAction(tr("FPS View Control"), this);
  this->viewFPSAct->setStatusTip(tr("First Person Shooter View Style"));
  connect(this->viewFPSAct, SIGNAL(triggered()), this, SLOT(ViewFPS()));

  this->viewOrbitAct = new QAction(tr("Orbit View Control"), this);
  this->viewOrbitAct->setStatusTip(tr("Orbit View Style"));
  connect(this->viewOrbitAct, SIGNAL(triggered()), this, SLOT(ViewOrbit()));
}

/////////////////////////////////////////////////
void MainWindow::CreateMenus()
{
  QHBoxLayout *menuLayout = new QHBoxLayout;

  QFrame *frame = new QFrame;
  QMenuBar *mb = new QMenuBar;
  mb->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  menuLayout->addWidget(mb);
  menuLayout->addStretch(5);
  menuLayout->setContentsMargins(0, 0, 0, 0);

  frame->setLayout(menuLayout);
  frame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

  this->setMenuWidget(frame);

  this->fileMenu = mb->addMenu(tr("&File"));
  this->fileMenu->addAction(this->openAct);
  this->fileMenu->addAction(this->importAct);
  this->fileMenu->addAction(this->newAct);
  this->fileMenu->addAction(this->saveAct);
  this->fileMenu->addAction(this->saveAsAct);
  this->fileMenu->addSeparator();
  this->fileMenu->addAction(this->quitAct);

  this->editMenu = mb->addMenu(tr("&Edit"));
  this->editMenu->addAction(this->resetWorldAct);
  this->editMenu->addAction(this->editWorldPropertiesAct);

  this->viewMenu = mb->addMenu(tr("&View"));
  this->viewMenu->addAction(this->viewResetAct);
  this->viewMenu->addAction(this->viewFullScreenAct);
  this->viewMenu->addAction(this->viewFPSAct);
  this->viewMenu->addAction(this->viewOrbitAct);

  mb->addSeparator();

  this->helpMenu = mb->addMenu(tr("&Help"));
  this->helpMenu->addAction(this->aboutAct);
}

/////////////////////////////////////////////////
void MainWindow::CreateToolbars()
{
  this->playToolbar = this->addToolBar(tr("Play"));
  this->playToolbar->addAction(this->playAct);
  this->playToolbar->addAction(this->pauseAct);
  this->playToolbar->addAction(this->stepAct);

  QActionGroup *actionGroup = new QActionGroup(this);
  this->mouseToolbar = this->addToolBar(tr("Mouse"));
  actionGroup->addAction(this->arrowAct);
  actionGroup->addAction(this->ringPoseAct);
  this->mouseToolbar->addAction(this->arrowAct);
  this->mouseToolbar->addAction(this->ringPoseAct);

  this->editToolbar = this->addToolBar(tr("Edit"));
  this->editToolbar->addAction(this->boxCreateAct);
  this->editToolbar->addAction(this->sphereCreateAct);
  this->editToolbar->addAction(this->cylinderCreateAct);
  // this->editToolbar->addAction(this->meshCreateAct);
  this->editToolbar->addSeparator();
  this->editToolbar->addAction(this->pointLghtCreateAct);
  this->editToolbar->addAction(this->spotLghtCreateAct);
  this->editToolbar->addAction(this->dirLghtCreateAct);
}

/////////////////////////////////////////////////
void MainWindow::OnMoveMode(bool _mode)
{
  if (_mode)
  {
    this->boxCreateAct->setChecked(false);
    this->sphereCreateAct->setChecked(false);
    this->cylinderCreateAct->setChecked(false);
    this->meshCreateAct->setChecked(false);
    this->pointLghtCreateAct->setChecked(false);
    this->spotLghtCreateAct->setChecked(false);
    this->dirLghtCreateAct->setChecked(false);
  }
}

/////////////////////////////////////////////////
void MainWindow::OnGUI(ConstGUIPtr &_msg)
{
  if (_msg->has_fullscreen() && _msg->fullscreen())
  {
    ViewFullScreen();
  }

  if (_msg->has_camera())
  {
    rendering::UserCameraPtr cam = gui::get_active_camera();

    if (_msg->camera().has_origin())
    {
      const msgs::Pose &msg_origin = _msg->camera().origin();

      math::Vector3 cam_origin_pos = math::Vector3(
        msg_origin.position().x(),
        msg_origin.position().y(),
        msg_origin.position().z());

      math::Quaternion cam_origin_rot = math::Quaternion(
        msg_origin.orientation().w(),
        msg_origin.orientation().x(),
        msg_origin.orientation().y(),
        msg_origin.orientation().z());

      math::Pose cam_origin(cam_origin_pos, cam_origin_rot);

      cam->SetWorldPose(cam_origin);
    }

    if (_msg->camera().has_track())
    {
      std::string name = _msg->camera().track().name();

      double minDist = 0.0;
      double maxDist = 0.0;

      if (_msg->camera().track().has_min_dist())
        minDist = _msg->camera().track().min_dist();
      if (_msg->camera().track().has_max_dist())
        maxDist = _msg->camera().track().max_dist();

      cam->AttachToVisual(name, false, minDist, maxDist);
    }
  }
}

/////////////////////////////////////////////////
void MainWindow::OnModel(ConstModelPtr &_msg)
{
  this->entities[_msg->name()] = _msg->id();
  for (int i = 0; i < _msg->link_size(); i++)
  {
    this->entities[_msg->link(i).name()] = _msg->link(i).id();

    for (int j = 0; j < _msg->link(i).collision_size(); j++)
    {
      this->entities[_msg->link(i).collision(j).name()] =
        _msg->link(i).collision(j).id();
    }
  }

  gui::Events::modelUpdate(*_msg);
}

/////////////////////////////////////////////////
void MainWindow::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  msgs::Model_V modelVMsg;

  if (_msg->has_type() && _msg->type() == modelVMsg.GetTypeName())
  {
    modelVMsg.ParseFromString(_msg->serialized_data());

    for (int i = 0; i < modelVMsg.models_size(); i++)
    {
      this->entities[modelVMsg.models(i).name()] = modelVMsg.models(i).id();

      for (int j = 0; j < modelVMsg.models(i).link_size(); j++)
      {
        this->entities[modelVMsg.models(i).link(j).name()] =
          modelVMsg.models(i).link(j).id();

        for (int k = 0; k < modelVMsg.models(i).link(j).collision_size(); k++)
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

/////////////////////////////////////////////////
unsigned int MainWindow::GetEntityId(const std::string &_name)
{
  unsigned int result = 0;

  std::string name = _name;
  boost::replace_first(name, gui::get_world()+"::", "");

  std::map<std::string, unsigned int>::iterator iter;
  iter = this->entities.find(name);
  if (iter != this->entities.end())
    result = iter->second;
  else
    gzerr << "Unable to find model[" << _name << "]\n";

  return result;
}

/////////////////////////////////////////////////
bool MainWindow::HasEntityName(const std::string &_name)
{
  bool result = false;

  std::string name = _name;
  boost::replace_first(name, gui::get_world()+"::", "");

  std::map<std::string, unsigned int>::iterator iter;
  iter = this->entities.find(name);

  if (iter != this->entities.end())
    result = true;

  return result;
}

/////////////////////////////////////////////////
void MainWindow::OnWorldModify(ConstWorldModifyPtr &_msg)
{
  if (_msg->has_create() && _msg->create())
  {
    this->renderWidget->CreateScene(_msg->world_name());
    this->requestMsg = msgs::CreateRequest("entity_list");
    this->requestPub->Publish(*this->requestMsg);
  }
  else if (_msg->has_remove() && _msg->remove())
    this->renderWidget->RemoveScene(_msg->world_name());
}

/////////////////////////////////////////////////
void MainWindow::OnManipMode(const std::string &_mode)
{
  if (_mode != "ring")
    this->arrowAct->setChecked(true);
  else if (_mode == "ring")
    this->ringPoseAct->setChecked(true);
}

/////////////////////////////////////////////////
void MainWindow::OnStats(ConstWorldStatisticsPtr &_msg)
{
  if (_msg->paused() && this->playAct->isChecked())
  {
    this->playAct->setChecked(false);
    this->pauseAct->setChecked(true);
  }
  else if (!_msg->paused() && !this->playAct->isChecked())
  {
    this->playAct->setChecked(true);
    this->pauseAct->setChecked(false);
  }
}

/////////////////////////////////////////////////
void MainWindow::ItemSelected(QTreeWidgetItem *_item, int)
{
  _item->setExpanded(!_item->isExpanded());
}

/////////////////////////////////////////////////
void MainWindow::OnCollapse()
{
  if (this->treeWidget->isVisible())
  {
    this->treeWidget->close();
    this->collapseButton->setText(">");
  }
  else
  {
    this->treeWidget->show();
    this->collapseButton->setText("<");
  }
}

/////////////////////////////////////////////////
TreeViewDelegate::TreeViewDelegate(QTreeView *_view, QWidget *_parent)
  : QItemDelegate(_parent), view(_view)
{
}

/////////////////////////////////////////////////
void TreeViewDelegate::paint(QPainter *painter,
                          const QStyleOptionViewItem &option,
                          const QModelIndex &index) const
{
  const QAbstractItemModel *model = index.model();
  Q_ASSERT(model);

  if (!model->parent(index).isValid())
  {
    QRect r = option.rect;

    QColor orange(245, 129, 19);
    QColor blue(71, 99, 183);
    QColor grey(100, 100, 100);

    if (option.state & QStyle::State_Open ||
        option.state & QStyle::State_MouseOver)
    {
      painter->setPen(blue);
      painter->setBrush(QBrush(blue));
    }
    else
    {
      painter->setPen(grey);
      painter->setBrush(QBrush(grey));
    }

    if (option.state & QStyle::State_Open)
      painter->drawLine(r.left()+8, r.top() + (r.height()*0.5 - 5),
                        r.left()+8, r.top() + r.height()-1);

    painter->save();
    painter->setRenderHints(QPainter::Antialiasing |
                            QPainter::TextAntialiasing);

    painter->drawRoundedRect(r.left()+4, r.top() + (r.height()*0.5 - 5),
                             10, 10, 20.0, 10.0, Qt::RelativeSize);


    // draw text
    QRect textrect = QRect(r.left() + 20, r.top(),
                           r.width() - 40,
                           r.height());

    QString text = elidedText(
        option.fontMetrics,
        textrect.width(),
        Qt::ElideMiddle,
        model->data(index, Qt::DisplayRole).toString());

    if (option.state & QStyle::State_MouseOver)
      painter->setPen(QPen(orange, 1));
    else
      painter->setPen(QPen(grey, 1));

    this->view->style()->drawItemText(painter, textrect, Qt::AlignLeft,
        option.palette, this->view->isEnabled(), text);

    painter->restore();
  }
  else
  {
      QItemDelegate::paint(painter, option, index);
  }
}

/////////////////////////////////////////////////
QSize TreeViewDelegate::sizeHint(const QStyleOptionViewItem &_opt,
    const QModelIndex &_index) const
{
  QStyleOptionViewItem option = _opt;
  QSize sz = QItemDelegate::sizeHint(_opt, _index) + QSize(2, 2);
  return sz;
}

