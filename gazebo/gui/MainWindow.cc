/*
 * Copyright 2011 Nate Koenig
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

#include "gazebo.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Events.hh"

#include "transport/Node.hh"
#include "transport/Transport.hh"

#include "rendering/UserCamera.hh"

#include "gui/Actions.hh"
#include "gui/Gui.hh"
#include "gui/InsertModelWidget.hh"
#include "gui/SkyWidget.hh"
#include "gui/ModelListWidget.hh"
#include "gui/LightListWidget.hh"
#include "gui/WorldPropertiesWidget.hh"
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
  this->scenePub =
    this->node->Advertise<msgs::Scene>("~/scene", 1);

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
  //this->CreateToolbars();

  QWidget *mainWidget = new QWidget;
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainWidget->show();
  this->setCentralWidget(mainWidget);

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

  QHBoxLayout *centerLayout = new QHBoxLayout;

  this->collapseButton = new QPushButton("<");
  this->collapseButton->setObjectName("collapseButton");
  this->collapseButton->setSizePolicy(QSizePolicy::Fixed,
                                      QSizePolicy::Expanding);

  this->collapseButton->setFocusPolicy(Qt::NoFocus);
  connect(this->collapseButton, SIGNAL(clicked()), this, SLOT(OnCollapse()));

  centerLayout->addWidget(this->treeWidget, 0);
  centerLayout->addWidget(collapseButton, 0);
  centerLayout->addWidget(this->renderWidget, 1);
  centerLayout->setContentsMargins(0, 0, 0, 0);
  centerLayout->setSpacing(0);

  mainLayout->addLayout(centerLayout);
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
  gazebo::stop();
  this->renderWidget->hide();
  this->treeWidget->hide();

  this->connections.clear();

  delete this->worldPropertiesWidget;
  delete this->renderWidget;
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

  g_pauseAct->setChecked(false);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Pause()
{
  msgs::WorldControl msg;
  msg.set_pause(true);

  g_playAct->setChecked(false);
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
  g_arrowAct->setChecked(true);
  gui::Events::createEntity("box", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateSphere()
{
  g_arrowAct->setChecked(true);
  gui::Events::createEntity("sphere", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateCylinder()
{
  g_arrowAct->setChecked(true);
  gui::Events::createEntity("cylinder", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateMesh()
{
  g_arrowAct->setChecked(true);
  gui::Events::createEntity("mesh", "mesh");
}

/////////////////////////////////////////////////
void MainWindow::CreatePointLight()
{
  g_arrowAct->setChecked(true);
  gui::Events::createEntity("pointlight", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateSpotLight()
{
  g_arrowAct->setChecked(true);
  gui::Events::createEntity("spotlight", "");
}

/////////////////////////////////////////////////
void MainWindow::CreateDirectionalLight()
{
  g_arrowAct->setChecked(true);
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
    this->showFullScreen();
    this->renderWidget->showFullScreen();
    this->treeWidget->hide();
    this->menuBar->hide();
    this->collapseButton->hide();
  }
  else
  {
    this->showNormal();
    this->renderWidget->showNormal();
    this->treeWidget->show();
    this->menuBar->show();
    this->collapseButton->show();
  }
}

/////////////////////////////////////////////////
void MainWindow::ViewReset()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->SetWorldPose(math::Pose(-5, 0, 1, 0, GZ_DTOR(11.31), 0));
}

/////////////////////////////////////////////////
void MainWindow::ViewGrid()
{
  msgs::Scene msg;
  msg.set_name("default");
  msg.set_grid(g_viewGridAct->isChecked());
  this->scenePub->Publish(msg);
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
  g_newAct = new QAction(tr("&New"), this);
  g_newAct->setShortcut(tr("Ctrl+N"));
  g_newAct->setStatusTip(tr("Create a new world"));
  connect(g_newAct, SIGNAL(triggered()), this, SLOT(New()));

  g_openAct = new QAction(tr("&Open"), this);
  g_openAct->setShortcut(tr("Ctrl+O"));
  g_openAct->setStatusTip(tr("Open an world file"));
  connect(g_openAct, SIGNAL(triggered()), this, SLOT(Open()));

  g_importAct = new QAction(tr("&Import Mesh"), this);
  g_importAct->setShortcut(tr("Ctrl+I"));
  g_importAct->setStatusTip(tr("Import a Collada mesh"));
  connect(g_importAct, SIGNAL(triggered()), this, SLOT(Import()));


  g_saveAct = new QAction(tr("&Save"), this);
  g_saveAct->setShortcut(tr("Ctrl+S"));
  g_saveAct->setStatusTip(tr("Save world"));
  connect(g_saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  g_saveAsAct = new QAction(tr("Save &As"), this);
  g_saveAsAct->setShortcut(tr("Ctrl+Shift+S"));
  g_saveAsAct->setStatusTip(tr("Save world to new file"));
  connect(g_saveAsAct, SIGNAL(triggered()), this, SLOT(SaveAs()));


  g_aboutAct = new QAction(tr("&About"), this);
  g_aboutAct->setStatusTip(tr("Show the about info"));
  connect(g_aboutAct, SIGNAL(triggered()), this, SLOT(About()));

  g_quitAct = new QAction(tr("&Quit"), this);
  g_quitAct->setStatusTip(tr("Quit"));
  connect(g_quitAct, SIGNAL(triggered()), this, SLOT(close()));

  g_newModelAct = new QAction(tr("New &Model"), this);
  g_newModelAct->setShortcut(tr("Ctrl+M"));
  g_newModelAct->setStatusTip(tr("Create a new model"));
  connect(g_newModelAct, SIGNAL(triggered()), this, SLOT(NewModel()));

  g_resetModelsAct = new QAction(tr("&Reset Model Poses"), this);
  g_resetModelsAct->setShortcut(tr("Ctrl+Shift-R"));
  g_resetModelsAct->setStatusTip(tr("Reset model poses"));
  connect(g_resetModelsAct, SIGNAL(triggered()), this,
    SLOT(OnResetModelOnly()));

  g_resetWorldAct = new QAction(tr("&Reset World"), this);
  g_resetWorldAct->setShortcut(tr("Ctrl+R"));
  g_resetWorldAct->setStatusTip(tr("Reset the world"));
  connect(g_resetWorldAct, SIGNAL(triggered()), this, SLOT(OnResetWorld()));


  g_editWorldPropertiesAct = new QAction(tr("&World"), this);
  g_editWorldPropertiesAct->setShortcut(tr("Ctrl+W"));
  g_editWorldPropertiesAct->setStatusTip(tr("Edit World Properties"));
  connect(g_editWorldPropertiesAct, SIGNAL(triggered()), this,
          SLOT(EditWorldProperties()));


  g_playAct = new QAction(QIcon(":/images/play.png"), tr("Play"), this);
  g_playAct->setStatusTip(tr("Run the world"));
  g_playAct->setCheckable(true);
  g_playAct->setChecked(true);
  connect(g_playAct, SIGNAL(triggered()), this, SLOT(Play()));

  g_pauseAct = new QAction(QIcon(":/images/pause.png"), tr("Pause"), this);
  g_pauseAct->setStatusTip(tr("Pause the world"));
  g_pauseAct->setCheckable(true);
  g_pauseAct->setChecked(false);
  connect(g_pauseAct, SIGNAL(triggered()), this, SLOT(Pause()));

  g_stepAct = new QAction(QIcon(":/images/end.png"), tr("Step"), this);
  g_stepAct->setStatusTip(tr("Step the world"));
  connect(g_stepAct, SIGNAL(triggered()), this, SLOT(Step()));


  g_arrowAct = new QAction(QIcon(":/images/arrow.png"),
      tr("Position object"), this);
  g_arrowAct->setStatusTip(tr("Move camera"));
  g_arrowAct->setCheckable(true);
  g_arrowAct->setChecked(true);
  connect(g_arrowAct, SIGNAL(triggered()), this, SLOT(Arrow()));

  g_ringPoseAct = new QAction(QIcon(":/images/translate.png"),
      tr("Position object"), this);
  g_ringPoseAct->setStatusTip(tr("Position object"));
  g_ringPoseAct->setCheckable(true);
  g_ringPoseAct->setChecked(false);
  connect(g_ringPoseAct, SIGNAL(triggered()), this, SLOT(RingPose()));


  g_boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  g_boxCreateAct->setStatusTip(tr("Create a box"));
  g_boxCreateAct->setCheckable(true);
  connect(g_boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));

  g_sphereCreateAct = new QAction(QIcon(":/images/sphere.png"),
      tr("Sphere"), this);
  g_sphereCreateAct->setStatusTip(tr("Create a sphere"));
  g_sphereCreateAct->setCheckable(true);
  connect(g_sphereCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSphere()));

  g_cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Cylinder"), this);
  g_cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  g_cylinderCreateAct->setCheckable(true);
  connect(g_cylinderCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateCylinder()));

  g_meshCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Mesh"), this);
  g_meshCreateAct->setStatusTip(tr("Create a mesh"));
  g_meshCreateAct->setCheckable(true);
  connect(g_meshCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateMesh()));


  g_pointLghtCreateAct = new QAction(QIcon(":/images/pointlight.png"),
      tr("Point Light"), this);
  g_pointLghtCreateAct->setStatusTip(tr("Create a point light"));
  g_pointLghtCreateAct->setCheckable(true);
  connect(g_pointLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreatePointLight()));

  g_spotLghtCreateAct = new QAction(QIcon(":/images/spotlight.png"),
      tr("Spot Light"), this);
  g_spotLghtCreateAct->setStatusTip(tr("Create a spot light"));
  g_spotLghtCreateAct->setCheckable(true);
  connect(g_spotLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSpotLight()));

  g_dirLghtCreateAct = new QAction(QIcon(":/images/directionallight.png"),
      tr("Directional Light"), this);
  g_dirLghtCreateAct->setStatusTip(tr("Create a directional light"));
  g_dirLghtCreateAct->setCheckable(true);
  connect(g_dirLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateDirectionalLight()));

  g_viewResetAct = new QAction(tr("Reset View"), this);
  g_viewResetAct->setStatusTip(tr("Move camera to origin"));
  connect(g_viewResetAct, SIGNAL(triggered()), this,
      SLOT(ViewReset()));

  g_viewGridAct = new QAction(tr("Grid"), this);
  g_viewGridAct->setStatusTip(tr("View Grid"));
  g_viewGridAct->setCheckable(true);
  g_viewGridAct->setChecked(true);
  connect(g_viewGridAct, SIGNAL(triggered()), this,
          SLOT(ViewGrid()));

  g_viewFullScreenAct = new QAction(tr("Full Screen"), this);
  g_viewFullScreenAct->setStatusTip(tr("View Full Screen(F-11 to exit)"));
  connect(g_viewFullScreenAct, SIGNAL(triggered()), this,
      SLOT(ViewFullScreen()));

  g_viewFPSAct = new QAction(tr("FPS View Control"), this);
  g_viewFPSAct->setStatusTip(tr("First Person Shooter View Style"));
  connect(g_viewFPSAct, SIGNAL(triggered()), this, SLOT(ViewFPS()));

  g_viewOrbitAct = new QAction(tr("Orbit View Control"), this);
  g_viewOrbitAct->setStatusTip(tr("Orbit View Style"));
  connect(g_viewOrbitAct, SIGNAL(triggered()), this, SLOT(ViewOrbit()));
}

/////////////////////////////////////////////////
void MainWindow::CreateMenus()
{
  QHBoxLayout *menuLayout = new QHBoxLayout;

  QFrame *frame = new QFrame;
  this->menuBar =  new QMenuBar;
  this->menuBar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  menuLayout->addWidget(this->menuBar);
  menuLayout->addStretch(5);
  menuLayout->setContentsMargins(0, 0, 0, 0);

  frame->setLayout(menuLayout);
  frame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

  this->setMenuWidget(frame);

  this->fileMenu = this->menuBar->addMenu(tr("&File"));
  this->fileMenu->addAction(g_openAct);
  this->fileMenu->addAction(g_importAct);
  this->fileMenu->addAction(g_newAct);
  this->fileMenu->addAction(g_saveAct);
  this->fileMenu->addAction(g_saveAsAct);
  this->fileMenu->addSeparator();
  this->fileMenu->addAction(g_quitAct);

  this->editMenu = this->menuBar->addMenu(tr("&Edit"));
  this->editMenu->addAction(g_resetModelsAct);
  this->editMenu->addAction(g_resetWorldAct);
  this->editMenu->addAction(g_editWorldPropertiesAct);

  this->viewMenu = this->menuBar->addMenu(tr("&View"));
  this->viewMenu->addAction(g_viewGridAct);
  this->viewMenu->addSeparator();
  this->viewMenu->addAction(g_viewResetAct);
  this->viewMenu->addAction(g_viewFullScreenAct);
  this->viewMenu->addSeparator();
  this->viewMenu->addAction(g_viewFPSAct);
  this->viewMenu->addAction(g_viewOrbitAct);

  this->menuBar->addSeparator();

  this->helpMenu = this->menuBar->addMenu(tr("&Help"));
  this->helpMenu->addAction(g_aboutAct);
}

/////////////////////////////////////////////////
void MainWindow::CreateToolbars()
{
  this->playToolbar = this->addToolBar(tr("Play"));
  this->playToolbar->addAction(g_playAct);
  this->playToolbar->addAction(g_pauseAct);
  this->playToolbar->addAction(g_stepAct);
}

/////////////////////////////////////////////////
void MainWindow::OnMoveMode(bool _mode)
{
  if (_mode)
  {
    g_boxCreateAct->setChecked(false);
    g_sphereCreateAct->setChecked(false);
    g_cylinderCreateAct->setChecked(false);
    g_meshCreateAct->setChecked(false);
    g_pointLghtCreateAct->setChecked(false);
    g_spotLghtCreateAct->setChecked(false);
    g_dirLghtCreateAct->setChecked(false);
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
      const msgs::Pose &msg_pose = _msg->camera().origin();

      math::Vector3 cam_pose_pos = math::Vector3(
        msg_pose.position().x(),
        msg_pose.position().y(),
        msg_pose.position().z());

      math::Quaternion cam_pose_rot = math::Quaternion(
        msg_pose.orientation().w(),
        msg_pose.orientation().x(),
        msg_pose.orientation().y(),
        msg_pose.orientation().z());

      math::Pose cam_pose(cam_pose_pos, cam_pose_rot);

      cam->SetWorldPose(cam_pose);
    }

    if (_msg->camera().has_view_controller())
    {
      cam->SetViewController(_msg->camera().view_controller());
    }

    if (_msg->camera().has_view_controller())
    {
      cam->SetViewController(_msg->camera().view_controller());
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
    g_arrowAct->setChecked(true);
  else if (_mode == "ring")
    g_ringPoseAct->setChecked(true);
}

/////////////////////////////////////////////////
void MainWindow::OnStats(ConstWorldStatisticsPtr &_msg)
{
  if (_msg->paused() && g_playAct->isChecked())
  {
    g_playAct->setChecked(false);
    g_pauseAct->setChecked(true);
  }
  else if (!_msg->paused() && !g_playAct->isChecked())
  {
    g_playAct->setChecked(true);
    g_pauseAct->setChecked(false);
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
    this->collapseButton->setStyleSheet(tr("QPushButton{margin-left: 10px;}"));
  }
  else
  {
    this->treeWidget->show();
    this->collapseButton->setText("<");
    this->collapseButton->setStyleSheet("QPushButton{margin-left:0px;}");
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

