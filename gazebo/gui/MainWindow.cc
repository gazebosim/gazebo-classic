/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <sdf/sdf.hh>

#include "gazebo/gazebo_config.h"

#include "gazebo/gui/TopicSelector.hh"
#include "gazebo/gui/DataLogger.hh"
#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/TopicView.hh"
#include "gazebo/gui/viewers/ImageView.hh"

#include "gazebo/gazebo.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/RenderEvents.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/InsertModelWidget.hh"
#include "gazebo/gui/ModelListWidget.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/ToolsWidget.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/building/BuildingEditor.hh"
#include "gazebo/gui/terrain/TerrainEditor.hh"


#ifdef HAVE_QWT
#include "gazebo/gui/Diagnostics.hh"
#endif


using namespace gazebo;
using namespace gui;

#define MINIMUM_TAB_WIDTH 250

extern bool g_fullscreen;

/////////////////////////////////////////////////
MainWindow::MainWindow()
  : renderWidget(0)
{
  this->menuLayout = NULL;
  this->menuBar = NULL;
  this->setObjectName("mainWindow");

  // Do these things first.
  {
    this->CreateActions();
  }

  this->inputStepSize = 1;
  this->requestMsg = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  gui::set_world(this->node->GetTopicNamespace());

  QWidget *mainWidget = new QWidget;
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainWidget->show();
  this->setCentralWidget(mainWidget);

  this->setDockOptions(QMainWindow::AnimatedDocks);

  this->leftColumn = new QStackedWidget(this);

  this->modelListWidget = new ModelListWidget(this);
  InsertModelWidget *insertModel = new InsertModelWidget(this);

  this->tabWidget = new QTabWidget();
  this->tabWidget->setObjectName("mainTab");
  this->tabWidget->addTab(this->modelListWidget, "World");
  this->tabWidget->addTab(insertModel, "Insert");
  this->tabWidget->setSizePolicy(QSizePolicy::Expanding,
                                 QSizePolicy::Expanding);
  this->tabWidget->setMinimumWidth(MINIMUM_TAB_WIDTH);
  this->AddToLeftColumn("default", this->tabWidget);

  this->CreateEditors();

  this->toolsWidget = new ToolsWidget();

  this->renderWidget = new RenderWidget(mainWidget);

  QHBoxLayout *centerLayout = new QHBoxLayout;

  QSplitter *splitter = new QSplitter(this);
  splitter->addWidget(this->leftColumn);
  splitter->addWidget(this->renderWidget);
  splitter->addWidget(this->toolsWidget);

  QList<int> sizes;
  sizes.push_back(MINIMUM_TAB_WIDTH);
  sizes.push_back(this->width() - MINIMUM_TAB_WIDTH);
  sizes.push_back(0);
  splitter->setSizes(sizes);

  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 2);
  splitter->setStretchFactor(2, 0);
  splitter->setCollapsible(2, false);
  splitter->setHandleWidth(10);

  centerLayout->addWidget(splitter);
  centerLayout->setContentsMargins(0, 0, 0, 0);
  centerLayout->setSpacing(0);

  mainLayout->setSpacing(0);
  mainLayout->addLayout(centerLayout, 1);
  mainLayout->addWidget(new QSizeGrip(mainWidget), 0,
                        Qt::AlignBottom | Qt::AlignRight);

  mainWidget->setLayout(mainLayout);

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));

  std::string title = "Gazebo";
  this->setWindowIconText(tr(title.c_str()));
  this->setWindowTitle(tr(title.c_str()));

  this->connections.push_back(
      gui::Events::ConnectFullScreen(
        boost::bind(&MainWindow::OnFullScreen, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectMoveMode(
        boost::bind(&MainWindow::OnMoveMode, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectManipMode(
        boost::bind(&MainWindow::OnManipMode, this, _1)));

  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&MainWindow::OnSetSelectedEntity, this, _1, _2)));

  this->connections.push_back(
      gui::Events::ConnectInputStepSize(
      boost::bind(&MainWindow::OnInputStepSizeChanged, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectFollow(
        boost::bind(&MainWindow::OnFollow, this, _1)));

  gui::ViewFactory::RegisterAll();

  // Do these things last
  {
    (void) new QShortcut(Qt::CTRL + Qt::Key_Q, this, SLOT(close()));
    this->CreateMenus();
  }
}

/////////////////////////////////////////////////
MainWindow::~MainWindow()
{
}

/////////////////////////////////////////////////
void MainWindow::Load()
{
  this->guiSub = this->node->Subscribe("~/gui", &MainWindow::OnGUI, this, true);
}

/////////////////////////////////////////////////
void MainWindow::Init()
{
  this->renderWidget->show();

  // Set the initial size of the window to 0.75 the desktop size,
  // with a minimum value of 1024x768.
  QSize winSize = QApplication::desktop()->size() * 0.75;
  winSize.setWidth(std::max(1024, winSize.width()));
  winSize.setHeight(std::max(768, winSize.height()));

  this->resize(winSize);

  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");
  this->serverControlPub =
    this->node->Advertise<msgs::ServerControl>("/gazebo/server/control");
  this->selectionPub =
    this->node->Advertise<msgs::Selection>("~/selection");
  this->scenePub =
    this->node->Advertise<msgs::Scene>("~/scene");

  this->newEntitySub = this->node->Subscribe("~/model/info",
      &MainWindow::OnModel, this, true);

  this->statsSub =
    this->node->Subscribe("~/world_stats", &MainWindow::OnStats, this);

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response",
      &MainWindow::OnResponse, this);

  this->worldModSub = this->node->Subscribe("/gazebo/world/modify",
                                            &MainWindow::OnWorldModify, this);

  this->requestMsg = msgs::CreateRequest("entity_list");
  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void MainWindow::closeEvent(QCloseEvent * /*_event*/)
{
  this->renderWidget->hide();
  this->tabWidget->hide();
  this->toolsWidget->hide();

  this->connections.clear();

  delete this->renderWidget;

  gazebo::shutdown();
}

/////////////////////////////////////////////////
void MainWindow::New()
{
  msgs::ServerControl msg;
  msg.set_new_world(true);
  this->serverControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Diagnostics()
{
#ifdef HAVE_QWT
  gui::Diagnostics *diag = new gui::Diagnostics(this);
  diag->show();
#endif
}

/////////////////////////////////////////////////
void MainWindow::SelectTopic()
{
  TopicSelector *selector = new TopicSelector(this);
  selector->exec();
  std::string topic = selector->GetTopic();
  std::string msgType = selector->GetMsgType();
  delete selector;

  if (!topic.empty())
  {
    TopicView *view = ViewFactory::NewView(msgType, topic, this);
    if (view)
      view->show();
    else
      gzerr << "Unable to create viewer for message type[" << msgType << "]\n";
  }
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
void MainWindow::SaveAs()
{
  std::string filename = QFileDialog::getSaveFileName(this,
      tr("Save World"), QString(),
      tr("SDF Files (*.xml *.sdf *.world)")).toStdString();

  // Return if the user has canceled.
  if (filename.empty())
    return;

  g_saveAct->setEnabled(true);
  this->saveFilename = filename;
  this->Save();
}

/////////////////////////////////////////////////
void MainWindow::Save()
{
  // Get the latest world in SDF.
  boost::shared_ptr<msgs::Response> response =
    transport::request(get_world(), "world_sdf");

  msgs::GzString msg;
  std::string msgData;

  // Make sure the response is correct
  if (response->response() != "error" && response->type() == msg.GetTypeName())
  {
    // Parse the response message
    msg.ParseFromString(response->serialized_data());

    // Parse the string into sdf, so that we can insert user camera settings.
    sdf::SDF sdf_parsed;
    sdf_parsed.SetFromString(msg.data());
    // Check that sdf contains world
    if (sdf_parsed.root->HasElement("world"))
    {
      sdf::ElementPtr world = sdf_parsed.root->GetElement("world");
      sdf::ElementPtr guiElem = world->GetElement("gui");

      if (guiElem->HasAttribute("fullscreen"))
        guiElem->GetAttribute("fullscreen")->Set(g_fullscreen);

      sdf::ElementPtr cameraElem = guiElem->GetElement("camera");
      rendering::UserCameraPtr cam = gui::get_active_camera();

      cameraElem->GetElement("pose")->Set(cam->GetWorldPose());
      cameraElem->GetElement("view_controller")->Set(
          cam->GetViewControllerTypeString());
      // TODO: export track_visual properties as well.
      msgData = sdf_parsed.root->ToString("");
    }
    else
    {
      msgData = msg.data();
      gzerr << "Unable to parse world file to add user camera settings.\n";
    }

    // Open the file
    std::ofstream out(this->saveFilename.c_str(), std::ios::out);

    if (!out)
    {
      QMessageBox msgBox;
      std::string str = "Unable to open file: " + this->saveFilename + "\n";
      str += "Check file permissions.";
      msgBox.setText(str.c_str());
      msgBox.exec();
    }
    else
      out << msgData;

    out.close();
  }
  else
  {
    QMessageBox msgBox;
    msgBox.setText("Unable to save world.\n"
                   "Unable to retrieve SDF world description from server.");
    msgBox.exec();
  }
}

/////////////////////////////////////////////////
void MainWindow::About()
{
  std::string helpTxt;

  helpTxt = "<table>"
    "<tr><td style='padding-right:20px'>"
    "<img src=':images/gazebo_neg_60x71.png'/></td>"
    "<td>";
  helpTxt += GAZEBO_VERSION_HEADER;
  helpTxt += "</td></tr></table>";

  helpTxt += "<div style='margin-left: 10px'>"
  "<div>"
    "<table>"
      "<tr>"
        "<td style='padding-right: 10px;'>Tutorials:</td>"
        "<td><a href='http://gazebosim.org/wiki/tutorials' "
        "style='text-decoration: none; color: #f58113'>"
        "http://gazebosim.org/wiki/tutorials</a></td>"
      "</tr>"
      "<tr>"
        "<td style='padding-right: 10px;'>User Guide:</td>"
        "<td><a href='http://gazebosim.org/user_guide' "
        "style='text-decoration: none; color: #f58113'>"
        "http://gazebosim.org/user_guide</a></td>"
      "</tr>"
      "<tr>"
        "<td style='padding-right: 10px;'>API:</td>"
        "<td><a href='http://gazebosim.org/api' "
        "style='text-decoration: none; color: #f58113'>"
        "http://gazebosim.org/api</a></td>"
      "</tr>"
      "<tr>"
        "<td style='padding-right: 10px;'>SDF:</td>"
        "<td><a href='http://gazebosim.org/sdf' "
        "style='text-decoration: none; color: #f58113'>"
        "http://gazebosim.org/sdf</a></td>"
      "</tr>"
      "<tr>"
        "<td style='padding-right: 10px;'>Messages:</td>"
        "<td><a href='http://gazebosim.org/msgs' "
        "style='text-decoration: none; color: #f58113'>"
        "http://gazebosim.org/msgs</a></td>"
      "</tr>"
    "</table>"
  "</div>";

  QPixmap icon(":images/gazebo_neg_60x71.png");
  QMessageBox aboutBox(this);
  aboutBox.setWindowTitle("About Gazebo");
  aboutBox.setTextFormat(Qt::RichText);
  aboutBox.setText(QString::fromStdString(helpTxt));
  aboutBox.exec();
}

/////////////////////////////////////////////////
void MainWindow::Play()
{
  msgs::WorldControl msg;
  msg.set_pause(false);

  g_pauseAct->setVisible(true);
  g_playAct->setVisible(false);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Pause()
{
  msgs::WorldControl msg;
  msg.set_pause(true);

  g_pauseAct->setVisible(false);
  g_playAct->setVisible(true);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Step()
{
  msgs::WorldControl msg;
  msg.set_multi_step(this->inputStepSize);

  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::OnInputStepSizeChanged(int _value)
{
  this->inputStepSize = _value;
}

/////////////////////////////////////////////////
void MainWindow::OnFollow(const std::string &_modelName)
{
  if (_modelName.empty())
  {
    this->renderWidget->DisplayOverlayMsg("", 0);
    this->editMenu->setEnabled(true);
  }
  else
  {
    this->renderWidget->DisplayOverlayMsg(
        "Press Escape to exit Follow mode", 0);
    this->editMenu->setEnabled(false);
  }
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
void MainWindow::Arrow()
{
  gui::Events::manipMode("select");
}

/////////////////////////////////////////////////
void MainWindow::Translate()
{
  gui::Events::manipMode("translate");
}

/////////////////////////////////////////////////
void MainWindow::Rotate()
{
  gui::Events::manipMode("rotate");
}

/////////////////////////////////////////////////
void MainWindow::Scale()
{
  gui::Events::manipMode("scale");
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
void MainWindow::CaptureScreenshot()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->SetCaptureDataOnce();
  this->renderWidget->DisplayOverlayMsg(
      "Screenshot saved in: " + cam->GetScreenshotPath(), 2000);
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
    this->leftColumn->hide();
    this->toolsWidget->hide();
    this->menuBar->hide();
  }
  else
  {
    this->showNormal();
    this->renderWidget->showNormal();
    this->leftColumn->show();
    this->toolsWidget->show();
    this->menuBar->show();
  }
}

/////////////////////////////////////////////////
void MainWindow::Reset()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();

  math::Vector3 camPos(5, -5, 2);
  math::Vector3 lookAt(0, 0, 0);
  math::Vector3 delta = camPos - lookAt;

  double yaw = atan2(delta.x, delta.y);
  double pitch = atan2(delta.z, sqrt(delta.x*delta.x + delta.y*delta.y));
  cam->SetWorldPose(math::Pose(camPos, math::Vector3(0, pitch, yaw)));
}

/////////////////////////////////////////////////
void MainWindow::ShowCollisions()
{
  if (g_showCollisionsAct->isChecked())
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "show_collision", "all");
  else
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "hide_collision", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowGrid()
{
  msgs::Scene msg;
  msg.set_name(gui::get_world());
  msg.set_grid(g_showGridAct->isChecked());
  this->scenePub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::ShowJoints()
{
  if (g_showJointsAct->isChecked())
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "show_joints", "all");
  else
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "hide_joints", "all");
}

/////////////////////////////////////////////////
void MainWindow::SetTransparent()
{
  if (g_transparentAct->isChecked())
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "set_transparent", "all");
  else
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "set_opaque", "all");
}

/////////////////////////////////////////////////
void MainWindow::SetWireframe()
{
  if (g_viewWireframeAct->isChecked())
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "set_wireframe", "all");
  else
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "set_solid", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowCOM()
{
  if (g_showCOMAct->isChecked())
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "show_com", "all");
  else
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "hide_com", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowContacts()
{
  if (g_showContactsAct->isChecked())
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "show_contact", "all");
  else
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "hide_contact", "all");
}

/////////////////////////////////////////////////
void MainWindow::FullScreen()
{
  g_fullscreen = !g_fullscreen;
  gui::Events::fullScreen(g_fullscreen);
}

/////////////////////////////////////////////////
void MainWindow::FPS()
{
  gui::Events::fps();
}

/////////////////////////////////////////////////
void MainWindow::Orbit()
{
  gui::Events::orbit();
}

/////////////////////////////////////////////////
void MainWindow::DataLogger()
{
  gui::DataLogger *dataLogger = new gui::DataLogger(this);
  dataLogger->show();
}

/////////////////////////////////////////////////
void MainWindow::CreateActions()
{
  /*g_newAct = new QAction(tr("&New World"), this);
  g_newAct->setShortcut(tr("Ctrl+N"));
  g_newAct->setStatusTip(tr("Create a new world"));
  connect(g_newAct, SIGNAL(triggered()), this, SLOT(New()));
  */

  g_topicVisAct = new QAction(tr("Topic Visualization"), this);
  g_topicVisAct->setShortcut(tr("Ctrl+T"));
  g_topicVisAct->setStatusTip(tr("Select a topic to visualize"));
  connect(g_topicVisAct, SIGNAL(triggered()), this, SLOT(SelectTopic()));

#ifdef HAVE_QWT
  /*g_diagnosticsAct = new QAction(tr("Diagnostic Plot"), this);
  g_diagnosticsAct->setShortcut(tr("Ctrl+U"));
  g_diagnosticsAct->setStatusTip(tr("Plot diagnostic information"));
  connect(g_diagnosticsAct, SIGNAL(triggered()), this, SLOT(Diagnostics()));
  */
#endif

  g_openAct = new QAction(tr("&Open World"), this);
  g_openAct->setShortcut(tr("Ctrl+O"));
  g_openAct->setStatusTip(tr("Open an world file"));
  connect(g_openAct, SIGNAL(triggered()), this, SLOT(Open()));

  /*g_importAct = new QAction(tr("&Import Mesh"), this);
  g_importAct->setShortcut(tr("Ctrl+I"));
  g_importAct->setStatusTip(tr("Import a Collada mesh"));
  connect(g_importAct, SIGNAL(triggered()), this, SLOT(Import()));
  */

  g_saveAct = new QAction(tr("&Save World"), this);
  g_saveAct->setShortcut(tr("Ctrl+S"));
  g_saveAct->setStatusTip(tr("Save world"));
  g_saveAct->setEnabled(false);
  connect(g_saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  g_saveAsAct = new QAction(tr("Save World &As"), this);
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
  g_resetModelsAct->setShortcut(tr("Ctrl+Shift+R"));
  g_resetModelsAct->setStatusTip(tr("Reset model poses"));
  connect(g_resetModelsAct, SIGNAL(triggered()), this,
    SLOT(OnResetModelOnly()));

  g_resetWorldAct = new QAction(tr("&Reset World"), this);
  g_resetWorldAct->setShortcut(tr("Ctrl+R"));
  g_resetWorldAct->setStatusTip(tr("Reset the world"));
  connect(g_resetWorldAct, SIGNAL(triggered()), this, SLOT(OnResetWorld()));

  QActionGroup *editorGroup = new QActionGroup(this);

  g_editBuildingAct = new QAction(tr("&Building Editor"), editorGroup);
  g_editBuildingAct->setShortcut(tr("Ctrl+B"));
  g_editBuildingAct->setStatusTip(tr("Enter Building Editor Mode"));
  g_editBuildingAct->setCheckable(true);
  g_editBuildingAct->setChecked(false);

  g_editTerrainAct = new QAction(tr("&Terrain Editor"), editorGroup);
  g_editTerrainAct->setShortcut(tr("Ctrl+E"));
  g_editTerrainAct->setStatusTip(tr("Enter Terrain Editor Mode"));
  g_editTerrainAct->setCheckable(true);
  g_editTerrainAct->setChecked(false);

  g_stepAct = new QAction(QIcon(":/images/end.png"), tr("Step"), this);
  g_stepAct->setStatusTip(tr("Step the world"));
  connect(g_stepAct, SIGNAL(triggered()), this, SLOT(Step()));
  this->CreateDisabledIcon(":/images/end.png", g_stepAct);

  g_playAct = new QAction(QIcon(":/images/play.png"), tr("Play"), this);
  g_playAct->setStatusTip(tr("Run the world"));
  g_playAct->setVisible(false);
  connect(g_playAct, SIGNAL(triggered()), this, SLOT(Play()));
  connect(g_playAct, SIGNAL(changed()), this, SLOT(OnPlayActionChanged()));
  this->OnPlayActionChanged();

  g_pauseAct = new QAction(QIcon(":/images/pause.png"), tr("Pause"), this);
  g_pauseAct->setStatusTip(tr("Pause the world"));
  g_pauseAct->setVisible(true);
  connect(g_pauseAct, SIGNAL(triggered()), this, SLOT(Pause()));

  g_arrowAct = new QAction(QIcon(":/images/arrow.png"),
      tr("Selection Mode"), this);
  g_arrowAct->setStatusTip(tr("Move camera"));
  g_arrowAct->setCheckable(true);
  g_arrowAct->setChecked(true);
  g_arrowAct->setToolTip(tr("Selection Mode (Esc)"));
  connect(g_arrowAct, SIGNAL(triggered()), this, SLOT(Arrow()));

  g_translateAct = new QAction(QIcon(":/images/translate.png"),
      tr("&Translation Mode"), this);
  g_translateAct->setStatusTip(tr("Translate an object"));
  g_translateAct->setCheckable(true);
  g_translateAct->setChecked(false);
  g_translateAct->setToolTip(tr("Translation Mode (T)"));
  connect(g_translateAct, SIGNAL(triggered()), this, SLOT(Translate()));
  this->CreateDisabledIcon(":/images/translate.png", g_translateAct);

  g_rotateAct = new QAction(QIcon(":/images/rotate.png"),
      tr("Rotation Mode"), this);
  g_rotateAct->setStatusTip(tr("Rotate an object"));
  g_rotateAct->setCheckable(true);
  g_rotateAct->setChecked(false);
  g_rotateAct->setToolTip(tr("Rotation Mode (R)"));
  connect(g_rotateAct, SIGNAL(triggered()), this, SLOT(Rotate()));
  this->CreateDisabledIcon(":/images/rotate.png", g_rotateAct);

  g_scaleAct = new QAction(QIcon(":/images/scale.png"),
      tr("Scale Mode"), this);
  g_scaleAct->setStatusTip(tr("Scale an object"));
  g_scaleAct->setCheckable(true);
  g_scaleAct->setChecked(false);
  g_scaleAct->setToolTip(tr("Scale Mode (S)"));
  connect(g_scaleAct, SIGNAL(triggered()), this, SLOT(Scale()));

  g_boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  g_boxCreateAct->setStatusTip(tr("Create a box"));
  g_boxCreateAct->setCheckable(true);
  connect(g_boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));
  this->CreateDisabledIcon(":/images/box.png", g_boxCreateAct);

  g_sphereCreateAct = new QAction(QIcon(":/images/sphere.png"),
      tr("Sphere"), this);
  g_sphereCreateAct->setStatusTip(tr("Create a sphere"));
  g_sphereCreateAct->setCheckable(true);
  connect(g_sphereCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSphere()));
  this->CreateDisabledIcon(":/images/sphere.png", g_sphereCreateAct);

  g_cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Cylinder"), this);
  g_cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  g_cylinderCreateAct->setCheckable(true);
  connect(g_cylinderCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateCylinder()));
  this->CreateDisabledIcon(":/images/cylinder.png", g_cylinderCreateAct);

  g_meshCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Mesh"), this);
  g_meshCreateAct->setStatusTip(tr("Create a mesh"));
  g_meshCreateAct->setCheckable(true);
  connect(g_meshCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateMesh()));
  this->CreateDisabledIcon(":/images/cylinder.png", g_meshCreateAct);


  g_pointLghtCreateAct = new QAction(QIcon(":/images/pointlight.png"),
      tr("Point Light"), this);
  g_pointLghtCreateAct->setStatusTip(tr("Create a point light"));
  g_pointLghtCreateAct->setCheckable(true);
  connect(g_pointLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreatePointLight()));
  this->CreateDisabledIcon(":/images/pointlight.png", g_pointLghtCreateAct);

  g_spotLghtCreateAct = new QAction(QIcon(":/images/spotlight.png"),
      tr("Spot Light"), this);
  g_spotLghtCreateAct->setStatusTip(tr("Create a spot light"));
  g_spotLghtCreateAct->setCheckable(true);
  connect(g_spotLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSpotLight()));
  this->CreateDisabledIcon(":/images/spotlight.png", g_spotLghtCreateAct);

  g_dirLghtCreateAct = new QAction(QIcon(":/images/directionallight.png"),
      tr("Directional Light"), this);
  g_dirLghtCreateAct->setStatusTip(tr("Create a directional light"));
  g_dirLghtCreateAct->setCheckable(true);
  connect(g_dirLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateDirectionalLight()));
  this->CreateDisabledIcon(":/images/directionallight.png", g_dirLghtCreateAct);

  g_resetAct = new QAction(tr("Reset Camera"), this);
  g_resetAct->setStatusTip(tr("Move camera to pose"));
  connect(g_resetAct, SIGNAL(triggered()), this,
      SLOT(Reset()));

  g_showCollisionsAct = new QAction(tr("Collisions"), this);
  g_showCollisionsAct->setStatusTip(tr("Show Collisions"));
  g_showCollisionsAct->setCheckable(true);
  g_showCollisionsAct->setChecked(false);
  connect(g_showCollisionsAct, SIGNAL(triggered()), this,
          SLOT(ShowCollisions()));

  g_showGridAct = new QAction(tr("Grid"), this);
  g_showGridAct->setStatusTip(tr("Show Grid"));
  g_showGridAct->setCheckable(true);
  g_showGridAct->setChecked(true);
  connect(g_showGridAct, SIGNAL(triggered()), this,
          SLOT(ShowGrid()));

  g_transparentAct = new QAction(tr("Transparent"), this);
  g_transparentAct->setStatusTip(tr("Transparent"));
  g_transparentAct->setCheckable(true);
  g_transparentAct->setChecked(false);
  connect(g_transparentAct, SIGNAL(triggered()), this,
          SLOT(SetTransparent()));

  g_viewWireframeAct = new QAction(tr("Wireframe"), this);
  g_viewWireframeAct->setStatusTip(tr("Wireframe"));
  g_viewWireframeAct->setCheckable(true);
  g_viewWireframeAct->setChecked(false);
  connect(g_viewWireframeAct, SIGNAL(triggered()), this,
          SLOT(SetWireframe()));

  g_showCOMAct = new QAction(tr("Center of Mass / Inertia"), this);
  g_showCOMAct->setStatusTip(tr("Show COM/MOI"));
  g_showCOMAct->setCheckable(true);
  g_showCOMAct->setChecked(false);
  connect(g_showCOMAct, SIGNAL(triggered()), this,
          SLOT(ShowCOM()));

  g_showContactsAct = new QAction(tr("Contacts"), this);
  g_showContactsAct->setStatusTip(tr("Show Contacts"));
  g_showContactsAct->setCheckable(true);
  g_showContactsAct->setChecked(false);
  connect(g_showContactsAct, SIGNAL(triggered()), this,
          SLOT(ShowContacts()));

  g_showJointsAct = new QAction(tr("Joints"), this);
  g_showJointsAct->setStatusTip(tr("Show Joints"));
  g_showJointsAct->setCheckable(true);
  g_showJointsAct->setChecked(false);
  connect(g_showJointsAct, SIGNAL(triggered()), this,
          SLOT(ShowJoints()));


  g_fullScreenAct = new QAction(tr("Full Screen"), this);
  g_fullScreenAct->setStatusTip(tr("Full Screen(F-11 to exit)"));
  connect(g_fullScreenAct, SIGNAL(triggered()), this,
      SLOT(FullScreen()));

  // g_fpsAct = new QAction(tr("FPS View Control"), this);
  // g_fpsAct->setStatusTip(tr("First Person Shooter View Style"));
  // connect(g_fpsAct, SIGNAL(triggered()), this, SLOT(FPS()));

  g_orbitAct = new QAction(tr("Orbit View Control"), this);
  g_orbitAct->setStatusTip(tr("Orbit View Style"));
  connect(g_orbitAct, SIGNAL(triggered()), this, SLOT(Orbit()));

  g_dataLoggerAct = new QAction(tr("&Log Data"), this);
  g_dataLoggerAct->setShortcut(tr("Ctrl+D"));
  g_dataLoggerAct->setStatusTip(tr("Data Logging Utility"));
  connect(g_dataLoggerAct, SIGNAL(triggered()), this, SLOT(DataLogger()));

  g_screenshotAct = new QAction(QIcon(":/images/screenshot.png"),
      tr("Screenshot"), this);
  g_screenshotAct->setStatusTip(tr("Take a screenshot"));
  connect(g_screenshotAct, SIGNAL(triggered()), this,
      SLOT(CaptureScreenshot()));
}

/////////////////////////////////////////////////
void MainWindow::ShowMenuBar(QMenuBar *_bar)
{
  if (!this->menuLayout)
    this->menuLayout = new QHBoxLayout;

  // Remove all widgets from the menuLayout
  while (this->menuLayout->takeAt(0) != 0)
  {
  }

  if (!this->menuBar)
  {
    // create the native menu bar
    this->menuBar = new QMenuBar;
    this->menuBar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    this->setMenuBar(this->menuBar);

    this->CreateMenuBar();
  }

  this->menuBar->clear();

  QMenuBar *newMenuBar = NULL;
  if (!_bar)
  {
    // Get the main window's menubar
    // Note: for some reason we can not call menuBar() again,
    // so manually retrieving the menubar from the mainwindow.
    QList<QMenuBar *> menuBars  = this->findChildren<QMenuBar *>();
    newMenuBar = menuBars[0];
  }
  else
  {
    newMenuBar = _bar;
  }
  QList<QMenu *> menus  = newMenuBar->findChildren<QMenu *>();
  for (int i = 0; i < menus.size(); ++i)
  {
    this->menuBar->addMenu(menus[i]);
  }

  this->menuLayout->addWidget(this->menuBar);

  this->menuLayout->addStretch(5);
  this->menuLayout->setContentsMargins(0, 0, 0, 0);
}

/////////////////////////////////////////////////
void MainWindow::CreateMenuBar()
{
  // main window's menu bar
  QMenuBar *bar = QMainWindow::menuBar();

  QMenu *fileMenu = bar->addMenu(tr("&File"));
  // fileMenu->addAction(g_openAct);
  // fileMenu->addAction(g_importAct);
  // fileMenu->addAction(g_newAct);
  fileMenu->addAction(g_saveAct);
  fileMenu->addAction(g_saveAsAct);
  fileMenu->addSeparator();
  fileMenu->addAction(g_quitAct);

  this->editMenu = bar->addMenu(tr("&Edit"));
  editMenu->addAction(g_resetModelsAct);
  editMenu->addAction(g_resetWorldAct);
  editMenu->addAction(g_editBuildingAct);

  // \TODO: Add this back in when implementing the full Terrain Editor spec.
  // editMenu->addAction(g_editTerrainAct);

  QMenu *viewMenu = bar->addMenu(tr("&View"));
  viewMenu->addAction(g_showGridAct);
  viewMenu->addSeparator();

  viewMenu->addAction(g_transparentAct);
  viewMenu->addAction(g_viewWireframeAct);
  viewMenu->addSeparator();
  viewMenu->addAction(g_showCollisionsAct);
  viewMenu->addAction(g_showJointsAct);
  viewMenu->addAction(g_showCOMAct);
  viewMenu->addAction(g_showContactsAct);
  viewMenu->addSeparator();

  viewMenu->addAction(g_resetAct);
  viewMenu->addAction(g_fullScreenAct);
  viewMenu->addSeparator();
  // viewMenu->addAction(g_fpsAct);
  viewMenu->addAction(g_orbitAct);

  QMenu *windowMenu = bar->addMenu(tr("&Window"));
  windowMenu->addAction(g_topicVisAct);
  windowMenu->addSeparator();
  windowMenu->addAction(g_dataLoggerAct);

#ifdef HAVE_QWT
  // windowMenu->addAction(g_diagnosticsAct);
#endif

  bar->addSeparator();

  QMenu *helpMenu = bar->addMenu(tr("&Help"));
  helpMenu->addAction(g_aboutAct);
}

/////////////////////////////////////////////////
void MainWindow::CreateMenus()
{
  this->ShowMenuBar();

  QFrame *frame = new QFrame;
  frame->setLayout(this->menuLayout);
  frame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

  this->setMenuWidget(frame);
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
    this->FullScreen();
  }

  if (_msg->has_camera())
  {
    rendering::UserCameraPtr cam = gui::get_active_camera();

    if (_msg->camera().has_pose())
    {
      const msgs::Pose &msg_pose = _msg->camera().pose();

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
  if (_mode == "select" || _mode == "make_entity")
    g_arrowAct->setChecked(true);
}

/////////////////////////////////////////////////
void MainWindow::OnSetSelectedEntity(const std::string &_name,
                                     const std::string &/*_mode*/)
{
  if (!_name.empty())
  {
    this->tabWidget->setCurrentIndex(0);
  }
}

/////////////////////////////////////////////////
void MainWindow::OnStats(ConstWorldStatisticsPtr &_msg)
{
  if (_msg->paused() && g_pauseAct->isVisible())
  {
    g_pauseAct->setVisible(false);
    g_playAct->setVisible(true);
  }
  else if (!_msg->paused() && !g_playAct->isVisible())
  {
    g_pauseAct->setVisible(true);
    g_playAct->setVisible(false);
  }
}

/////////////////////////////////////////////////
void MainWindow::OnPlayActionChanged()
{
  if (g_playAct->isVisible())
  {
    g_stepAct->setToolTip("Step the world");
    g_stepAct->setEnabled(true);
  }
  else
  {
    g_stepAct->setToolTip("Pause the world before stepping");
    g_stepAct->setEnabled(false);
  }
}

/////////////////////////////////////////////////
void MainWindow::ItemSelected(QTreeWidgetItem *_item, int)
{
  _item->setExpanded(!_item->isExpanded());
}

/////////////////////////////////////////////////
void MainWindow::AddToLeftColumn(const std::string &_name, QWidget *_widget)
{
  this->leftColumn->addWidget(_widget);
  this->leftColumnStack[_name] = this->leftColumn->count()-1;
}

/////////////////////////////////////////////////
void MainWindow::ShowLeftColumnWidget(const std::string &_name)
{
  std::map<std::string, int>::iterator iter = this->leftColumnStack.find(_name);

  if (iter != this->leftColumnStack.end())
    this->leftColumn->setCurrentIndex(iter->second);
  else
    gzerr << "Widget with name[" << _name << "] has not been added to the left"
      << " column stack.\n";
}

/////////////////////////////////////////////////
RenderWidget *MainWindow::GetRenderWidget() const
{
  return this->renderWidget;
}

/////////////////////////////////////////////////
void MainWindow::CreateEditors()
{
  // Create a Terrain Editor
  this->editors.push_back(new TerrainEditor(this));

  // Create a Building Editor
  this->editors.push_back(new BuildingEditor(this));
}

/////////////////////////////////////////////////
void MainWindow::CreateDisabledIcon(const std::string &_pixmap, QAction *_act)
{
  QIcon icon = _act->icon();
  QPixmap pixmap(_pixmap.c_str());
  QPainter p(&pixmap);
  p.setCompositionMode(QPainter::CompositionMode_DestinationIn);
  p.fillRect(pixmap.rect(), QColor(0, 0, 0, 100));
  icon.addPixmap(pixmap, QIcon::Disabled);
  _act->setIcon(icon);
}
