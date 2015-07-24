/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sdf/sdf.hh>
#include <boost/scoped_ptr.hpp>

#include "gazebo/gazebo_config.h"

#include "gazebo/gui/GuiPlugin.hh"
#include "gazebo/gui/CloneWindow.hh"
#include "gazebo/gui/TopicSelector.hh"
#include "gazebo/gui/DataLogger.hh"
#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/TopicView.hh"
#include "gazebo/gui/viewers/ImageView.hh"

#include "gazebo/gazebo_client.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/InsertModelWidget.hh"
#include "gazebo/gui/LayersWidget.hh"
#include "gazebo/gui/ModelListWidget.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/ToolsWidget.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/AlignWidget.hh"
#include "gazebo/gui/ViewAngleWidget.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/SpaceNav.hh"
#include "gazebo/gui/building/BuildingEditor.hh"
#include "gazebo/gui/terrain/TerrainEditor.hh"
#include "gazebo/gui/model/ModelEditor.hh"

#ifdef HAVE_QWT
#include "gazebo/gui/Diagnostics.hh"
#endif

#ifdef HAVE_OCULUS
#include "gazebo/gui/OculusWindow.hh"
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
  LayersWidget *layersWidget = new LayersWidget(this);

  this->tabWidget = new QTabWidget();
  this->tabWidget->setObjectName("mainTab");
  this->tabWidget->addTab(this->modelListWidget, "World");
  this->tabWidget->addTab(insertModel, "Insert");
  this->tabWidget->addTab(layersWidget, "Layers");
  this->tabWidget->setSizePolicy(QSizePolicy::Expanding,
                                 QSizePolicy::Expanding);
  this->tabWidget->setMinimumWidth(MINIMUM_TAB_WIDTH);
  this->AddToLeftColumn("default", this->tabWidget);

  this->toolsWidget = new ToolsWidget();

  this->renderWidget = new RenderWidget(mainWidget);

  this->CreateEditors();

  QHBoxLayout *centerLayout = new QHBoxLayout;

  this->splitter = new QSplitter(this);
  this->splitter->addWidget(this->leftColumn);
  this->splitter->addWidget(this->renderWidget);
  this->splitter->addWidget(this->toolsWidget);
  this->splitter->setContentsMargins(0, 0, 0, 0);

#ifdef _WIN32
  // The splitter appears solid white in Windows, so we make it transparent.
  this->splitter->setStyleSheet(
  "QSplitter { color: #ffffff; background-color: transparent; }"
  "QSplitter::handle { color: #ffffff; background-color: transparent; }");
#endif

  QList<int> sizes;
  sizes.push_back(MINIMUM_TAB_WIDTH);
  sizes.push_back(this->width() - MINIMUM_TAB_WIDTH);
  sizes.push_back(0);
  this->splitter->setSizes(sizes);

  this->splitter->setStretchFactor(0, 0);
  this->splitter->setStretchFactor(1, 2);
  this->splitter->setStretchFactor(2, 0);
  this->splitter->setHandleWidth(10);

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

#ifdef HAVE_OCULUS
  this->oculusWindow = NULL;
#endif

  this->connections.push_back(
      gui::Events::ConnectLeftPaneVisibility(
        boost::bind(&MainWindow::SetLeftPaneVisibility, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectFullScreen(
        boost::bind(&MainWindow::OnFullScreen, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectShowToolbars(
        boost::bind(&MainWindow::OnShowToolbars, this, _1)));

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

  // Create a pointer to the space navigator interface
  this->spacenav = new SpaceNav();

  // Use a signal/slot to load plugins. This makes the process thread safe.
  connect(this, SIGNAL(AddPlugins()),
          this, SLOT(OnAddPlugins()), Qt::QueuedConnection);

  // Create data logger dialog
  this->dataLogger = new gui::DataLogger(this);
  connect(dataLogger, SIGNAL(rejected()), this, SLOT(OnDataLoggerClosed()));

  this->show();
}

/////////////////////////////////////////////////
MainWindow::~MainWindow()
{
  this->DeleteActions();
}

/////////////////////////////////////////////////
void MainWindow::Load()
{
  this->guiSub = this->node->Subscribe("~/gui", &MainWindow::OnGUI, this, true);
#ifdef HAVE_OCULUS
  int oculusAutoLaunch = getINIProperty<int>("oculus.autolaunch", 0);
  int oculusX = getINIProperty<int>("oculus.x", 0);
  int oculusY = getINIProperty<int>("oculus.y", 0);
  std::string visual = getINIProperty<std::string>("oculus.visual", "");

  if (oculusAutoLaunch == 1)
  {
    if (!visual.empty())
    {
      this->oculusWindow = new gui::OculusWindow(
        oculusX, oculusY, visual);

      if (this->oculusWindow->CreateCamera())
        this->oculusWindow->show();
    }
    else
      gzlog << "Oculus: No visual link specified in for attaching the camera. "
            << "Did you forget to set ~/.gazebo/gui.ini?\n";
  }
#endif

  // Load the space navigator
  if (!this->spacenav->Load())
    gzerr << "Unable to load space navigator\n";
}

/////////////////////////////////////////////////
void MainWindow::Init()
{
  // Default window size is entire desktop.
  QSize winSize = QApplication::desktop()->screenGeometry().size();

  // Get the size properties from the INI file.
  int winWidth = getINIProperty<int>("geometry.width", winSize.width());
  int winHeight = getINIProperty<int>("geometry.height", winSize.height());

  winWidth = winWidth < 0 ? winSize.width() : winWidth;
  winHeight = winHeight < 0 ? winSize.height() : winHeight;

  // Get the position properties from the INI file.
  int winXPos = getINIProperty<int>("geometry.x", 0);
  int winYPos = getINIProperty<int>("geometry.y", 0);

  this->setGeometry(winXPos, winYPos, winWidth, winHeight);

  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");
  this->serverControlPub =
    this->node->Advertise<msgs::ServerControl>("/gazebo/server/control");
  this->scenePub =
    this->node->Advertise<msgs::Scene>("~/scene");

  this->newEntitySub = this->node->Subscribe("~/model/info",
      &MainWindow::OnModel, this, true);

  this->lightSub = this->node->Subscribe("~/light", &MainWindow::OnLight, this);

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response",
      &MainWindow::OnResponse, this);

  this->worldModSub = this->node->Subscribe("/gazebo/world/modify",
                                            &MainWindow::OnWorldModify, this);

  this->requestMsg = msgs::CreateRequest("scene_info");
  this->requestPub->Publish(*this->requestMsg);

  gui::Events::mainWindowReady();
}

/////////////////////////////////////////////////
void MainWindow::closeEvent(QCloseEvent * /*_event*/)
{
  this->renderWidget->hide();
  this->tabWidget->hide();
  this->toolsWidget->hide();

  this->connections.clear();

#ifdef HAVE_OCULUS
  if (this->oculusWindow)
  {
    delete this->oculusWindow;
    this->oculusWindow = NULL;
  }
#endif
  delete this->renderWidget;

  // Cleanup the space navigator
  delete this->spacenav;
  this->spacenav = NULL;

  emit Close();

  gazebo::client::shutdown();
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
  // Note that file dialog static functions seem to be broken (issue #1514)
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
void MainWindow::SaveINI()
{
  char *home = getenv("HOME");
  if (!home)
  {
    gzerr << "HOME environment variable not found. "
      "Unable to save configuration file\n";
    return;
  }

  boost::filesystem::path path = home;
  path = path / ".gazebo" / "gui.ini";

  // When/if the configuration gets more complex, create a
  // configuration manager class so that all key-value pairs are kept
  // in a centralized place with error checking.
  setINIProperty("geometry.width", this->width());
  setINIProperty("geometry.height", this->height());
  setINIProperty("geometry.x", this->x());
  setINIProperty("geometry.y", this->y());

  gui::saveINI(path);
}

/////////////////////////////////////////////////
void MainWindow::SaveAs()
{
  QFileDialog fileDialog(this, tr("Save World"), QDir::homePath(),
      tr("SDF Files (*.xml *.sdf *.world)"));
  fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
  fileDialog.setAcceptMode(QFileDialog::AcceptSave);

  if (fileDialog.exec() == QDialog::Accepted)
  {
    QStringList selected = fileDialog.selectedFiles();
    if (selected.empty())
      return;

    std::string filename = selected[0].toStdString();

    g_saveAct->setEnabled(true);
    this->saveFilename = filename;
    this->Save();
  }
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
    if (sdf_parsed.Root()->HasElement("world"))
    {
      sdf::ElementPtr world = sdf_parsed.Root()->GetElement("world");
      sdf::ElementPtr guiElem = world->GetElement("gui");

      if (guiElem->HasAttribute("fullscreen"))
        guiElem->GetAttribute("fullscreen")->Set(g_fullscreen);

      sdf::ElementPtr cameraElem = guiElem->GetElement("camera");
      rendering::UserCameraPtr cam = gui::get_active_camera();

      cameraElem->GetElement("pose")->Set(cam->GetWorldPose());
      cameraElem->GetElement("view_controller")->Set(
          cam->GetViewControllerTypeString());

      cameraElem->GetElement("projection_type")->Set(cam->GetProjectionType());

      // TODO: export track_visual properties as well.
      msgData = sdf_parsed.Root()->ToString("");
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
void MainWindow::Clone()
{
  boost::scoped_ptr<CloneWindow> cloneWindow(new CloneWindow(this));
  if (cloneWindow->exec() == QDialog::Accepted && cloneWindow->IsValidPort())
  {
    // Create a gzserver clone in the server side.
    msgs::ServerControl msg;
    msg.set_save_world_name("");
    msg.set_clone(true);
    msg.set_new_port(cloneWindow->GetPort());
    this->serverControlPub->Publish(msg);
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
        "<td><a href='http://gazebosim.org/tutorials' "
        "style='text-decoration: none; color: #f58113'>"
        "http://gazebosim.org/tutorials</a></td>"
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

  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Pause()
{
  msgs::WorldControl msg;
  msg.set_pause(true);

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
void MainWindow::Align()
{
  for (unsigned int i = 0 ; i < this->alignActionGroups.size(); ++i)
  {
    this->alignActionGroups[i]->setExclusive(false);
    if (this->alignActionGroups[i]->checkedAction())
      this->alignActionGroups[i]->checkedAction()->setChecked(false);
    this->alignActionGroups[i]->setExclusive(true);
  }
}

/////////////////////////////////////////////////
void MainWindow::Snap()
{
  gui::Events::manipMode("snap");
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
    this->leftColumn->hide();
    this->toolsWidget->hide();
    this->menuBar->hide();
    this->setContentsMargins(0, 0, 0, 0);
    this->centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);
  }
  else
  {
    this->showNormal();
    this->leftColumn->show();
    this->toolsWidget->show();
    this->menuBar->show();
  }
  g_fullScreenAct->setChecked(_value);
  g_fullscreen = _value;
}

/////////////////////////////////////////////////
void MainWindow::OnShowToolbars(bool _value)
{
  if (_value)
  {
    this->GetRenderWidget()->GetTimePanel()->show();
    this->GetRenderWidget()->GetToolbar()->show();
  }
  else
  {
    this->GetRenderWidget()->GetTimePanel()->hide();
    this->GetRenderWidget()->GetToolbar()->hide();
  }
  g_showToolbarsAct->setChecked(_value);
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
void MainWindow::ShowOrigin()
{
  msgs::Scene msg;
  msg.set_name(gui::get_world());
  msg.set_origin_visual(g_showOriginAct->isChecked());
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
void MainWindow::ShowGUIOverlays()
{
  this->GetRenderWidget()->SetOverlaysVisible(g_overlayAct->isChecked());
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
void MainWindow::ShowInertia()
{
  if (g_showInertiaAct->isChecked())
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "show_inertia", "all");
  else
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "hide_inertia", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowLinkFrame()
{
  if (g_showLinkFrameAct->isChecked())
  {
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "show_link_frame", "all");
  }
  else
  {
    transport::requestNoReply(this->node->GetTopicNamespace(),
        "hide_link_frame", "all");
  }
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
void MainWindow::ShowToolbars()
{
  gui::Events::showToolbars(g_showToolbarsAct->isChecked());
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
void MainWindow::ViewOculus()
{
#ifdef HAVE_OCULUS
  rendering::ScenePtr scene = rendering::get_scene();
  if (scene->GetOculusCameraCount() != 0)
  {
    gzlog << "Oculus camera already exists." << std::endl;
    return;
  }

  int oculusX = getINIProperty<int>("oculus.x", 0);
  int oculusY = getINIProperty<int>("oculus.y", 0);
  std::string visual = getINIProperty<std::string>("oculus.visual", "");

  if (!visual.empty())
  {
    this->oculusWindow = new gui::OculusWindow(
        oculusX, oculusY, visual);

    if (this->oculusWindow->CreateCamera())
      this->oculusWindow->show();
  }
  else
  {
    gzlog << "Oculus: No visual link specified in for attaching the camera. "
          << "Did you forget to set ~/.gazebo/gui.ini?\n";
  }
#endif
}

/////////////////////////////////////////////////
void MainWindow::DataLogger()
{
  if (g_dataLoggerAct->isChecked())
  {
    this->dataLogger->show();
  }
  else
  {
    this->dataLogger->close();
  }
}

/////////////////////////////////////////////////
void MainWindow::OnDataLoggerClosed()
{
  // Uncheck action on toolbar when user closes dialog
  g_dataLoggerAct->setChecked(false);
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

  g_saveAct = new QAction(tr("&Save World"), this);
  g_saveAct->setShortcut(tr("Ctrl+S"));
  g_saveAct->setStatusTip(tr("Save world"));
  g_saveAct->setEnabled(false);
  connect(g_saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  g_saveAsAct = new QAction(tr("Save World &As"), this);
  g_saveAsAct->setShortcut(tr("Ctrl+Shift+S"));
  g_saveAsAct->setStatusTip(tr("Save world to new file"));
  connect(g_saveAsAct, SIGNAL(triggered()), this, SLOT(SaveAs()));

  g_saveCfgAct = new QAction(tr("Save &Configuration"), this);
  g_saveCfgAct->setStatusTip(tr("Save GUI configuration"));
  connect(g_saveCfgAct, SIGNAL(triggered()), this, SLOT(SaveINI()));

  g_cloneAct = new QAction(tr("Clone World"), this);
  g_cloneAct->setStatusTip(tr("Clone the world"));
  connect(g_cloneAct, SIGNAL(triggered()), this, SLOT(Clone()));

  g_aboutAct = new QAction(tr("&About"), this);
  g_aboutAct->setStatusTip(tr("Show the about info"));
  connect(g_aboutAct, SIGNAL(triggered()), this, SLOT(About()));

  g_quitAct = new QAction(tr("&Quit"), this);
  g_quitAct->setStatusTip(tr("Quit"));
  connect(g_quitAct, SIGNAL(triggered()), this, SLOT(close()));

  g_resetModelsAct = new QAction(tr("&Reset Model Poses"), this);
  g_resetModelsAct->setShortcut(tr("Ctrl+Shift+R"));
  this->addAction(g_resetModelsAct);
  g_resetModelsAct->setStatusTip(tr("Reset model poses"));
  connect(g_resetModelsAct, SIGNAL(triggered()), this,
    SLOT(OnResetModelOnly()));

  g_resetWorldAct = new QAction(tr("&Reset World"), this);
  g_resetWorldAct->setShortcut(tr("Ctrl+R"));
  this->addAction(g_resetWorldAct);
  g_resetWorldAct->setStatusTip(tr("Reset the world"));
  connect(g_resetWorldAct, SIGNAL(triggered()), this, SLOT(OnResetWorld()));

  QActionGroup *editorGroup = new QActionGroup(this);
  // Exclusive doesn't allow all actions to be unchecked at the same time
  editorGroup->setExclusive(false);
  connect(editorGroup, SIGNAL(triggered(QAction *)), this,
      SLOT(OnEditorGroup(QAction *)));

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

  g_editModelAct = new QAction(tr("&Model Editor"), editorGroup);
  g_editModelAct->setShortcut(tr("Ctrl+M"));
  g_editModelAct->setStatusTip(tr("Enter Model Editor Mode"));
  g_editModelAct->setCheckable(true);
  g_editModelAct->setChecked(false);

  g_stepAct = new QAction(QIcon(":/images/end.png"), tr("Step"), this);
  g_stepAct->setStatusTip(tr("Step the world"));
  connect(g_stepAct, SIGNAL(triggered()), this, SLOT(Step()));
  this->CreateDisabledIcon(":/images/end.png", g_stepAct);
  g_stepAct->setEnabled(false);

  g_playAct = new QAction(QIcon(":/images/play.png"), tr("Play"), this);
  g_playAct->setStatusTip(tr("Run the world"));
  g_playAct->setVisible(false);
  connect(g_playAct, SIGNAL(triggered()), this, SLOT(Play()));

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

  g_resetAct = new QAction(tr("Reset View Angle"), this);
  g_resetAct->setStatusTip(tr("Move camera to initial pose"));

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

  g_showOriginAct = new QAction(tr("Origin"), this);
  g_showOriginAct->setStatusTip(tr("Show World Origin"));
  g_showOriginAct->setCheckable(true);
  g_showOriginAct->setChecked(true);
  connect(g_showOriginAct, SIGNAL(triggered()), this,
          SLOT(ShowOrigin()));

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

  g_showCOMAct = new QAction(tr("Center of Mass"), this);
  g_showCOMAct->setStatusTip(tr("Show center of mass"));
  g_showCOMAct->setCheckable(true);
  g_showCOMAct->setChecked(false);
  connect(g_showCOMAct, SIGNAL(triggered()), this,
          SLOT(ShowCOM()));

  g_showInertiaAct = new QAction(tr("Inertias"), this);
  g_showInertiaAct->setStatusTip(tr("Show moments of inertia"));
  g_showInertiaAct->setCheckable(true);
  g_showInertiaAct->setChecked(false);
  connect(g_showInertiaAct, SIGNAL(triggered()), this,
      SLOT(ShowInertia()));

  g_showLinkFrameAct = new QAction(tr("Link Frames"), this);
  g_showLinkFrameAct->setStatusTip(tr("Show link frames"));
  g_showLinkFrameAct->setCheckable(true);
  g_showLinkFrameAct->setChecked(false);
  connect(g_showLinkFrameAct, SIGNAL(triggered()), this,
      SLOT(ShowLinkFrame()));

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

  g_showToolbarsAct = new QAction(tr("Show Toolbars"), this);
  g_showToolbarsAct->setStatusTip(
      tr("Show or hide the top and bottom toolbars"));
  g_showToolbarsAct->setShortcut(tr("Ctrl+H"));
  this->addAction(g_showToolbarsAct);
  g_showToolbarsAct->setCheckable(true);
  g_showToolbarsAct->setChecked(true);
  connect(g_showToolbarsAct, SIGNAL(triggered()), this,
      SLOT(ShowToolbars()));

  g_fullScreenAct = new QAction(tr("Full Screen"), this);
  g_fullScreenAct->setStatusTip(tr("Full Screen (F-11 to exit)"));
  g_fullScreenAct->setShortcut(tr("F11"));
  connect(g_fullScreenAct, SIGNAL(triggered()), this,
      SLOT(FullScreen()));

  g_fpsAct = new QAction(tr("FPS View Control"), this);
  g_fpsAct->setStatusTip(tr("First Person Shooter View Style"));
  g_fpsAct->setCheckable(true);
  g_fpsAct->setChecked(false);
  connect(g_fpsAct, SIGNAL(triggered()), this, SLOT(FPS()));

  g_orbitAct = new QAction(tr("Orbit View Control"), this);
  g_orbitAct->setStatusTip(tr("Orbit View Style"));
  g_orbitAct->setCheckable(true);
  g_orbitAct->setChecked(true);
  connect(g_orbitAct, SIGNAL(triggered()), this, SLOT(Orbit()));

  g_overlayAct = new QAction(tr("Show GUI Overlays"), this);
  g_overlayAct->setStatusTip(tr("Show GUI Overlays"));
  g_overlayAct->setEnabled(false);
  g_overlayAct->setCheckable(true);
  g_overlayAct->setChecked(false);
  connect(g_overlayAct, SIGNAL(triggered()), this, SLOT(ShowGUIOverlays()));

  QActionGroup *viewControlActionGroup = new QActionGroup(this);
  viewControlActionGroup->addAction(g_fpsAct);
  viewControlActionGroup->addAction(g_orbitAct);
  viewControlActionGroup->setExclusive(true);

  g_viewOculusAct = new QAction(tr("Oculus Rift"), this);
  g_viewOculusAct->setStatusTip(tr("Oculus Rift Render Window"));
  connect(g_viewOculusAct, SIGNAL(triggered()), this, SLOT(ViewOculus()));
#ifndef HAVE_OCULUS
  g_viewOculusAct->setEnabled(false);
#endif

  g_cameraOrthoAct = new QAction(tr("Orthographic"), this);
  g_cameraOrthoAct->setStatusTip(tr("Orthographic Projection"));
  g_cameraOrthoAct->setCheckable(true);
  g_cameraOrthoAct->setChecked(false);

  g_cameraPerspectiveAct = new QAction(tr("Perspective"), this);
  g_cameraPerspectiveAct->setStatusTip(tr("Perspective Projection"));
  g_cameraPerspectiveAct->setCheckable(true);
  g_cameraPerspectiveAct->setChecked(true);

  QActionGroup *projectionActionGroup = new QActionGroup(this);
  projectionActionGroup->addAction(g_cameraOrthoAct);
  projectionActionGroup->addAction(g_cameraPerspectiveAct);
  projectionActionGroup->setExclusive(true);

  g_dataLoggerAct = new QAction(QIcon(":images/log_record.png"),
      tr("&Log Data"), this);
  g_dataLoggerAct->setShortcut(tr("Ctrl+D"));
  g_dataLoggerAct->setStatusTip(tr("Data Logging Utility"));
  g_dataLoggerAct->setToolTip(tr("Log Data (Ctrl+D)"));
  g_dataLoggerAct->setCheckable(true);
  g_dataLoggerAct->setChecked(false);
  connect(g_dataLoggerAct, SIGNAL(triggered()), this, SLOT(DataLogger()));

  g_screenshotAct = new QAction(QIcon(":/images/screenshot.png"),
      tr("Screenshot"), this);
  g_screenshotAct->setStatusTip(tr("Take a screenshot"));
  connect(g_screenshotAct, SIGNAL(triggered()), this,
      SLOT(CaptureScreenshot()));

  g_copyAct = new QAction(QIcon(":/images/copy_object.png"),
      tr("Copy (Ctrl + C)"), this);
  g_copyAct->setStatusTip(tr("Copy Entity"));
  g_copyAct->setCheckable(false);
  this->CreateDisabledIcon(":/images/copy_object.png", g_copyAct);
  g_copyAct->setEnabled(false);

  g_pasteAct = new QAction(QIcon(":/images/paste_object.png"),
      tr("Paste (Ctrl + V)"), this);
  g_pasteAct->setStatusTip(tr("Paste Entity"));
  g_pasteAct->setCheckable(false);
  this->CreateDisabledIcon(":/images/paste_object.png", g_pasteAct);
  g_pasteAct->setEnabled(false);

  g_snapAct = new QAction(QIcon(":/images/magnet.png"),
      tr("Snap Mode (N)"), this);
  g_snapAct->setStatusTip(tr("Snap entity"));
  g_snapAct->setCheckable(true);
  g_snapAct->setToolTip(tr("Snap Mode"));
  connect(g_snapAct, SIGNAL(triggered()), this, SLOT(Snap()));

  // set up align actions and widget
  QAction *xAlignMin = new QAction(QIcon(":/images/x_min.png"),
      tr("X Align Min"), this);
  QAction *xAlignCenter = new QAction(QIcon(":/images/x_center.png"),
      tr("X Align Center"), this);
  QAction *xAlignMax = new QAction(QIcon(":/images/x_max.png"),
      tr("X Align Max"), this);
  QAction *yAlignMin = new QAction(QIcon(":/images/y_min.png"),
      tr("Y Align Min"), this);
  QAction *yAlignCenter = new QAction(QIcon(":/images/y_center.png"),
      tr("Y Align Center"), this);
  QAction *yAlignMax = new QAction(QIcon(":/images/y_max.png"),
      tr("Y Align Max"), this);
  QAction *zAlignMin = new QAction(QIcon(":/images/z_min.png"),
      tr("Z Align Min"), this);
  QAction *zAlignCenter = new QAction(QIcon(":/images/z_center.png"),
      tr("Z Align Center"), this);
  QAction *zAlignMax = new QAction(QIcon(":/images/z_max.png"),
      tr("Z Align Max"), this);
  this->CreateDisabledIcon(":/images/x_min.png", xAlignMin);
  this->CreateDisabledIcon(":/images/x_center.png", xAlignCenter);
  this->CreateDisabledIcon(":/images/x_max.png", xAlignMax);
  this->CreateDisabledIcon(":/images/y_min.png", yAlignMin);
  this->CreateDisabledIcon(":/images/y_center.png", yAlignCenter);
  this->CreateDisabledIcon(":/images/y_max.png", yAlignMax);
  this->CreateDisabledIcon(":/images/z_min.png", zAlignMin);
  this->CreateDisabledIcon(":/images/z_center.png", zAlignCenter);
  this->CreateDisabledIcon(":/images/z_max.png", zAlignMax);

  QActionGroup *xAlignActionGroup = new QActionGroup(this);
  xAlignActionGroup->addAction(xAlignMin);
  xAlignActionGroup->addAction(xAlignCenter);
  xAlignActionGroup->addAction(xAlignMax);
  xAlignActionGroup->setExclusive(true);
  QActionGroup *yAlignActionGroup = new QActionGroup(this);
  yAlignActionGroup->addAction(yAlignMin);
  yAlignActionGroup->addAction(yAlignCenter);
  yAlignActionGroup->addAction(yAlignMax);
  yAlignActionGroup->setExclusive(true);
  QActionGroup *zAlignActionGroup = new QActionGroup(this);
  zAlignActionGroup->addAction(zAlignMin);
  zAlignActionGroup->addAction(zAlignCenter);
  zAlignActionGroup->addAction(zAlignMax);
  zAlignActionGroup->setExclusive(true);
  this->alignActionGroups.push_back(xAlignActionGroup);
  this->alignActionGroups.push_back(yAlignActionGroup);
  this->alignActionGroups.push_back(zAlignActionGroup);

  AlignWidget *alignWidget = new AlignWidget(this);
  alignWidget->Add(AlignWidget::ALIGN_X, AlignWidget::ALIGN_MIN, xAlignMin);
  alignWidget->Add(AlignWidget::ALIGN_X, AlignWidget::ALIGN_CENTER,
      xAlignCenter);
  alignWidget->Add(AlignWidget::ALIGN_X, AlignWidget::ALIGN_MAX, xAlignMax);
  alignWidget->Add(AlignWidget::ALIGN_Y, AlignWidget::ALIGN_MIN, yAlignMin);
  alignWidget->Add(AlignWidget::ALIGN_Y, AlignWidget::ALIGN_CENTER,
      yAlignCenter);
  alignWidget->Add(AlignWidget::ALIGN_Y, AlignWidget::ALIGN_MAX, yAlignMax);
  alignWidget->Add(AlignWidget::ALIGN_Z, AlignWidget::ALIGN_MIN, zAlignMin);
  alignWidget->Add(AlignWidget::ALIGN_Z, AlignWidget::ALIGN_CENTER,
      zAlignCenter);
  alignWidget->Add(AlignWidget::ALIGN_Z, AlignWidget::ALIGN_MAX, zAlignMax);
  alignWidget->adjustSize();
  alignWidget->setFixedWidth(alignWidget->width()+5);

  g_alignAct = new QWidgetAction(this);
  g_alignAct->setCheckable(true);
  g_alignAct->setDefaultWidget(alignWidget);
  g_alignAct->setEnabled(false);
  connect(g_alignAct, SIGNAL(triggered()), this, SLOT(Align()));

  // set up view angle actions and widget
  QAction *viewAngleTop = new QAction(QIcon(":/images/view_angle_top.png"),
      tr("View from the top"), this);
  QAction *viewAngleBottom = new QAction(
      QIcon(":/images/view_angle_bottom.png"),
      tr("View from the bottom"), this);
  QAction *viewAngleFront = new QAction(QIcon(":/images/view_angle_front.png"),
      tr("View from the front"), this);
  QAction *viewAngleBack = new QAction(QIcon(":/images/view_angle_back.png"),
      tr("View from the back"), this);
  QAction *viewAngleLeft = new QAction(QIcon(":/images/view_angle_left.png"),
      tr("View from the left"), this);
  QAction *viewAngleRight = new QAction(QIcon(":/images/view_angle_right.png"),
      tr("View from the right"), this);

  // Create another action instead of using g_resetAct here directly because
  // we don't want the icon on the menu.
  QAction *viewAngleReset = new QAction(QIcon(":/images/view_angle_home.png"),
      tr("Reset View Angle"), this);
  connect(g_resetAct, SIGNAL(triggered()), viewAngleReset, SLOT(trigger()));

  ViewAngleWidget *viewAngleWidget = new ViewAngleWidget(this);
  viewAngleWidget->setObjectName("viewAngleWidget");
  viewAngleWidget->Add(ViewAngleWidget::TOP, viewAngleTop);
  viewAngleWidget->Add(ViewAngleWidget::BOTTOM, viewAngleBottom);
  viewAngleWidget->Add(ViewAngleWidget::FRONT, viewAngleFront);
  viewAngleWidget->Add(ViewAngleWidget::BACK, viewAngleBack);
  viewAngleWidget->Add(ViewAngleWidget::LEFT, viewAngleLeft);
  viewAngleWidget->Add(ViewAngleWidget::RIGHT, viewAngleRight);
  viewAngleWidget->Add(ViewAngleWidget::RESET, viewAngleReset);

  g_viewAngleAct = new QWidgetAction(this);
  g_viewAngleAct->setDefaultWidget(viewAngleWidget);
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

    // populate main window's menu bar with menus from normal simulation mode
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
    if (!menuBars.empty())
      newMenuBar = menuBars[0];
  }
  else
  {
    newMenuBar = _bar;
  }

  if (!newMenuBar)
  {
    gzerr << "Unable to set NULL menu bar" << std::endl;
    return;
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
void MainWindow::DeleteActions()
{
  delete g_topicVisAct;
  g_topicVisAct = 0;

  delete g_openAct;
  g_openAct = 0;

  delete g_saveAct;
  g_saveAct = 0;

  delete g_saveAsAct;
  g_saveAsAct = 0;

  delete g_saveCfgAct;
  g_saveCfgAct = 0;

  delete g_cloneAct;
  g_cloneAct = 0;

  delete g_aboutAct;
  g_aboutAct = 0;

  delete g_quitAct;
  g_quitAct = 0;

  delete g_resetModelsAct;
  g_resetModelsAct = 0;

  delete g_resetWorldAct;
  g_resetWorldAct = 0;

  delete g_editBuildingAct;
  g_editBuildingAct = 0;

  delete g_editTerrainAct;
  g_editTerrainAct = 0;

  delete g_editModelAct;
  g_editModelAct = 0;

  delete g_stepAct;
  g_stepAct = 0;

  delete g_playAct;
  g_playAct = 0;

  delete g_pauseAct;
  g_pauseAct = 0;

  delete g_arrowAct;
  g_arrowAct = 0,

  delete g_translateAct;
  g_translateAct = 0;

  delete g_rotateAct;
  g_rotateAct = 0;

  delete g_scaleAct;
  g_scaleAct = 0;

  delete g_boxCreateAct;
  g_boxCreateAct = 0;

  delete g_sphereCreateAct;
  g_sphereCreateAct = 0;

  delete g_cylinderCreateAct;
  g_cylinderCreateAct = 0;

  delete g_pointLghtCreateAct;
  g_pointLghtCreateAct = 0;

  delete g_spotLghtCreateAct;
  g_spotLghtCreateAct = 0;

  delete g_dirLghtCreateAct;
  g_dirLghtCreateAct = 0;

  delete g_resetAct;
  g_resetAct = 0;

  delete g_showCollisionsAct;
  g_showCollisionsAct = 0;

  delete g_showGridAct;
  g_showGridAct = 0;

  delete g_showOriginAct;
  g_showOriginAct = 0;

  delete g_transparentAct;
  g_transparentAct = 0;

  delete g_viewWireframeAct;
  g_viewWireframeAct = 0;

  delete g_showCOMAct;
  g_showCOMAct = 0;

  delete g_showInertiaAct;
  g_showInertiaAct = 0;

  delete g_showLinkFrameAct;
  g_showLinkFrameAct = 0;

  delete g_showContactsAct;
  g_showContactsAct = 0;

  delete g_showJointsAct;
  g_showJointsAct = 0;

  delete g_showToolbarsAct;
  g_showToolbarsAct = 0;

  delete g_fullScreenAct;
  g_fullScreenAct = 0;

  delete g_fpsAct;
  g_fpsAct = 0;

  delete g_orbitAct;
  g_orbitAct = 0;

  delete g_overlayAct;
  g_overlayAct = 0;

  delete g_viewOculusAct;
  g_viewOculusAct = 0;

  delete g_dataLoggerAct;
  g_dataLoggerAct = 0;

  delete g_screenshotAct;
  g_screenshotAct = 0;

  delete g_copyAct;
  g_copyAct = 0;

  delete g_pasteAct;
  g_pasteAct = 0;

  delete g_snapAct;
  g_snapAct = 0;

  delete g_alignAct;
  g_alignAct = 0;

  delete g_cameraOrthoAct;
  g_cameraOrthoAct = 0;

  delete g_cameraPerspectiveAct;
  g_cameraPerspectiveAct = 0;

  delete g_viewAngleAct;
  g_viewAngleAct = 0;
}


/////////////////////////////////////////////////
void MainWindow::CreateMenuBar()
{
  // main window's menu bar
  QMenuBar *bar = QMainWindow::menuBar();

  QMenu *fileMenu = bar->addMenu(tr("&File"));
  // fileMenu->addAction(g_openAct);
  // fileMenu->addAction(g_newAct);
  fileMenu->addAction(g_saveAct);
  fileMenu->addAction(g_saveAsAct);
  fileMenu->addSeparator();
  fileMenu->addAction(g_saveCfgAct);
  fileMenu->addAction(g_cloneAct);
  fileMenu->addSeparator();
  fileMenu->addAction(g_quitAct);

  this->editMenu = bar->addMenu(tr("&Edit"));
  editMenu->addAction(g_resetModelsAct);
  editMenu->addAction(g_resetWorldAct);
  editMenu->addSeparator();
  editMenu->addAction(g_editBuildingAct);
  editMenu->addAction(g_editModelAct);

  // \TODO: Add this back in when implementing the full Terrain Editor spec.
  // editMenu->addAction(g_editTerrainAct);

  QMenu *cameraMenu = bar->addMenu(tr("&Camera"));
  cameraMenu->addAction(g_cameraOrthoAct);
  cameraMenu->addAction(g_cameraPerspectiveAct);
  cameraMenu->addSeparator();
  cameraMenu->addAction(g_fpsAct);
  cameraMenu->addAction(g_orbitAct);
  cameraMenu->addSeparator();
  cameraMenu->addAction(g_resetAct);

  QMenu *viewMenu = bar->addMenu(tr("&View"));
  viewMenu->addAction(g_showGridAct);
  viewMenu->addAction(g_showOriginAct);
  viewMenu->addSeparator();

  viewMenu->addAction(g_transparentAct);
  viewMenu->addAction(g_viewWireframeAct);
  viewMenu->addSeparator();
  viewMenu->addAction(g_showCollisionsAct);
  viewMenu->addAction(g_showJointsAct);
  viewMenu->addAction(g_showCOMAct);
  viewMenu->addAction(g_showInertiaAct);
  viewMenu->addAction(g_showContactsAct);
  viewMenu->addAction(g_showLinkFrameAct);

  QMenu *windowMenu = bar->addMenu(tr("&Window"));
  windowMenu->addAction(g_topicVisAct);
  windowMenu->addSeparator();
  windowMenu->addAction(g_viewOculusAct);
  windowMenu->addSeparator();
  windowMenu->addAction(g_overlayAct);
  windowMenu->addAction(g_showToolbarsAct);
  windowMenu->addAction(g_fullScreenAct);

#ifdef HAVE_QWT
  // windowMenu->addAction(g_diagnosticsAct);
#endif

  bar->addSeparator();

  QMenu *helpMenu = bar->addMenu(tr("&Help"));
  helpMenu->addAction(g_aboutAct);
}

/////////////////////////////////////////////////
void MainWindow::AddMenu(QMenu *_menu)
{
  if (!_menu)
    return;

  // Get the main window's menubar
  // Note: for some reason we can not call menuBar() again,
  // so manually retrieving the menubar from the mainwindow.
  QList<QMenuBar *> menuBars  = this->findChildren<QMenuBar *>();
  if (!menuBars.empty())
  {
    // Note: addMenu(QMenu *) works the first time but when
    // ShowMenuBar() is called more than once which results in menus being
    // re-added, (e.g. when switching between model editor and simulation modes)
    // _menu does not show up in the menu bar.
    // So workaround is to use addMenu(QString)
    QMenu *newMenu = menuBars[0]->addMenu(_menu->title());

    for (auto &menuAct : _menu->actions())
      newMenu->addAction(menuAct);
  }
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
void MainWindow::OnMoveMode(bool _mode)
{
  if (_mode)
  {
    g_boxCreateAct->setChecked(false);
    g_sphereCreateAct->setChecked(false);
    g_cylinderCreateAct->setChecked(false);
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

      cam->SetDefaultPose(cam_pose);
      cam->SetUseSDFPose(true);
    }

    if (_msg->camera().has_view_controller())
    {
      cam->SetViewController(_msg->camera().view_controller());
    }

    if (_msg->camera().has_projection_type())
    {
      cam->SetProjectionType(_msg->camera().projection_type());
      g_cameraOrthoAct->setChecked(true);
      // Disable view control options when in ortho projection
      g_fpsAct->setEnabled(false);
      g_orbitAct->setEnabled(false);
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

  // Store all the plugins for processing
  {
    boost::mutex::scoped_lock lock(this->pluginLoadMutex);
    for (int i = 0; i < _msg->plugin_size(); ++i)
    {
      boost::shared_ptr<msgs::Plugin> pm(new msgs::Plugin(_msg->plugin(i)));
      this->pluginMsgs.push_back(pm);
    }
  }

  // Call the signal to trigger plugin loading in the main thread.
  this->AddPlugins();
}

/////////////////////////////////////////////////
void MainWindow::OnAddPlugins()
{
  boost::mutex::scoped_lock lock(this->pluginLoadMutex);

  // Load all plugins.
  for (std::vector<boost::shared_ptr<msgs::Plugin const> >::iterator iter =
      this->pluginMsgs.begin(); iter != this->pluginMsgs.end(); ++iter)
  {
    // Make sure the filename string is not empty
    if (!(*iter)->filename().empty())
    {
      // Try to create the plugin
      gazebo::GUIPluginPtr plugin = gazebo::GUIPlugin::Create(
          (*iter)->filename(), (*iter)->name());

      if (!plugin)
      {
        gzerr << "Unable to create gui overlay plugin with filename["
          << (*iter)->filename() << "]\n";
      }
      else
      {
        gzlog << "Loaded GUI plugin[" << (*iter)->filename() << "]\n";

        // Attach the plugin to the render widget.
        this->renderWidget->AddPlugin(plugin, msgs::PluginToSDF(**iter));
      }
    }
  }
  this->pluginMsgs.clear();

  g_overlayAct->setChecked(true);
  g_overlayAct->setEnabled(true);
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
void MainWindow::OnLight(ConstLightPtr &_msg)
{
  gui::Events::lightUpdate(*_msg);
}

/////////////////////////////////////////////////
void MainWindow::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  msgs::Scene sceneMsg;

  if (_msg->has_type() && _msg->type() == sceneMsg.GetTypeName())
  {
    sceneMsg.ParseFromString(_msg->serialized_data());

    for (int i = 0; i < sceneMsg.model_size(); ++i)
    {
      this->entities[sceneMsg.model(i).name()] = sceneMsg.model(i).id();

      for (int j = 0; j < sceneMsg.model(i).link_size(); ++j)
      {
        this->entities[sceneMsg.model(i).link(j).name()] =
          sceneMsg.model(i).link(j).id();

        for (int k = 0; k < sceneMsg.model(i).link(j).collision_size(); ++k)
        {
          this->entities[sceneMsg.model(i).link(j).collision(k).name()] =
            sceneMsg.model(i).link(j).collision(k).id();
        }
      }
      gui::Events::modelUpdate(sceneMsg.model(i));
    }

    for (int i = 0; i < sceneMsg.light_size(); ++i)
    {
      gui::Events::lightUpdate(sceneMsg.light(i));
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
    this->requestMsg = msgs::CreateRequest("scene_info");
    this->requestPub->Publish(*this->requestMsg);
  }
  else if (_msg->has_remove() && _msg->remove())
    this->renderWidget->RemoveScene(_msg->world_name());
  else if (_msg->has_cloned())
  {
    if (_msg->cloned())
    {
      gzlog << "Cloned world available at:\n\t" << _msg->cloned_uri()
            << std::endl;
    }
    else
      gzerr << "Error cloning a world" << std::endl;
  }
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
bool MainWindow::IsPaused() const
{
  if (this->renderWidget)
  {
    TimePanel *timePanel = this->renderWidget->GetTimePanel();
    if (timePanel)
      return timePanel->IsPaused();
  }
  return false;
}

/////////////////////////////////////////////////
void MainWindow::CreateEditors()
{
  // Create a Terrain Editor
  this->editors["terrain"] = new TerrainEditor(this);

  // Create a Building Editor
  this->editors["building"] = new BuildingEditor(this);

  // Create a Model Editor
  this->editors["model"] = new ModelEditor(this);
}

/////////////////////////////////////////////////
void MainWindow::CreateDisabledIcon(const std::string &_pixmap, QAction *_act)
{
  QIcon icon = _act->icon();
  QPixmap pixmap(_pixmap.c_str());
  QPixmap disabledPixmap(pixmap.size());
  disabledPixmap.fill(Qt::transparent);
  QPainter p(&disabledPixmap);
  p.setOpacity(0.4);
  p.drawPixmap(0, 0, pixmap);
  icon.addPixmap(disabledPixmap, QIcon::Disabled);
  _act->setIcon(icon);
}

/////////////////////////////////////////////////
void MainWindow::SetLeftPaneVisibility(bool _on)
{
  int leftPane = _on ? MINIMUM_TAB_WIDTH : 0;
  int rightPane = this->splitter->sizes().at(2);

  QList<int> sizes;
  sizes.push_back(leftPane);
  sizes.push_back(this->width() - leftPane - rightPane);
  sizes.push_back(rightPane);

  this->splitter->setSizes(sizes);
}

/////////////////////////////////////////////////
void MainWindow::OnEditorGroup(QAction *_action)
{
  QActionGroup * editorGroup = _action->actionGroup();
  // Manually uncheck all other actions in the group
  for (int i = 0; i < editorGroup->actions().size(); ++i)
  {
    if (editorGroup->actions()[i] != _action)
    {
      editorGroup->actions()[i]->setChecked(false);
    }
  }
}

/////////////////////////////////////////////////
Editor *MainWindow::GetEditor(const std::string &_name) const
{
  auto iter = this->editors.find(_name);
  if (iter != this->editors.end())
    return iter->second;

  return NULL;
}

/////////////////////////////////////////////////
QAction *MainWindow::CloneAction(QAction *_action, QObject *_parent)
{
  if (!_action || !_parent)
  {
    gzwarn << "Missing action or parent. Not cloning action." << std::endl;
    return NULL;
  }

  QAction *actionClone = new QAction(_action->text(), _parent);

  // Copy basic information from original action.
  actionClone->setStatusTip(_action->statusTip());
  actionClone->setCheckable(_action->isCheckable());
  actionClone->setChecked(_action->isChecked());

  // Do not copy shortcut to avoid overlaps. Instead, connect actions.
  // Cloned action will trigger original action, which does the desired effect.
  connect(actionClone, SIGNAL(triggered()), _action, SLOT(trigger()));
  // Then the original action reports its checked state to the cloned action
  // without triggering it circularly.
  connect(_action, SIGNAL(toggled(bool)), actionClone, SLOT(setChecked(bool)));

  return actionClone;
}
