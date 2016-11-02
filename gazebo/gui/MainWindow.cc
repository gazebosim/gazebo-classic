/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <QDesktopServices>
#include <functional>

#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include <boost/algorithm/string.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/gazebo_client.hh"

#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/AlignWidget.hh"
#include "gazebo/gui/CloneWindow.hh"
#include "gazebo/gui/DataLogger.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiPlugin.hh"
#include "gazebo/gui/InsertModelWidget.hh"
#include "gazebo/gui/LayersWidget.hh"
#include "gazebo/gui/ModelListWidget.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/SpaceNav.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/ToolsWidget.hh"
#include "gazebo/gui/TopicSelector.hh"
#include "gazebo/gui/TopToolbar.hh"
#include "gazebo/gui/UserCmdHistory.hh"
#include "gazebo/gui/ViewAngleWidget.hh"
#include "gazebo/gui/plot/PlotWindow.hh"
#include "gazebo/gui/building/BuildingEditor.hh"
#include "gazebo/gui/model/ModelEditor.hh"
#include "gazebo/gui/terrain/TerrainEditor.hh"
#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/TopicView.hh"
#include "gazebo/gui/viewers/ImageView.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/MainWindowPrivate.hh"

#ifdef HAVE_OCULUS
#include "gazebo/gui/OculusWindow.hh"
#endif


using namespace gazebo;
using namespace gui;

#define MINIMUM_TAB_WIDTH 250

extern bool g_fullscreen;

/////////////////////////////////////////////////
MainWindow::MainWindow()
  : dataPtr(new MainWindowPrivate)
{
  this->setObjectName("mainWindow");

  // Do these things first.
  {
    this->CreateActions();
  }

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  gui::set_world(this->dataPtr->node->GetTopicNamespace());

  QWidget *mainWidget = new QWidget;
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainWidget->show();
  this->setCentralWidget(mainWidget);

  this->setDockOptions(QMainWindow::AnimatedDocks);

  this->dataPtr->leftColumn = new QStackedWidget(this);

  this->dataPtr->modelListWidget = new ModelListWidget(this);
  this->dataPtr->insertModel = new InsertModelWidget(this);
  LayersWidget *layersWidget = new LayersWidget(this);

  this->dataPtr->tabWidget = new QTabWidget();
  this->dataPtr->tabWidget->setObjectName("mainTab");
  this->dataPtr->tabWidget->addTab(this->dataPtr->modelListWidget, "World");
  this->dataPtr->tabWidget->addTab(this->dataPtr->insertModel, "Insert");
  this->dataPtr->tabWidget->addTab(layersWidget, "Layers");
  this->dataPtr->tabWidget->setSizePolicy(QSizePolicy::Expanding,
                                 QSizePolicy::Expanding);
  this->dataPtr->tabWidget->setMinimumWidth(MINIMUM_TAB_WIDTH);
  this->AddToLeftColumn("default", this->dataPtr->tabWidget);

  this->dataPtr->toolsWidget = new ToolsWidget();

  this->dataPtr->renderWidget = new gui::RenderWidget(mainWidget);

  this->CreateEditors();

  QHBoxLayout *centerLayout = new QHBoxLayout;

  this->dataPtr->splitter = new QSplitter(this);
  this->dataPtr->splitter->addWidget(this->dataPtr->leftColumn);
  this->dataPtr->splitter->addWidget(this->dataPtr->renderWidget);
  this->dataPtr->splitter->addWidget(this->dataPtr->toolsWidget);
  this->dataPtr->splitter->setContentsMargins(0, 0, 0, 0);

#ifdef _WIN32
  // The splitter appears solid white in Windows, so we make it transparent.
  this->dataPtr->splitter->setStyleSheet(
  "QSplitter { color: #ffffff; background-color: transparent; }"
  "QSplitter::handle { color: #ffffff; background-color: transparent; }");
#endif

  QList<int> sizes;
  sizes.push_back(MINIMUM_TAB_WIDTH);
  sizes.push_back(this->width() - MINIMUM_TAB_WIDTH);
  sizes.push_back(0);
  this->dataPtr->splitter->setSizes(sizes);

  this->dataPtr->splitter->setStretchFactor(0, 0);
  this->dataPtr->splitter->setStretchFactor(1, 2);
  this->dataPtr->splitter->setStretchFactor(2, 0);
  this->dataPtr->splitter->setHandleWidth(10);

  centerLayout->addWidget(this->dataPtr->splitter);
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
  this->dataPtr->oculusWindow = nullptr;
#endif

  this->dataPtr->connections.push_back(
      gui::Events::ConnectLeftPaneVisibility(
        std::bind(&MainWindow::SetLeftPaneVisibility, this,
        std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectFullScreen(
        std::bind(&MainWindow::OnFullScreen, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectShowToolbars(
        std::bind(&MainWindow::OnShowToolbars, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectMoveMode(
        std::bind(&MainWindow::OnMoveMode, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectManipMode(
        std::bind(&MainWindow::OnManipMode, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       std::bind(&MainWindow::OnSetSelectedEntity, this,
       std::placeholders::_1, std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectInputStepSize(
      std::bind(&MainWindow::OnInputStepSizeChanged, this,
      std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectFollow(
        std::bind(&MainWindow::OnFollow, this,
        std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectWindowMode(
      std::bind(&MainWindow::OnWindowMode, this,
      std::placeholders::_1)));

  gui::ViewFactory::RegisterAll();

  // Do these things last
  {
    (void) new QShortcut(Qt::CTRL + Qt::Key_Q, this, SLOT(close()));
    this->CreateMenus();
  }

  // Create a pointer to the space navigator interface
  this->dataPtr->spacenav = new SpaceNav();

  // Use a signal/slot to load plugins. This makes the process thread safe.
  this->connect(this, SIGNAL(AddPlugins()),
          this, SLOT(OnAddPlugins()), Qt::QueuedConnection);

  // Use a signal/slot to track a visual. This makes the process thread safe.
  this->connect(this, SIGNAL(TrackVisual(const std::string &)),
          this, SLOT(OnTrackVisual(const std::string &)), Qt::QueuedConnection);

  // Create data logger dialog
  this->dataPtr->dataLogger = new gui::DataLogger(this);
  this->connect(this->dataPtr->dataLogger, SIGNAL(rejected()), this, SLOT(
    OnDataLoggerClosed()));

  this->show();
}

/////////////////////////////////////////////////
MainWindow::~MainWindow()
{
  delete this->dataPtr->userCmdHistory;
  this->dataPtr->userCmdHistory = nullptr;

  // Cleanup global actions
  this->DeleteActions();
}

/////////////////////////////////////////////////
void MainWindow::Load()
{
  this->dataPtr->guiSub = this->dataPtr->node->Subscribe("~/gui",
    &MainWindow::OnGUI, this, true);
#ifdef HAVE_OCULUS
  int oculusAutoLaunch = getINIProperty<int>("oculus.autolaunch", 0);
  int oculusX = getINIProperty<int>("oculus.x", 0);
  int oculusY = getINIProperty<int>("oculus.y", 0);
  std::string visual = getINIProperty<std::string>("oculus.visual", "");

  if (oculusAutoLaunch == 1)
  {
    if (!visual.empty())
    {
      this->dataPtr->oculusWindow = new gui::OculusWindow(
        oculusX, oculusY, visual);

      if (this->dataPtr->oculusWindow->CreateCamera())
        this->dataPtr->oculusWindow->show();
    }
    else
      gzlog << "Oculus: No visual link specified in for attaching the camera. "
            << "Did you forget to set ~/.gazebo/gui.ini?\n";
  }
#endif

  // Load the space navigator
  if (!this->dataPtr->spacenav->Load())
    gzerr << "Unable to load space navigator\n";
}

/////////////////////////////////////////////////
void MainWindow::Init()
{
  // Get the size properties from the INI file.
  int winWidth = getINIProperty<int>("geometry.width", -1);
  int winHeight = getINIProperty<int>("geometry.height", -1);

  // Width or height were not specified. Therefore make the window
  // maximized.
  if (winWidth <= 0 || winHeight <= 0)
  {
    // Output error if the gui.ini file has missing value.
    if (winWidth > 0)
    {
      gzerr << "gui.ini file has width but not height specified. "
       << "The main window will appear maximized.\n";
    }

    // Output error if the gui.ini file has missing value.
    if (winHeight > 0)
    {
      gzerr << "gui.ini file has height but not width specified. "
       << "The main window will appear maximized.\n";
    }

    this->showMaximized();
  }
  else
  {
    // Get the position properties from the INI file.
    int winXPos = getINIProperty<int>("geometry.x", 0);
    int winYPos = getINIProperty<int>("geometry.y", 0);

    this->setGeometry(winXPos, winYPos, winWidth, winHeight);

    if (this->width() > winWidth)
    {
      gzwarn << "Requested geometry.width of " << winWidth
        << " but the minimum width of the window is "
        << this->width() << "." << std::endl;
    }

    if (this->height() > winHeight)
    {
      gzwarn << "Requested geometry.height of " << winHeight
        << " but the minimum height of the window is "
        << this->height() << "." << std::endl;
    }
  }

  this->dataPtr->worldControlPub =
    this->dataPtr->node->Advertise<msgs::WorldControl>("~/world_control");
  this->dataPtr->serverControlPub =
    this->dataPtr->node->Advertise<msgs::ServerControl>(
      "/gazebo/server/control");
  this->dataPtr->scenePub =
    this->dataPtr->node->Advertise<msgs::Scene>("~/scene");
  this->dataPtr->userCmdPub = this->dataPtr->node->Advertise<msgs::UserCmd>(
    "~/user_cmd");

  this->dataPtr->newEntitySub = this->dataPtr->node->Subscribe("~/model/info",
      &MainWindow::OnModel, this, true);

  // \todo Treating both light topics the same way, this should be improved
  this->dataPtr->lightModifySub = this->dataPtr->node->Subscribe(
    "~/light/modify",
    &MainWindow::OnLight, this);

  this->dataPtr->lightFactorySub = this->dataPtr->node->Subscribe(
    "~/factory/light",
    &MainWindow::OnLight, this);

  this->dataPtr->requestPub =
    this->dataPtr->node->Advertise<msgs::Request>("~/request");
  this->dataPtr->responseSub = this->dataPtr->node->Subscribe("~/response",
      &MainWindow::OnResponse, this);

  this->dataPtr->worldModSub = this->dataPtr->node->Subscribe(
                                            "/gazebo/world/modify",
                                            &MainWindow::OnWorldModify, this);

  this->dataPtr->requestMsg = msgs::CreateRequest("scene_info");
  this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);

  gui::Events::mainWindowReady();
}

/////////////////////////////////////////////////
void MainWindow::closeEvent(QCloseEvent * /*_event*/)
{
  this->dataPtr->renderWidget->hide();
  this->dataPtr->tabWidget->hide();
  this->dataPtr->toolsWidget->hide();

  this->dataPtr->responseSub.reset();
  this->dataPtr->guiSub.reset();
  this->dataPtr->newEntitySub.reset();
  this->dataPtr->worldModSub.reset();
  this->dataPtr->lightModifySub.reset();
  this->dataPtr->lightFactorySub.reset();
  this->dataPtr->worldControlPub.reset();
  this->dataPtr->serverControlPub.reset();
  this->dataPtr->requestPub.reset();
  this->dataPtr->scenePub.reset();
  this->dataPtr->userCmdPub.reset();

  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
  this->dataPtr->pluginMsgs.clear();

  this->dataPtr->node.reset();

  this->dataPtr->connections.clear();

#ifdef HAVE_OCULUS
  if (this->dataPtr->oculusWindow)
  {
    delete this->dataPtr->oculusWindow;
    this->dataPtr->oculusWindow = nullptr;
  }
#endif
  delete this->dataPtr->renderWidget;

  // Cleanup the space navigator
  delete this->dataPtr->spacenav;
  this->dataPtr->spacenav = nullptr;

  emit Close();
}

/////////////////////////////////////////////////
void MainWindow::New()
{
  msgs::ServerControl msg;
  msg.set_new_world(true);
  this->dataPtr->serverControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Plot()
{
  gui::PlotWindow *plot = new gui::PlotWindow(this);
  plot->show();
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
    this->dataPtr->serverControlPub->Publish(msg);
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
    this->dataPtr->saveFilename = filename;
    this->Save();
  }
}

/////////////////////////////////////////////////
void MainWindow::Save()
{
  // Get the latest world in SDF.
  boost::shared_ptr<msgs::Response> response =
    transport::request(get_world(), "world_sdf_save");

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

      cameraElem->GetElement("pose")->Set(cam->WorldPose());
      cameraElem->GetElement("view_controller")->Set(
          cam->GetViewControllerTypeString());

      cameraElem->GetElement("projection_type")->Set(cam->ProjectionType());

      // TODO: export track_visual properties as well.
      msgData = sdf_parsed.Root()->ToString("");
    }
    else
    {
      msgData = msg.data();
      gzerr << "Unable to parse world file to add user camera settings.\n";
    }

    // Open the file
    std::ofstream out(this->dataPtr->saveFilename.c_str(), std::ios::out);

    if (!out)
    {
      QMessageBox msgBox;
      std::string str = "Unable to open file: " + this->dataPtr->saveFilename;
      str += ".\nCheck file permissions.";
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
  std::unique_ptr<CloneWindow> cloneWindow(new CloneWindow(this));
  if (cloneWindow->exec() == QDialog::Accepted && cloneWindow->IsValidPort())
  {
    // Create a gzserver clone in the server side.
    msgs::ServerControl msg;
    msg.set_save_world_name("");
    msg.set_clone(true);
    msg.set_new_port(cloneWindow->Port());
    this->dataPtr->serverControlPub->Publish(msg);
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
void MainWindow::HotkeyChart()
{
  QDesktopServices::openUrl(QUrl("http://gazebosim.org/hotkeys.html"));
}

/////////////////////////////////////////////////
void MainWindow::Play()
{
  msgs::WorldControl msg;
  msg.set_pause(false);

  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Pause()
{
  msgs::WorldControl msg;
  msg.set_pause(true);

  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::Step()
{
  msgs::WorldControl msg;
  msg.set_multi_step(this->dataPtr->inputStepSize);

  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::OnInputStepSizeChanged(int _value)
{
  this->dataPtr->inputStepSize = _value;
}

/////////////////////////////////////////////////
void MainWindow::OnFollow(const std::string &_modelName)
{
  if (_modelName.empty())
  {
    this->dataPtr->renderWidget->DisplayOverlayMsg("", 0);
    this->dataPtr->editMenu->setEnabled(true);
  }
  else
  {
    this->dataPtr->renderWidget->DisplayOverlayMsg(
        "Press Escape to exit Follow mode", 0);
    this->dataPtr->editMenu->setEnabled(false);
  }
}

/////////////////////////////////////////////////
void MainWindow::OnResetModelOnly()
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(false);
  msg.mutable_reset()->set_time_only(false);
  msg.mutable_reset()->set_model_only(true);

  // Register user command on server
  msgs::UserCmd userCmdMsg;
  userCmdMsg.set_description("Reset models");
  userCmdMsg.set_type(msgs::UserCmd::WORLD_CONTROL);
  userCmdMsg.mutable_world_control()->CopyFrom(msg);
  this->dataPtr->userCmdPub->Publish(userCmdMsg);
}

/////////////////////////////////////////////////
void MainWindow::OnResetWorld()
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);

  // Register user command on server
  msgs::UserCmd userCmdMsg;
  userCmdMsg.set_description("Reset world");
  userCmdMsg.set_type(msgs::UserCmd::WORLD_CONTROL);
  userCmdMsg.mutable_world_control()->CopyFrom(msg);
  this->dataPtr->userCmdPub->Publish(userCmdMsg);
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
  for (unsigned int i = 0 ; i < this->dataPtr->alignActionGroups.size(); ++i)
  {
    this->dataPtr->alignActionGroups[i]->setExclusive(false);
    if (this->dataPtr->alignActionGroups[i]->checkedAction())
      this->dataPtr->alignActionGroups[i]->checkedAction()->setChecked(false);
    this->dataPtr->alignActionGroups[i]->setExclusive(true);
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
  this->dataPtr->renderWidget->DisplayOverlayMsg(
      "Screenshot saved in: " + cam->ScreenshotPath(), 2000);
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
    this->dataPtr->leftColumn->hide();
    this->dataPtr->toolsWidget->hide();
    this->dataPtr->menuBar->hide();
    this->setContentsMargins(0, 0, 0, 0);
    this->centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);
  }
  else
  {
    this->showNormal();
    this->dataPtr->leftColumn->show();
    this->dataPtr->toolsWidget->show();
    this->dataPtr->menuBar->show();
  }
  g_fullScreenAct->setChecked(_value);
  g_fullscreen = _value;
}

/////////////////////////////////////////////////
void MainWindow::OnShowToolbars(bool _value)
{
  if (_value)
  {
    this->RenderWidget()->GetTimePanel()->show();
    this->RenderWidget()->GetToolbar()->show();
  }
  else
  {
    this->RenderWidget()->GetTimePanel()->hide();
    this->RenderWidget()->GetToolbar()->hide();
  }
  g_showToolbarsAct->setChecked(_value);
}

/////////////////////////////////////////////////
void MainWindow::ShowCollisions()
{
  if (g_showCollisionsAct->isChecked())
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "show_collision", "all");
  else
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "hide_collision", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowGrid()
{
  msgs::Scene msg;
  msg.set_name(gui::get_world());
  msg.set_grid(g_showGridAct->isChecked());
  this->dataPtr->scenePub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::ShowOrigin()
{
  msgs::Scene msg;
  msg.set_name(gui::get_world());
  msg.set_origin_visual(g_showOriginAct->isChecked());
  this->dataPtr->scenePub->Publish(msg);
}

/////////////////////////////////////////////////
void MainWindow::ShowJoints()
{
  if (g_showJointsAct->isChecked())
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "show_joints", "all");
  else
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "hide_joints", "all");
}

/////////////////////////////////////////////////
void MainWindow::SetTransparent()
{
  if (g_transparentAct->isChecked())
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "set_transparent", "all");
  else
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "set_opaque", "all");
}

/////////////////////////////////////////////////
void MainWindow::SetWireframe()
{
  if (g_viewWireframeAct->isChecked())
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "set_wireframe", "all");
  else
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "set_solid", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowGUIOverlays()
{
  this->RenderWidget()->SetOverlaysVisible(g_overlayAct->isChecked());
}

/////////////////////////////////////////////////
void MainWindow::ShowCOM()
{
  if (g_showCOMAct->isChecked())
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "show_com", "all");
  else
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "hide_com", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowInertia()
{
  if (g_showInertiaAct->isChecked())
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "show_inertia", "all");
  else
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "hide_inertia", "all");
}

/////////////////////////////////////////////////
void MainWindow::ShowLinkFrame()
{
  if (g_showLinkFrameAct->isChecked())
  {
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "show_link_frame", "all");
  }
  else
  {
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "hide_link_frame", "all");
  }
}

/////////////////////////////////////////////////
void MainWindow::ShowSkeleton()
{
  if (g_showSkeletonAct->isChecked())
  {
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "show_skeleton", "all");
  }
  else
  {
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "hide_skeleton", "all");
  }
}

/////////////////////////////////////////////////
void MainWindow::ShowContacts()
{
  if (g_showContactsAct->isChecked())
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
        "show_contact", "all");
  else
    transport::requestNoReply(this->dataPtr->node->GetTopicNamespace(),
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
  if (scene->OculusCameraCount() != 0)
  {
    gzlog << "Oculus camera already exists." << std::endl;
    return;
  }

  int oculusX = getINIProperty<int>("oculus.x", 0);
  int oculusY = getINIProperty<int>("oculus.y", 0);
  std::string visual = getINIProperty<std::string>("oculus.visual", "");

  if (!visual.empty())
  {
    this->dataPtr->oculusWindow = new gui::OculusWindow(
        oculusX, oculusY, visual);

    if (this->dataPtr->oculusWindow->CreateCamera())
      this->dataPtr->oculusWindow->show();
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
    this->dataPtr->dataLogger->show();
  }
  else
  {
    this->dataPtr->dataLogger->close();
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
  this->connect(g_newAct, SIGNAL(triggered()), this, SLOT(New()));
  */

  g_topicVisAct = new QAction(tr("Topic Visualization"), this);
  g_topicVisAct->setShortcut(tr("Ctrl+T"));
  g_topicVisAct->setStatusTip(tr("Select a topic to visualize"));
  this->connect(g_topicVisAct, SIGNAL(triggered()), this, SLOT(SelectTopic()));

  g_plotAct = new QAction(QIcon(":images/graph_line_toolbar.svg"),
      tr("Plot"), this);
  g_plotAct->setShortcut(tr("Ctrl+P"));
  g_plotAct->setToolTip(tr("Create plot (Ctrl+P)"));
  this->connect(g_plotAct, SIGNAL(triggered()), this, SLOT(Plot()));

  g_openAct = new QAction(tr("&Open World"), this);
  g_openAct->setShortcut(tr("Ctrl+O"));
  g_openAct->setStatusTip(tr("Open an world file"));
  this->connect(g_openAct, SIGNAL(triggered()), this, SLOT(Open()));

  g_saveAct = new QAction(tr("&Save World"), this);
  g_saveAct->setShortcut(tr("Ctrl+S"));
  g_saveAct->setStatusTip(tr("Save world"));
  g_saveAct->setEnabled(false);
  this->connect(g_saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  g_saveAsAct = new QAction(tr("Save World &As"), this);
  g_saveAsAct->setShortcut(tr("Ctrl+Shift+S"));
  g_saveAsAct->setStatusTip(tr("Save world to new file"));
  this->connect(g_saveAsAct, SIGNAL(triggered()), this, SLOT(SaveAs()));

  g_saveCfgAct = new QAction(tr("Save &Configuration"), this);
  g_saveCfgAct->setStatusTip(tr("Save GUI configuration"));
  this->connect(g_saveCfgAct, SIGNAL(triggered()), this, SLOT(SaveINI()));

  g_cloneAct = new QAction(tr("Clone World"), this);
  g_cloneAct->setStatusTip(tr("Clone the world"));
  this->connect(g_cloneAct, SIGNAL(triggered()), this, SLOT(Clone()));

  g_hotkeyChartAct = new QAction(tr("&Hotkey Chart"), this);
  g_hotkeyChartAct->setStatusTip(tr("Open hotkey chart in a browser"));
  this->connect(g_hotkeyChartAct, SIGNAL(triggered()), this,
      SLOT(HotkeyChart()));

  g_aboutAct = new QAction(tr("&About"), this);
  g_aboutAct->setStatusTip(tr("Show the about info"));
  this->connect(g_aboutAct, SIGNAL(triggered()), this, SLOT(About()));

  g_quitAct = new QAction(tr("&Quit"), this);
  g_quitAct->setStatusTip(tr("Quit"));
  this->connect(g_quitAct, SIGNAL(triggered()), this, SLOT(close()));

  g_resetModelsAct = new QAction(tr("&Reset Model Poses"), this);
  g_resetModelsAct->setShortcut(tr("Ctrl+Shift+R"));
  this->addAction(g_resetModelsAct);
  g_resetModelsAct->setStatusTip(tr("Reset model poses"));
  this->connect(g_resetModelsAct, SIGNAL(triggered()), this,
    SLOT(OnResetModelOnly()));

  g_resetWorldAct = new QAction(tr("&Reset World"), this);
  g_resetWorldAct->setShortcut(tr("Ctrl+R"));
  this->addAction(g_resetWorldAct);
  g_resetWorldAct->setStatusTip(tr("Reset the world"));
  this->connect(g_resetWorldAct, SIGNAL(triggered()), this,
      SLOT(OnResetWorld()));

  QActionGroup *editorGroup = new QActionGroup(this);
  // Exclusive doesn't allow all actions to be unchecked at the same time
  editorGroup->setExclusive(false);
  this->connect(editorGroup, SIGNAL(triggered(QAction *)), this,
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
  this->connect(g_stepAct, SIGNAL(triggered()), this, SLOT(Step()));
  this->CreateDisabledIcon(":/images/end.png", g_stepAct);
  g_stepAct->setEnabled(false);

  g_playAct = new QAction(QIcon(":/images/play.png"), tr("Play"), this);
  g_playAct->setStatusTip(tr("Run the world"));
  g_playAct->setVisible(false);
  this->connect(g_playAct, SIGNAL(triggered()), this, SLOT(Play()));

  g_pauseAct = new QAction(QIcon(":/images/pause.png"), tr("Pause"), this);
  g_pauseAct->setStatusTip(tr("Pause the world"));
  g_pauseAct->setVisible(true);
  this->connect(g_pauseAct, SIGNAL(triggered()), this, SLOT(Pause()));

  g_arrowAct = new QAction(QIcon(":/images/arrow.png"),
      tr("Selection Mode"), this);
  g_arrowAct->setStatusTip(tr("Move camera"));
  g_arrowAct->setCheckable(true);
  g_arrowAct->setChecked(true);
  g_arrowAct->setToolTip(tr("Selection Mode (Esc)"));
  this->connect(g_arrowAct, SIGNAL(triggered()), this, SLOT(Arrow()));

  g_translateAct = new QAction(QIcon(":/images/translate.png"),
      tr("&Translation Mode"), this);
  g_translateAct->setStatusTip(tr("Translate an object"));
  g_translateAct->setCheckable(true);
  g_translateAct->setChecked(false);
  g_translateAct->setToolTip(tr("Translation Mode (T)"));
  this->connect(g_translateAct, SIGNAL(triggered()), this, SLOT(Translate()));
  this->CreateDisabledIcon(":/images/translate.png", g_translateAct);

  g_rotateAct = new QAction(QIcon(":/images/rotate.png"),
      tr("Rotation Mode"), this);
  g_rotateAct->setStatusTip(tr("Rotate an object"));
  g_rotateAct->setCheckable(true);
  g_rotateAct->setChecked(false);
  g_rotateAct->setToolTip(tr("Rotation Mode (R)"));
  this->connect(g_rotateAct, SIGNAL(triggered()), this, SLOT(Rotate()));
  this->CreateDisabledIcon(":/images/rotate.png", g_rotateAct);

  g_scaleAct = new QAction(QIcon(":/images/scale.png"),
      tr("Scale Mode"), this);
  g_scaleAct->setStatusTip(tr("Scale an object"));
  g_scaleAct->setCheckable(true);
  g_scaleAct->setChecked(false);
  g_scaleAct->setToolTip(tr("Scale Mode (S)"));
  this->connect(g_scaleAct, SIGNAL(triggered()), this, SLOT(Scale()));

  g_boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  g_boxCreateAct->setStatusTip(tr("Create a box"));
  g_boxCreateAct->setCheckable(true);
  this->connect(g_boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));
  this->CreateDisabledIcon(":/images/box.png", g_boxCreateAct);

  g_sphereCreateAct = new QAction(QIcon(":/images/sphere.png"),
      tr("Sphere"), this);
  g_sphereCreateAct->setStatusTip(tr("Create a sphere"));
  g_sphereCreateAct->setCheckable(true);
  this->connect(g_sphereCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSphere()));
  this->CreateDisabledIcon(":/images/sphere.png", g_sphereCreateAct);

  g_cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Cylinder"), this);
  g_cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  g_cylinderCreateAct->setCheckable(true);
  this->connect(g_cylinderCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateCylinder()));
  this->CreateDisabledIcon(":/images/cylinder.png", g_cylinderCreateAct);

  g_pointLghtCreateAct = new QAction(QIcon(":/images/pointlight.png"),
      tr("Point Light"), this);
  g_pointLghtCreateAct->setStatusTip(tr("Create a point light"));
  g_pointLghtCreateAct->setCheckable(true);
  this->connect(g_pointLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreatePointLight()));
  this->CreateDisabledIcon(":/images/pointlight.png", g_pointLghtCreateAct);

  g_spotLghtCreateAct = new QAction(QIcon(":/images/spotlight.png"),
      tr("Spot Light"), this);
  g_spotLghtCreateAct->setStatusTip(tr("Create a spot light"));
  g_spotLghtCreateAct->setCheckable(true);
  this->connect(g_spotLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSpotLight()));
  this->CreateDisabledIcon(":/images/spotlight.png", g_spotLghtCreateAct);

  g_dirLghtCreateAct = new QAction(QIcon(":/images/directionallight.png"),
      tr("Directional Light"), this);
  g_dirLghtCreateAct->setStatusTip(tr("Create a directional light"));
  g_dirLghtCreateAct->setCheckable(true);
  this->connect(g_dirLghtCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateDirectionalLight()));
  this->CreateDisabledIcon(":/images/directionallight.png", g_dirLghtCreateAct);

  g_resetAct = new QAction(tr("Reset View Angle"), this);
  g_resetAct->setStatusTip(tr("Move camera to initial pose"));

  g_showCollisionsAct = new QAction(tr("Collisions"), this);
  g_showCollisionsAct->setStatusTip(tr("Show Collisions"));
  g_showCollisionsAct->setCheckable(true);
  g_showCollisionsAct->setChecked(false);
  this->connect(g_showCollisionsAct, SIGNAL(triggered()), this,
          SLOT(ShowCollisions()));

  g_showGridAct = new QAction(tr("Grid"), this);
  g_showGridAct->setStatusTip(tr("Show Grid"));
  g_showGridAct->setCheckable(true);
  g_showGridAct->setChecked(true);
  this->connect(g_showGridAct, SIGNAL(triggered()), this,
          SLOT(ShowGrid()));

  g_showOriginAct = new QAction(tr("Origin"), this);
  g_showOriginAct->setStatusTip(tr("Show World Origin"));
  g_showOriginAct->setCheckable(true);
  g_showOriginAct->setChecked(true);
  this->connect(g_showOriginAct, SIGNAL(triggered()), this,
          SLOT(ShowOrigin()));

  g_transparentAct = new QAction(tr("Transparent"), this);
  g_transparentAct->setStatusTip(tr("Transparent"));
  g_transparentAct->setCheckable(true);
  g_transparentAct->setChecked(false);
  this->connect(g_transparentAct, SIGNAL(triggered()), this,
          SLOT(SetTransparent()));

  g_viewWireframeAct = new QAction(tr("Wireframe"), this);
  g_viewWireframeAct->setStatusTip(tr("Wireframe"));
  g_viewWireframeAct->setCheckable(true);
  g_viewWireframeAct->setChecked(false);
  this->connect(g_viewWireframeAct, SIGNAL(triggered()), this,
          SLOT(SetWireframe()));

  g_showCOMAct = new QAction(tr("Center of Mass"), this);
  g_showCOMAct->setStatusTip(tr("Show center of mass"));
  g_showCOMAct->setCheckable(true);
  g_showCOMAct->setChecked(false);
  this->connect(g_showCOMAct, SIGNAL(triggered()), this,
          SLOT(ShowCOM()));

  g_showInertiaAct = new QAction(tr("Inertias"), this);
  g_showInertiaAct->setStatusTip(tr("Show moments of inertia"));
  g_showInertiaAct->setCheckable(true);
  g_showInertiaAct->setChecked(false);
  this->connect(g_showInertiaAct, SIGNAL(triggered()), this,
      SLOT(ShowInertia()));

  g_showLinkFrameAct = new QAction(tr("Link Frames"), this);
  g_showLinkFrameAct->setStatusTip(tr("Show link frames"));
  g_showLinkFrameAct->setCheckable(true);
  g_showLinkFrameAct->setChecked(false);
  this->connect(g_showLinkFrameAct, SIGNAL(triggered()), this,
      SLOT(ShowLinkFrame()));

  g_showSkeletonAct = new QAction(tr("Skeletons"), this);
  g_showSkeletonAct->setStatusTip(tr("Show skeletons"));
  g_showSkeletonAct->setCheckable(true);
  g_showSkeletonAct->setChecked(false);
  this->connect(g_showSkeletonAct, SIGNAL(triggered()), this,
      SLOT(ShowSkeleton()));

  g_showContactsAct = new QAction(tr("Contacts"), this);
  g_showContactsAct->setStatusTip(tr("Show Contacts"));
  g_showContactsAct->setCheckable(true);
  g_showContactsAct->setChecked(false);
  this->connect(g_showContactsAct, SIGNAL(triggered()), this,
          SLOT(ShowContacts()));

  g_showJointsAct = new QAction(tr("Joints"), this);
  g_showJointsAct->setStatusTip(tr("Show Joints"));
  g_showJointsAct->setCheckable(true);
  g_showJointsAct->setChecked(false);
  this->connect(g_showJointsAct, SIGNAL(triggered()), this,
          SLOT(ShowJoints()));

  g_showToolbarsAct = new QAction(tr("Show Toolbars"), this);
  g_showToolbarsAct->setStatusTip(
      tr("Show or hide the top and bottom toolbars"));
  g_showToolbarsAct->setShortcut(tr("Ctrl+H"));
  this->addAction(g_showToolbarsAct);
  g_showToolbarsAct->setCheckable(true);
  g_showToolbarsAct->setChecked(true);
  this->connect(g_showToolbarsAct, SIGNAL(triggered()), this,
      SLOT(ShowToolbars()));

  g_fullScreenAct = new QAction(tr("Full Screen"), this);
  g_fullScreenAct->setStatusTip(tr("Full Screen (F-11 to exit)"));
  g_fullScreenAct->setShortcut(tr("F11"));
  this->connect(g_fullScreenAct, SIGNAL(triggered()), this,
      SLOT(FullScreen()));

  g_fpsAct = new QAction(tr("FPS View Control"), this);
  g_fpsAct->setStatusTip(tr("First Person Shooter View Style"));
  g_fpsAct->setCheckable(true);
  g_fpsAct->setChecked(false);
  this->connect(g_fpsAct, SIGNAL(triggered()), this, SLOT(FPS()));

  g_orbitAct = new QAction(tr("Orbit View Control"), this);
  g_orbitAct->setStatusTip(tr("Orbit View Style"));
  g_orbitAct->setCheckable(true);
  g_orbitAct->setChecked(true);
  this->connect(g_orbitAct, SIGNAL(triggered()), this, SLOT(Orbit()));

  g_overlayAct = new QAction(tr("Show GUI Overlays"), this);
  g_overlayAct->setStatusTip(tr("Show GUI Overlays"));
  g_overlayAct->setEnabled(false);
  g_overlayAct->setCheckable(true);
  g_overlayAct->setChecked(false);
  this->connect(g_overlayAct, SIGNAL(triggered()), this,
      SLOT(ShowGUIOverlays()));

  QActionGroup *viewControlActionGroup = new QActionGroup(this);
  viewControlActionGroup->addAction(g_fpsAct);
  viewControlActionGroup->addAction(g_orbitAct);
  viewControlActionGroup->setExclusive(true);

  g_viewOculusAct = new QAction(tr("Oculus Rift"), this);
  g_viewOculusAct->setStatusTip(tr("Oculus Rift Render Window"));
  this->connect(g_viewOculusAct, SIGNAL(triggered()), this, SLOT(ViewOculus()));
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
  g_dataLoggerAct->setToolTip(tr("Log data (Ctrl+D)"));
  g_dataLoggerAct->setCheckable(true);
  g_dataLoggerAct->setChecked(false);
  this->connect(g_dataLoggerAct, SIGNAL(triggered()), this, SLOT(DataLogger()));

  g_screenshotAct = new QAction(QIcon(":/images/screenshot.png"),
      tr("Screenshot"), this);
  g_screenshotAct->setToolTip(tr("Take a screenshot"));
  this->connect(g_screenshotAct, SIGNAL(triggered()), this,
      SLOT(CaptureScreenshot()));

  g_copyAct = new QAction(QIcon(":/images/copy_object.png"),
      tr("Copy"), this);
  g_copyAct->setStatusTip(tr("Copy Entity"));
  g_copyAct->setCheckable(false);
  this->CreateDisabledIcon(":/images/copy_object.png", g_copyAct);
  g_copyAct->setEnabled(false);
  g_copyAct->setShortcut(tr("Ctrl+C"));

  g_pasteAct = new QAction(QIcon(":/images/paste_object.png"),
      tr("Paste"), this);
  g_pasteAct->setStatusTip(tr("Paste Entity"));
  g_pasteAct->setCheckable(false);
  this->CreateDisabledIcon(":/images/paste_object.png", g_pasteAct);
  g_pasteAct->setEnabled(false);
  g_pasteAct->setShortcut(tr("Ctrl+V"));

  g_snapAct = new QAction(QIcon(":/images/magnet.png"),
      tr("Snap Mode (N)"), this);
  g_snapAct->setStatusTip(tr("Snap entity"));
  g_snapAct->setCheckable(true);
  g_snapAct->setToolTip(tr("Snap Mode"));
  this->connect(g_snapAct, SIGNAL(triggered()), this, SLOT(Snap()));

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
  this->dataPtr->alignActionGroups.push_back(xAlignActionGroup);
  this->dataPtr->alignActionGroups.push_back(yAlignActionGroup);
  this->dataPtr->alignActionGroups.push_back(zAlignActionGroup);

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
  this->connect(g_alignAct, SIGNAL(triggered()), this, SLOT(Align()));

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
  this->connect(g_resetAct, SIGNAL(triggered()), viewAngleReset,
      SLOT(trigger()));

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

  // Undo
  g_undoAct = new QAction(QIcon(":/images/undo.png"),
      tr("Undo"), this);
  g_undoAct->setShortcut(tr("Ctrl+Z"));
  g_undoAct->setCheckable(false);
  g_undoAct->setStatusTip(tr("Undo"));
  this->CreateDisabledIcon(":/images/undo.png", g_undoAct);
  g_undoAct->setEnabled(false);

  // Undo history
  g_undoHistoryAct = new QAction(QIcon(":/images/down_spin_arrow.png"),
      tr("Undo history"), this);
  g_undoHistoryAct->setCheckable(false);
  this->CreateDisabledIcon(":/images/down_spin_arrow.png", g_undoHistoryAct);
  g_undoHistoryAct->setEnabled(false);

  // Redo
  g_redoAct = new QAction(QIcon(":/images/redo.png"),
      tr("Redo"), this);
  g_redoAct->setShortcut(tr("Shift+Ctrl+Z"));
  g_redoAct->setCheckable(false);
  g_redoAct->setStatusTip(tr("Redo"));
  this->CreateDisabledIcon(":/images/redo.png", g_redoAct);
  g_redoAct->setEnabled(false);

  // Redo history
  g_redoHistoryAct = new QAction(QIcon(":/images/down_spin_arrow.png"),
      tr("Redo history"), this);
  g_redoHistoryAct->setCheckable(false);
  this->CreateDisabledIcon(":/images/down_spin_arrow.png", g_redoHistoryAct);
  g_redoHistoryAct->setEnabled(false);

  this->dataPtr->userCmdHistory = new UserCmdHistory();
}

/////////////////////////////////////////////////
void MainWindow::ShowMenuBar(QMenuBar *_bar)
{
  if (!this->dataPtr->menuLayout)
    this->dataPtr->menuLayout = new QHBoxLayout;

  // Remove all widgets from the menuLayout
  while (this->dataPtr->menuLayout->takeAt(0) != 0)
  {
  }

  if (!this->dataPtr->menuBar)
  {
    // create the native menu bar
    this->dataPtr->menuBar = new QMenuBar;
    this->dataPtr->menuBar->setSizePolicy(QSizePolicy::Fixed,
      QSizePolicy::Fixed);
    this->setMenuBar(this->dataPtr->menuBar);

    // populate main window's menu bar with menus from normal simulation mode
    this->CreateMenuBar();
  }

  this->dataPtr->menuBar->clear();

  QMenuBar *newMenuBar = nullptr;
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
    gzerr << "Unable to set nullptr menu bar" << std::endl;
    return;
  }

  QList<QMenu *> menus  = newMenuBar->findChildren<QMenu *>();
  for (int i = 0; i < menus.size(); ++i)
  {
    this->dataPtr->menuBar->addMenu(menus[i]);
  }

  this->dataPtr->menuLayout->addWidget(this->dataPtr->menuBar);

  this->dataPtr->menuLayout->addStretch(5);
  this->dataPtr->menuLayout->setContentsMargins(0, 0, 0, 0);
}

/////////////////////////////////////////////////
void MainWindow::DeleteActions()
{
  delete g_topicVisAct;
  g_topicVisAct = nullptr;

  delete g_openAct;
  g_openAct = nullptr;

  delete g_saveAct;
  g_saveAct = nullptr;

  delete g_saveAsAct;
  g_saveAsAct = nullptr;

  delete g_saveCfgAct;
  g_saveCfgAct = nullptr;

  delete g_cloneAct;
  g_cloneAct = nullptr;

  delete g_hotkeyChartAct;
  g_hotkeyChartAct = nullptr;

  delete g_aboutAct;
  g_aboutAct = nullptr;

  delete g_quitAct;
  g_quitAct = nullptr;

  delete g_resetModelsAct;
  g_resetModelsAct = nullptr;

  delete g_resetWorldAct;
  g_resetWorldAct = nullptr;

  delete g_editBuildingAct;
  g_editBuildingAct = nullptr;

  delete g_editTerrainAct;
  g_editTerrainAct = nullptr;

  delete g_editModelAct;
  g_editModelAct = nullptr;

  delete g_stepAct;
  g_stepAct = nullptr;

  delete g_playAct;
  g_playAct = nullptr;

  delete g_pauseAct;
  g_pauseAct = nullptr;

  delete g_arrowAct;
  g_arrowAct = nullptr,

  delete g_translateAct;
  g_translateAct = nullptr;

  delete g_rotateAct;
  g_rotateAct = nullptr;

  delete g_scaleAct;
  g_scaleAct = nullptr;

  delete g_boxCreateAct;
  g_boxCreateAct = nullptr;

  delete g_sphereCreateAct;
  g_sphereCreateAct = nullptr;

  delete g_cylinderCreateAct;
  g_cylinderCreateAct = nullptr;

  delete g_pointLghtCreateAct;
  g_pointLghtCreateAct = nullptr;

  delete g_spotLghtCreateAct;
  g_spotLghtCreateAct = nullptr;

  delete g_dirLghtCreateAct;
  g_dirLghtCreateAct = nullptr;

  delete g_resetAct;
  g_resetAct = nullptr;

  delete g_showCollisionsAct;
  g_showCollisionsAct = nullptr;

  delete g_showGridAct;
  g_showGridAct = nullptr;

  delete g_showOriginAct;
  g_showOriginAct = nullptr;

  delete g_transparentAct;
  g_transparentAct = nullptr;

  delete g_viewWireframeAct;
  g_viewWireframeAct = nullptr;

  delete g_showCOMAct;
  g_showCOMAct = nullptr;

  delete g_showInertiaAct;
  g_showInertiaAct = nullptr;

  delete g_showLinkFrameAct;
  g_showLinkFrameAct = nullptr;

  delete g_showSkeletonAct;
  g_showSkeletonAct = nullptr;

  delete g_showContactsAct;
  g_showContactsAct = nullptr;

  delete g_showJointsAct;
  g_showJointsAct = nullptr;

  delete g_showToolbarsAct;
  g_showToolbarsAct = nullptr;

  delete g_fullScreenAct;
  g_fullScreenAct = nullptr;

  delete g_fpsAct;
  g_fpsAct = nullptr;

  delete g_orbitAct;
  g_orbitAct = nullptr;

  delete g_overlayAct;
  g_overlayAct = nullptr;

  delete g_viewOculusAct;
  g_viewOculusAct = nullptr;

  delete g_dataLoggerAct;
  g_dataLoggerAct = nullptr;

  delete g_screenshotAct;
  g_screenshotAct = nullptr;

  delete g_copyAct;
  g_copyAct = nullptr;

  delete g_pasteAct;
  g_pasteAct = nullptr;

  delete g_snapAct;
  g_snapAct = nullptr;

  delete g_alignAct;
  g_alignAct = nullptr;

  delete g_cameraOrthoAct;
  g_cameraOrthoAct = nullptr;

  delete g_cameraPerspectiveAct;
  g_cameraPerspectiveAct = nullptr;

  delete g_viewAngleAct;
  g_viewAngleAct = nullptr;

  delete g_undoAct;
  g_undoAct = nullptr;

  delete g_undoHistoryAct;
  g_undoHistoryAct = nullptr;

  delete g_redoAct;
  g_redoAct = nullptr;

  delete g_redoHistoryAct;
  g_redoHistoryAct = nullptr;

  delete g_plotAct;
  g_plotAct = nullptr;
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

  this->dataPtr->editMenu = bar->addMenu(tr("&Edit"));
  this->dataPtr->editMenu->addAction(g_undoAct);
  this->dataPtr->editMenu->addAction(g_redoAct);
  this->dataPtr->editMenu->addSeparator();
  this->dataPtr->editMenu->addAction(g_copyAct);
  this->dataPtr->editMenu->addAction(g_pasteAct);
  this->dataPtr->editMenu->addSeparator();
  this->dataPtr->editMenu->addAction(g_resetModelsAct);
  this->dataPtr->editMenu->addAction(g_resetWorldAct);
  this->dataPtr->editMenu->addSeparator();
  this->dataPtr->editMenu->addAction(g_editBuildingAct);
  this->dataPtr->editMenu->addAction(g_editModelAct);


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
  viewMenu->addAction(g_showSkeletonAct);

  QMenu *windowMenu = bar->addMenu(tr("&Window"));
  windowMenu->addAction(g_topicVisAct);
  windowMenu->addSeparator();
  windowMenu->addAction(g_viewOculusAct);
  windowMenu->addSeparator();
  windowMenu->addAction(g_overlayAct);
  windowMenu->addAction(g_showToolbarsAct);
  windowMenu->addAction(g_fullScreenAct);
  windowMenu->addAction(g_plotAct);

  bar->addSeparator();

  QMenu *helpMenu = bar->addMenu(tr("&Help"));
  helpMenu->addAction(g_hotkeyChartAct);
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
  frame->setLayout(this->dataPtr->menuLayout);
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

      auto cam_pose_pos = ignition::math::Vector3d(
        msg_pose.position().x(),
        msg_pose.position().y(),
        msg_pose.position().z());

      auto cam_pose_rot = ignition::math::Quaterniond(
        msg_pose.orientation().w(),
        msg_pose.orientation().x(),
        msg_pose.orientation().y(),
        msg_pose.orientation().z());

      ignition::math::Pose3d cam_pose(cam_pose_pos, cam_pose_rot);

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
      if (_msg->camera().track().has_static_())
        cam->SetTrackIsStatic(_msg->camera().track().static_());

      if (_msg->camera().track().has_use_model_frame())
        cam->SetTrackUseModelFrame(_msg->camera().track().use_model_frame());

      if (_msg->camera().track().has_xyz())
        cam->SetTrackPosition(msgs::ConvertIgn(_msg->camera().track().xyz()));

      if (_msg->camera().track().has_inherit_yaw())
        cam->SetTrackInheritYaw(_msg->camera().track().inherit_yaw());

      if (_msg->camera().track().has_min_dist())
      {
        double minDist = _msg->camera().track().min_dist();
        cam->SetTrackMinDistance(minDist);
      }

      if (_msg->camera().track().has_max_dist())
      {
        double maxDist = _msg->camera().track().max_dist();
        cam->SetTrackMaxDistance(maxDist);
      }

      if (_msg->camera().track().has_name() &&
          _msg->camera().track().name() != "__default__")
      {
        std::string name = _msg->camera().track().name();
        cam->TrackVisual(name);
        // Call the signal to track a visual in the main thread.
        this->TrackVisual(name);
      }
    }
  }

  // Store all the plugins for processing
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->pluginLoadMutex);
    for (int i = 0; i < _msg->plugin_size(); ++i)
    {
      std::shared_ptr<msgs::Plugin> pm(new msgs::Plugin(_msg->plugin(i)));
      this->dataPtr->pluginMsgs.push_back(pm);
    }
  }

  // Call the signal to trigger plugin loading in the main thread.
  this->AddPlugins();
}

/////////////////////////////////////////////////
void MainWindow::OnAddPlugins()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->pluginLoadMutex);

  // Load all plugins.
  for (auto iter = this->dataPtr->pluginMsgs.begin();
      iter != this->dataPtr->pluginMsgs.end(); ++iter)
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
        this->dataPtr->renderWidget->AddPlugin(plugin,
          msgs::PluginToSDF(**iter));
      }
    }
  }
  this->dataPtr->pluginMsgs.clear();

  g_overlayAct->setChecked(true);
  g_overlayAct->setEnabled(true);
}

/////////////////////////////////////////////////
void MainWindow::OnTrackVisual(const std::string &_visualName)
{
  gui::Events::follow(_visualName);
}

/////////////////////////////////////////////////
void MainWindow::OnModel(ConstModelPtr &_msg)
{
  this->dataPtr->entities[_msg->name()] = _msg->id();
  for (int i = 0; i < _msg->link_size(); i++)
  {
    this->dataPtr->entities[_msg->link(i).name()] = _msg->link(i).id();

    for (int j = 0; j < _msg->link(i).collision_size(); j++)
    {
      this->dataPtr->entities[_msg->link(i).collision(j).name()] =
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
  if (!this->dataPtr->requestMsg || _msg->id() !=
    this->dataPtr->requestMsg->id())
    return;

  msgs::Scene sceneMsg;

  if (_msg->has_type() && _msg->type() == sceneMsg.GetTypeName())
  {
    sceneMsg.ParseFromString(_msg->serialized_data());

    for (int i = 0; i < sceneMsg.model_size(); ++i)
    {
      this->dataPtr->entities[sceneMsg.model(i).name()] =
        sceneMsg.model(i).id();

      for (int j = 0; j < sceneMsg.model(i).link_size(); ++j)
      {
        this->dataPtr->entities[sceneMsg.model(i).link(j).name()] =
          sceneMsg.model(i).link(j).id();

        for (int k = 0; k < sceneMsg.model(i).link(j).collision_size(); ++k)
        {
          const auto &entity = sceneMsg.model(i).link(j).collision(k).name();
          this->dataPtr->entities[entity] =
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

  delete this->dataPtr->requestMsg;
  this->dataPtr->requestMsg = nullptr;
}

/////////////////////////////////////////////////
unsigned int MainWindow::EntityId(const std::string &_name)
{
  unsigned int result = 0;

  std::string name = _name;
  boost::replace_first(name, gui::get_world()+"::", "");

  std::map<std::string, unsigned int>::iterator iter;
  iter = this->dataPtr->entities.find(name);
  if (iter != this->dataPtr->entities.end())
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
  iter = this->dataPtr->entities.find(name);

  if (iter != this->dataPtr->entities.end())
    result = true;

  return result;
}

/////////////////////////////////////////////////
void MainWindow::OnWorldModify(ConstWorldModifyPtr &_msg)
{
  if (_msg->has_create() && _msg->create())
  {
    this->dataPtr->renderWidget->CreateScene(_msg->world_name());
    this->dataPtr->requestMsg = msgs::CreateRequest("scene_info");
    this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
  }
  else if (_msg->has_remove() && _msg->remove())
    this->dataPtr->renderWidget->RemoveScene(_msg->world_name());
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
    this->dataPtr->tabWidget->setCurrentIndex(0);
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
  this->dataPtr->leftColumn->addWidget(_widget);
  this->dataPtr->leftColumnStack[_name] = this->dataPtr->leftColumn->count()-1;
}

/////////////////////////////////////////////////
void MainWindow::ShowLeftColumnWidget(const std::string &_name)
{
  std::map<std::string, int>::iterator iter =
    this->dataPtr->leftColumnStack.find(_name);

  if (iter != this->dataPtr->leftColumnStack.end())
    this->dataPtr->leftColumn->setCurrentIndex(iter->second);
  else
    gzerr << "Widget with name[" << _name << "] has not been added to the left"
      << " column stack.\n";
}

/////////////////////////////////////////////////
RenderWidget *MainWindow::RenderWidget() const
{
  return this->dataPtr->renderWidget;
}

/////////////////////////////////////////////////
bool MainWindow::IsPaused() const
{
  if (this->dataPtr->renderWidget)
  {
    TimePanel *timePanel = this->dataPtr->renderWidget->GetTimePanel();
    if (timePanel)
      return timePanel->IsPaused();
  }
  return false;
}

/////////////////////////////////////////////////
void MainWindow::CreateEditors()
{
  // Create a Terrain Editor
  this->dataPtr->editors["terrain"] =
      std::unique_ptr<TerrainEditor>(new TerrainEditor(this));

  // Create a Building Editor
  this->dataPtr->editors["building"] =
      std::unique_ptr<BuildingEditor>(new BuildingEditor(this));

  // Create a Model Editor
  this->dataPtr->editors["model"] =
      std::unique_ptr<ModelEditor>(new ModelEditor(this));
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
  int rightPane = this->dataPtr->splitter->sizes().at(2);

  QList<int> sizes;
  sizes.push_back(leftPane);
  sizes.push_back(this->width() - leftPane - rightPane);
  sizes.push_back(rightPane);

  this->dataPtr->splitter->setSizes(sizes);
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
Editor *MainWindow::Editor(const std::string &_name) const
{
  auto iter = this->dataPtr->editors.find(_name);
  if (iter != this->dataPtr->editors.end())
    return iter->second.get();

  return nullptr;
}

/////////////////////////////////////////////////
QAction *MainWindow::CloneAction(QAction *_action, QObject *_parent)
{
  if (!_action || !_parent)
  {
    gzwarn << "Missing action or parent. Not cloning action." << std::endl;
    return nullptr;
  }

  QAction *actionClone = new QAction(_action->text(), _parent);

  // Copy basic information from original action.
  actionClone->setStatusTip(_action->statusTip());
  actionClone->setCheckable(_action->isCheckable());
  actionClone->setChecked(_action->isChecked());

  // Do not copy shortcut to avoid overlaps. Instead, connect actions.
  // Cloned action will trigger original action, which does the desired effect.
  this->connect(actionClone, SIGNAL(triggered()), _action, SLOT(trigger()));
  // Then the original action reports its checked state to the cloned action
  // without triggering it circularly.
  this->connect(_action, SIGNAL(toggled(bool)), actionClone,
      SLOT(setChecked(bool)));

  return actionClone;
}

/////////////////////////////////////////////////
void MainWindow::OnWindowMode(const std::string &_mode)
{
  bool simulation = _mode == "Simulation";
  bool logPlayback = _mode == "LogPlayback";

  bool simOrLog = simulation || logPlayback;

  // File
  // g_openAct->setVisible(simOrLog);
  g_saveAct->setVisible(simOrLog);
  g_saveAsAct->setVisible(simOrLog);
  g_saveCfgAct->setVisible(simOrLog);
  g_cloneAct->setVisible(simulation);
  g_quitAct->setVisible(simOrLog);

  // Edit
  this->dataPtr->editMenu->menuAction()->setVisible(simulation);
  g_resetModelsAct->setVisible(simulation);
  g_resetWorldAct->setVisible(simulation);
  g_editBuildingAct->setVisible(simulation);
  // g_editTerrainAct->setVisible(simulation);
  g_editModelAct->setVisible(simulation);

  // Camera
  g_cameraOrthoAct->setVisible(simOrLog);
  g_cameraPerspectiveAct->setVisible(simOrLog);
  g_fpsAct->setVisible(simOrLog);
  g_orbitAct->setVisible(simOrLog);
  g_resetAct->setVisible(simOrLog);

  // View
  g_showGridAct->setVisible(simOrLog);
  g_showOriginAct->setVisible(simOrLog);
  g_transparentAct->setVisible(simOrLog);
  g_viewWireframeAct->setVisible(simOrLog);
  g_showCollisionsAct->setVisible(simOrLog);
  g_showCOMAct->setVisible(simOrLog);
  g_showInertiaAct->setVisible(simOrLog);
  g_showLinkFrameAct->setVisible(simOrLog);
  g_showSkeletonAct->setVisible(simOrLog);
  g_showContactsAct->setVisible(simOrLog);
  g_showJointsAct->setVisible(simOrLog);

  // Window
  g_topicVisAct->setVisible(simOrLog);
  g_viewOculusAct->setVisible(simOrLog);
  g_overlayAct->setVisible(simOrLog);
  g_showToolbarsAct->setVisible(simOrLog);
  g_fullScreenAct->setVisible(simOrLog);
  g_plotAct->setVisible(simOrLog);

  // About
  g_hotkeyChartAct->setVisible(simOrLog);
  g_aboutAct->setVisible(simOrLog);

  // Insert
  if (logPlayback)
    this->dataPtr->tabWidget->removeTab(
      this->dataPtr->tabWidget->indexOf(this->dataPtr->insertModel));
  else if (simulation && this->dataPtr->tabWidget->indexOf(
            this->dataPtr->insertModel) == -1)
    this->dataPtr->tabWidget->insertTab(1, this->dataPtr->insertModel,
      "Insert");

  // User commands
  this->dataPtr->userCmdHistory->SetActive(simulation);
}
