/*
 * Copyright 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "RestUiPlugin.hh"

using namespace gazebo;

/////////////////////////////////////////////////
RestUiPlugin::RestUiPlugin()
: menuTitle("Web service"),
  loginTitle("Web service login"),
  urlLabel("url"),
  defaultUrl("https://"),
  widget(NULL)
{
}

/////////////////////////////////////////////////
void RestUiPlugin::Load(int _argc, char ** _argv)
{
  gzmsg << "RestUiPlugin: cmd line arguments (menu=, title=, label=, url=)\n";
  for (int i = 0; i < _argc; ++i)
  {
    std::string arg = _argv[i];
    if (arg.find("menu=") == 0)
    {
      this->menuTitle = arg.substr(5);
    }
    else if (arg.find("title=") == 0 )
    {
      this->loginTitle = arg.substr(6);
    }
    else if (arg.find("label=") == 0 )
    {
      this->urlLabel = arg.substr(6);
    }
    else if (arg.find("url=") == 0 )
    {
      this->defaultUrl = arg.substr(4);
    }
  }
  gzmsg << "   menu title: " << this->menuTitle  << std::endl;
  gzmsg << "   Login window title: " << this->loginTitle  << std::endl;
  gzmsg << "   Login window label: " << this->urlLabel  << std::endl;
  gzmsg << "   Web servide URL: " << this->defaultUrl  << std::endl;
}

/////////////////////////////////////////////////
void RestUiPlugin::Init()
{
  // Connect to the sensor update event.
  this->connections.push_back(
      gui::Events::ConnectMainWindowReady(
      boost::bind(&RestUiPlugin::OnMainWindowReady, this)));

  this->connections.push_back(
      event::Events::ConnectPreRender(
      boost::bind(&RestUiPlugin::Update, this)));
}

/////////////////////////////////////////////////
void RestUiPlugin::Update()
{
  if (this->widget)
  {
    this->widget->Update();
  }
}

/////////////////////////////////////////////////
void RestUiPlugin::OnMainWindowReady()
{
  // add menu for this plugin
  std::string menuStr("&");
  menuStr += this->menuTitle;
  QMenu *menu = new QMenu(QString(menuStr.c_str()));

  QAction *loginAct = new QAction(QString("&Login"), menu);
  loginAct->setStatusTip(QString("Login to web service"));
  QAction *logoutAct = new QAction(QString("Log&out"), menu);
  logoutAct->setStatusTip(QString("Logout from web service"));
  logoutAct->setEnabled(false);

  gui::MainWindow *mainWindow = gui::get_main_window();
  // create a global widget instance, to act as a global QT object
  // the RestUiPlugin class is not a QT object
  this->widget = new RestUiWidget(mainWindow,
                                  *loginAct,
                                  *logoutAct,
                                  this->menuTitle.c_str(),
                                  this->loginTitle.c_str(),
                                  this->urlLabel.c_str(),
                                  this->defaultUrl.c_str());

  QObject::connect(loginAct, SIGNAL(triggered()),
                   this->widget, SLOT(Login()));
  menu->addAction(loginAct);

  QObject::connect(logoutAct, SIGNAL(triggered()),
                   this->widget, SLOT(Logout()));

  menu->addAction(logoutAct);

  mainWindow->AddMenu(menu);
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(RestUiPlugin)
