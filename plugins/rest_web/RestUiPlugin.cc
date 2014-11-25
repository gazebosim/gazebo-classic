/*
 * Copyright 2014 Open Source Robotics Foundation
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
using namespace std;


RestUiPlugin::RestUiPlugin()
  :widget(NULL)
{
  menuTitle = "Web service";
  loginTitle = "Web service login";
  urlLabel = "url";
  defaultUrl = "https://";
  cout << "RestUiPlugin()" << endl;
}

RestUiPlugin::~RestUiPlugin()
{
  cout << "~RestUiPlugin()" << endl;
}

void RestUiPlugin::Load(int _argc, char ** _argv)
{
  cout << "RestUiPlugin::Load()" << endl;
  cout << "looking for [menu=, title=, label=, url=" << endl;
  cout << _argc << " args" << endl;
  for (int i=0; i < _argc; i++)
  {
    std::string arg = _argv[i]; 
    cout << " " << i << ": " << arg << endl;

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
}

void RestUiPlugin::Init()
{
  // Connect to the sensor update event.
  this->connections.push_back(
      gui::Events::ConnectMainWindowReady(
      boost::bind(&RestUiPlugin::OnMainWindowReady, this)));

  this->connections.push_back(
        event::Events::ConnectPreRender(
        boost::bind(&RestUiPlugin::Update, this)));

  std::cerr << "RestUiPlugin::Init() done" <<  std::endl;

}

void RestUiPlugin::Update()
{
  if(widget) {
    widget->Update();
  }
}

void RestUiPlugin::OnMainWindowReady()
{
  cout << "RestUiPlugin::OnMainWindowReady()" << endl;
  // add menu for this plugin
  std::string menuStr("&");
  menuStr += this->menuTitle;
  QMenu *menu = new QMenu(QString(menuStr.c_str()));
  QAction* loginAct = new QAction(QString("&Login"), menu );
  loginAct->setStatusTip(QString("Login to Mentor 2 Learning Companion"));
  gui::MainWindow *mainWindow = gui::get_main_window();
  // create a global widget instance, to act as a global QT object
  // the RestUiPlugin class is not a QT object
  widget = new RestUiWidget(mainWindow,
                            this->menuTitle.c_str(),
                            this->loginTitle.c_str(),
                            this->urlLabel.c_str(),
                            this->defaultUrl.c_str());
  QObject::connect(loginAct, SIGNAL(triggered()), widget, SLOT(LoginMOOC()));
  menu->addAction(loginAct);
  mainWindow->AddMenu(menu);
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(RestUiPlugin)



