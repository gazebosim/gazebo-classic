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

#include <QApplication>

#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/Plugin.hh"
#include "common/CommonTypes.hh"
#include "gui/MainWindow.hh"
#include "gui/Gui.hh"

// These are needed by QT. They need to stay valid during the entire
// lifetime of the application, and argc > 0 and argv must contain one valid
// character string
int g_argc = 1;
char **g_argv;


using namespace gazebo;

std::string g_worldname = "default";
std::vector<ServerPluginPtr> g_plugins;

QApplication *g_app;
gui::MainWindow *g_main_win;
rendering::UserCameraPtr g_active_camera;
bool g_fullscreen = false;

void gui::load()
{
  rendering::load();
  rendering::init();

  g_argv = new char*[g_argc];
  for (int i=0; i < g_argc; i++)
  {
    g_argv[i] = new char[strlen("gazebo")];
    strcpy(g_argv[i], "gazebo");
  }

  g_app = new QApplication(g_argc, g_argv);

  g_main_win = new MainWindow();

  g_main_win->Load();
  g_main_win->resize(1024,768);
}

void gui::init()
{
  g_main_win->show();
  g_main_win->Init();

  for (std::vector<ServerPluginPtr>::iterator iter = g_plugins.begin(); 
       iter != g_plugins.end(); iter++)
  {
    (*iter)->Init();
  }
}

void gui::run()
{
  g_app->exec();
}

void gui::stop()
{
  g_active_camera.reset();
  g_app->quit();
}

void gui::fini()
{
  g_active_camera.reset();
  rendering::fini();
}

void gui::set_world( const std::string& _name)
{
  g_worldname = _name;
}

std::string gui::get_world()
{
  return g_worldname;
}

void gui::set_active_camera( rendering::UserCameraPtr _cam )
{
  g_active_camera = _cam;
}

rendering::UserCameraPtr gui::get_active_camera()
{
  return g_active_camera;
}

void gui::load_plugin( const std::string &_filename )
{
  gazebo::ServerPluginPtr plugin = gazebo::ServerPlugin::Create(_filename, _filename);
  plugin->Load();
  g_plugins.push_back( plugin );
}
