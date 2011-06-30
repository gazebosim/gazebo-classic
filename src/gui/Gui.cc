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
#include "rendering/Rendering.hh"
#include "gui/MainWindow.hh"
#include "gui/Gui.hh"

// These are needed by QT. They need to stay valid during the entire
// lifetime of the application, and argc > 0 and argv must contain one valid
// character string
int g_argc = 1;
char **g_argv;

using namespace gazebo;

const std::string default_config =
"<?xml version='1.0'?>\
<gazebo>\
  <config>\
    <verbosity>4</verbosity>\
    <gui>\
      <size>800 600</size>\
      <pos>0 0</pos>\
    </gui>\
  </config>\
</gazebo>\
";


QApplication *g_app;
gui::MainWindow *g_main_win;

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
}

void gui::run()
{
  g_app->exec();
}

void gui::stop()
{
  g_app->quit();
}

void gui::fini()
{
  rendering::fini();
}
