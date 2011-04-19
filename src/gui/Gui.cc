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

#include "MainWindow.hh"
#include "Gui.hh"

using namespace gazebo;

QApplication *g_app;
gui::MainWindow *g_main_win;

void gui::load(const std::string &filename)
{
  g_app = new QApplication(0,NULL);
  g_main_win = new MainWindow();

  g_main_win->Load(filename);
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

void gui::quit()
{
  g_app->quit();
}
