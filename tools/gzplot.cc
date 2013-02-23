/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <gazebo/gui/qt.h>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/Diagnostics.hh>
#include <gazebo/gazebo_config.h>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  if (!gazebo::load())
  {
    printf("load error\n");
    return -1;
  }

  gazebo::init();
  gazebo::run();

  QApplication *app = new QApplication(_argc, _argv);
  QFile file(":/style.qss");
  file.open(QFile::ReadOnly);
  QString styleSheet = QLatin1String(file.readAll());

  app->setStyleSheet(styleSheet);

  gazebo::gui::Diagnostics *diagnostics = new gazebo::gui::Diagnostics(NULL);
  diagnostics->show();

  if (!gazebo::init())
  {
    gzerr << "Unable to initialize Gazebo\n";
    return -1;
  }

  app->exec();

  gazebo::fini();
}
