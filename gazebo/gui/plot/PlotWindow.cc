/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/gui/plot/PlotWindow.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the PlotWindow class
    class PlotWindowPrivate
    {
    };
  }
}

/////////////////////////////////////////////////
PlotWindow::PlotWindow(QWidget *_parent)
  : QDialog(_parent),
    dataPtr(new PlotWindowPrivate())
{
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle("Gazebo: Plotting Utility");
  this->setObjectName("PlotWindow");
  this->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
      Qt::WindowCloseButtonHint | Qt::WindowStaysOnTopHint |
      Qt::CustomizeWindowHint);
}

/////////////////////////////////////////////////
PlotWindow::~PlotWindow()
{
}
