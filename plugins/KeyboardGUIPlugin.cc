/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include <algorithm>
#include <mutex>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/Actions.hh>
#include "KeyboardGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(KeyboardGUIPlugin)

/////////////////////////////////////////////////
KeyboardGUIPlugin::KeyboardGUIPlugin()
  : GUIPlugin()
{
  gazebo::gui::MainWindow *mainWindow = gazebo::gui::get_main_window();
  if (mainWindow)
  {
    this->renderWidget = mainWindow->RenderWidget();
    this->renderWidget->installEventFilter(this);
  }
  this->move(0, 0);
  this->resize(0, 0);

  // Initialize transport.
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->keyboardPub =
    this->gzNode->Advertise<msgs::Any>("~/keyboard/keypress");
}

/////////////////////////////////////////////////
KeyboardGUIPlugin::~KeyboardGUIPlugin()
{
}


/////////////////////////////////////////////////
void KeyboardGUIPlugin::OnKeyPress(gazebo::common::KeyEvent _event)
{
  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_INT32);
  msg.set_int_value(_event.text[0]);
  this->keyboardPub->Publish(msg);
}

/////////////////////////////////////////////////
bool KeyboardGUIPlugin::eventFilter(QObject *_obj, QEvent *_event)
{
  // gzdbg << _event->type() << " : " <<  QEvent::KeyPress << "\n";
  // if (_event->type() == QEvent::KeyPress)  // why does this not work?
  if (_event->type() == 51)  /// FIXME TODO
  {
    QKeyEvent *qtKeyEvent = (QKeyEvent *)_event;

    gazebo::common::KeyEvent gazeboKeyEvent;
    gazeboKeyEvent.key = qtKeyEvent->key();
    gazeboKeyEvent.text = qtKeyEvent->text().toStdString();

    this->OnKeyPress(gazeboKeyEvent);
  }
  return QObject::eventFilter(_obj, _event);
}

