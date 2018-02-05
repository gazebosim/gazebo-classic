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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <gazebo/transport/Node.hh>
#include "KeyboardGUIPlugin.hh"

namespace gazebo
{
  /// \brief Private data for the KeyboardGUIPlugin class
  class KeyboardGUIPluginPrivate
  {
    /// \brief Pointer to a node for communication.
    public: transport::NodePtr gzNode;

    /// \brief keyboard publisher.
    public: transport::PublisherPtr keyboardPub;
  };
}

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(KeyboardGUIPlugin)

/////////////////////////////////////////////////
KeyboardGUIPlugin::KeyboardGUIPlugin()
  : GUIPlugin(), dataPtr(new KeyboardGUIPluginPrivate)
{
  gazebo::gui::MainWindow *mainWindow = gazebo::gui::get_main_window();
  if (!mainWindow)
  {
    gzerr << "Couldn't get main window, keyboard events won't be filtered."
        << std::endl;
    return;
  }
  mainWindow->installEventFilter(this);

  // Make this invisible
  this->move(0, 0);
  this->resize(0, 0);

  // Initialize transport.
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init();
  this->dataPtr->keyboardPub =
      this->dataPtr->gzNode->Advertise<msgs::Any>("~/keyboard/keypress");
}

/////////////////////////////////////////////////
KeyboardGUIPlugin::~KeyboardGUIPlugin()
{
  this->dataPtr->keyboardPub.reset();
  this->dataPtr->gzNode->Fini();
}

/////////////////////////////////////////////////
void KeyboardGUIPlugin::OnKeyPress(const gazebo::common::KeyEvent &_event)
{
  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_INT32);
  msg.set_int_value(_event.key);
  this->dataPtr->keyboardPub->Publish(msg);
}

/////////////////////////////////////////////////
bool KeyboardGUIPlugin::eventFilter(QObject *_obj, QEvent *_event)
{
  // Check if there was a keypress in a child
  if (_event->type() == QEvent::ShortcutOverride)
  {
    QKeyEvent *qtKeyEvent = dynamic_cast<QKeyEvent *>(_event);

    gazebo::common::KeyEvent gazeboKeyEvent;
    gazeboKeyEvent.text = qtKeyEvent->text().toStdString();

    // QKeyEvent::key() does not distiguish between lowercase and uppercase.
    // Need to use QKeyEvent::text() to get the unicode character.
    // Special keys (shift, ctrl) will have an empty text field.
    gazeboKeyEvent.key = !gazeboKeyEvent.text.empty() ? gazeboKeyEvent.text[0] :
      qtKeyEvent->key();

    this->OnKeyPress(gazeboKeyEvent);
  }
  return QObject::eventFilter(_obj, _event);
}
