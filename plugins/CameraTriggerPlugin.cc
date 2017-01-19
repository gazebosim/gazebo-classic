/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"

#include "plugins/CameraTriggerPlugin.hh"

namespace gazebo
{
  /// \brief Private class for the CameraTriggerPlugin
  class CameraTriggerPluginPrivate
  {
    /// \brief Event triggered when a new image frame is available.
    /// \param[in] _image The raw image buffer
    /// \param[in] _width Width of the image
    /// \param[in] _height Height of the image
    /// \param[in] _depth Depth of the image data
    /// \param[in] _format Format the image data is in
    public: void OnNewFrame(const unsigned char *_image,
                            unsigned int _width, unsigned int _height,
                            unsigned int _depth, const std::string &_format);


    /// \brief PreRender event that is fired within the rendering thread
    public: void PreRender();

    /// \brief Callback each time a key message is received.
    /// \param[in] _msg Keypress message.
    public: void OnKeyPress(ConstAnyPtr &_msg);

    /// \brief Enable / Disable camera updates
    /// \param[in] _enabled True to enable camera update
    public: void SetCameraEnabled(const bool _enabled);

    /// \brief Pointer to the parent camera sensor.
    public: sensors::CameraSensorPtr parentSensor;

    /// \brief Pointer to the rendering camera.
    public: rendering::CameraPtr camera;

    /// \brief Event connection when a new image frame is available.
    public: event::ConnectionPtr newFrameConnection;

    /// \brief Event connection for a preRender event
    public: event::ConnectionPtr preRenderConnection;

    /// \brief Subscribe to keyboard messages.
    public: transport::SubscriberPtr keyboardSub;

    /// \brief Node for communication.
    public: transport::NodePtr node;
  };
}

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraTriggerPlugin)

/////////////////////////////////////////////////
CameraTriggerPlugin::CameraTriggerPlugin()
  : SensorPlugin(), dataPtr(new CameraTriggerPluginPrivate)
{
}

/////////////////////////////////////////////////
CameraTriggerPlugin::~CameraTriggerPlugin()
{
  this->dataPtr->parentSensor.reset();
  this->dataPtr->camera.reset();
}

/////////////////////////////////////////////////
void CameraTriggerPlugin::Load(sensors::SensorPtr _sensor,
    sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->dataPtr->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->dataPtr->parentSensor)
  {
    gzerr << "CameraTriggerPlugin not attached to a camera sensor\n";
    return;
  }

  this->dataPtr->camera = this->dataPtr->parentSensor->Camera();

  // Initialize transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->keyboardSub =
      this->dataPtr->node->Subscribe("~/keyboard/keypress",
      &CameraTriggerPluginPrivate::OnKeyPress, this->dataPtr.get(), true);

  this->dataPtr->newFrameConnection =
      this->dataPtr->camera->ConnectNewImageFrame(
      std::bind(&CameraTriggerPluginPrivate::OnNewFrame, this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // disable camera updates by default
  this->dataPtr->SetCameraEnabled(false);
}

/////////////////////////////////////////////////
void CameraTriggerPluginPrivate::OnNewFrame(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  // we got a new frame, now we can disable camera updates
  this->SetCameraEnabled(false);
}

/////////////////////////////////////////////////
void CameraTriggerPluginPrivate::OnKeyPress(ConstAnyPtr &_msg)
{
  // 'c'
  if (_msg->int_value() == 99)
  {
    // Connect to preRender event. We want to perform any camera sensor
    // operations in the rendering thread
    this->preRenderConnection =
        event::Events::ConnectPreRender(
        std::bind(&CameraTriggerPluginPrivate::PreRender, this));
  }
}

/////////////////////////////////////////////////
void CameraTriggerPluginPrivate::PreRender()
{
  // trigger an update and disconnect from any future preRender event callbacks.
  this->SetCameraEnabled(true);
  this->preRenderConnection.reset();
}

/////////////////////////////////////////////////
void CameraTriggerPluginPrivate::SetCameraEnabled(const bool _enabled)
{
  this->parentSensor->SetActive(_enabled);
  this->parentSensor->SetUpdateRate(_enabled ? 0.0 : IGN_DBL_MIN);
}
