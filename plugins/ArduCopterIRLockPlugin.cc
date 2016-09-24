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

#include <memory>
#include <functional>

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Conversions.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/selection_buffer/SelectionBuffer.hh>

#include "plugins/ArduCopterIRLockPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ArduCopterIRLockPlugin)

namespace gazebo
{
  class ArduCopterIRLockPluginPrivate
  {
    /// \brief Pointer to the parent camera sensor
    public: sensors::CameraSensorPtr parentSensor;

    /// \brief Selection buffer used for occlusion detection
    public: std::unique_ptr<rendering::SelectionBuffer> selectionBuffer;

    /// \brief All event connections.
    public: std::vector<event::ConnectionPtr> connections;

    /// \brief A list of fiducials tracked by this camera.
    public: std::vector<std::string> fiducials;

    public: int handle;

    public: struct irlockPacket
            {
              uint64_t timestamp;
              uint16_t num_targets;
              float pos_x;
              float pos_y;
              float size_x;
              float size_y;
            };
  };
}

/////////////////////////////////////////////////
ignition::math::Vector2i GetScreenSpaceCoords(ignition::math::Vector3d _pt,
    gazebo::rendering::CameraPtr _cam)
{
  // Convert from 3D world pos to 2D screen pos
  Ogre::Vector3 pos = _cam->OgreCamera()->getProjectionMatrix() *
      _cam->OgreCamera()->getViewMatrix() *
      gazebo::rendering::Conversions::Convert(_pt);

  ignition::math::Vector2i screenPos;
  screenPos.X() = ((pos.x / 2.0) + 0.5) * _cam->ViewportWidth();
  screenPos.Y() = (1 - ((pos.y / 2.0) + 0.5)) * _cam->ViewportHeight();

  return screenPos;
}

/////////////////////////////////////////////////
ArduCopterIRLockPlugin::ArduCopterIRLockPlugin()
    : SensorPlugin(),
      dataPtr(new ArduCopterIRLockPluginPrivate)
{
  // socket
  this->dataPtr->handle = socket(AF_INET, SOCK_DGRAM /*SOCK_STREAM*/, 0);
  fcntl(this->dataPtr->handle, F_SETFD, FD_CLOEXEC);
  int one = 1;
  setsockopt(this->dataPtr->handle, IPPROTO_TCP, TCP_NODELAY, &one,
      sizeof(one));
  setsockopt(this->dataPtr->handle, SOL_SOCKET, SO_REUSEADDR, &one,
      sizeof(one));
  fcntl(this->dataPtr->handle, F_SETFL,
      fcntl(this->dataPtr->handle, F_GETFL, 0) | O_NONBLOCK);
}

/////////////////////////////////////////////////
ArduCopterIRLockPlugin::~ArduCopterIRLockPlugin()
{
  this->dataPtr->connections.clear();
  this->dataPtr->parentSensor.reset();
}

/////////////////////////////////////////////////
void ArduCopterIRLockPlugin::Load(sensors::SensorPtr _sensor,
                                  sdf::ElementPtr _sdf)
{
  this->dataPtr->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->dataPtr->parentSensor)
  {
    gzerr << "ArduCopterIRLockPlugin not attached to a camera sensor\n";
    return;
  }

  // load the fiducials
  if (_sdf->HasElement("fiducial"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("fiducial");
    while (elem)
    {
      this->dataPtr->fiducials.push_back(elem->Get<std::string>());
      elem = elem->GetNextElement("fiducial");
    }
  }
  else
  {
    gzerr << "No fidicuals specified. ArduCopterIRLockPlugin will not be run."
        << std::endl;
    return;
  }

  this->dataPtr->parentSensor->SetActive(true);

  this->dataPtr->connections.push_back(
      this->dataPtr->parentSensor->Camera()->ConnectNewImageFrame(
      std::bind(&ArduCopterIRLockPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)));
}

/////////////////////////////////////////////////
void ArduCopterIRLockPlugin::OnNewFrame(const unsigned char * /*_image*/,
    unsigned int /*_width*/, unsigned int /*_height*/, unsigned int /*_depth*/,
    const std::string &/*_format*/)
{
  rendering::CameraPtr camera = this->dataPtr->parentSensor->Camera();
  rendering::ScenePtr scene = camera->GetScene();

  if (!this->dataPtr->selectionBuffer)
  {
    std::string cameraName = camera->OgreCamera()->getName();
    this->dataPtr->selectionBuffer.reset(
        new rendering::SelectionBuffer(cameraName, scene->OgreSceneManager(),
        camera->RenderTexture()->getBuffer()->getRenderTarget()));
  }

  for (const auto &f : this->dataPtr->fiducials)
  {
    // check if fiducial is visible within the frustum
    rendering::VisualPtr vis = scene->GetVisual(f);
    if (!vis)
      continue;

    if (!camera->IsVisible(vis))
      continue;

    ignition::math::Vector2i pt = GetScreenSpaceCoords(
        vis->GetWorldPose().pos.Ign(), camera);

    // use selection buffer to check if visual is occluded by other entities
    // in the camera view
    Ogre::Entity *entity =
      this->dataPtr->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

    rendering::VisualPtr result;
    if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
    {
      try
      {
        result = scene->GetVisual(
            Ogre::any_cast<std::string>(
            entity->getUserObjectBindings().getUserAny()));
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
        continue;
      }
    }

    if (result && result->GetRootVisual() == vis)
    {
      this->Publish(vis->GetName(), pt.X(), pt.Y());
    }
  }
}

/////////////////////////////////////////////////
void ArduCopterIRLockPlugin::Publish(const std::string &/*_fiducial*/,
    unsigned int _x, unsigned int _y)
{
  rendering::CameraPtr camera = this->dataPtr->parentSensor->Camera();

  double imageWidth = this->dataPtr->parentSensor->ImageWidth();
  double imageHeight = this->dataPtr->parentSensor->ImageHeight();
  double hfov = camera->HFOV().Radian();
  double vfov = camera->VFOV().Radian();
  double pixelsPerRadianX = imageWidth / hfov;
  double pixelsPerRadianY = imageHeight / vfov;
  float angleX = (static_cast<double>(_x) - (imageWidth * 0.5)) /
      pixelsPerRadianX;
  float angleY = -((imageHeight * 0.5) - static_cast<double>(_y)) /
      pixelsPerRadianY;

  // send_packet
  ArduCopterIRLockPluginPrivate::irlockPacket pkt;

  pkt.timestamp = static_cast<uint64_t>
    (1.0e3*this->dataPtr->parentSensor->LastMeasurementTime().Double());
  pkt.num_targets = static_cast<uint16_t>(1);
  pkt.pos_x = angleX;
  pkt.pos_y = angleY;
  // 1x1 pixel box for now
  pkt.size_x = static_cast<float>(1);
  pkt.size_y = static_cast<float>(1);

  // std::cerr << "fiducial '" << _fiducial << "':" << _x << ", " << _y
  //     << ", pos: " << pkt.pos_x << ", " << pkt.pos_y << std::endl;

  struct sockaddr_in sockaddr;
  memset(&sockaddr, 0, sizeof(sockaddr));
  sockaddr.sin_port = htons(9005);  // TODO: make it variable
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = inet_addr("127.0.0.1");  // TODO: make it variable
  ::sendto(this->dataPtr->handle, &pkt, sizeof(pkt), 0,
    (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}
