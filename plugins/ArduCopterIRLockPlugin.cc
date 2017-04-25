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

#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  using raw_type = void;
#endif

#if defined(_MSC_VER)
  #include <BaseTsd.h>
  typedef SSIZE_T ssize_t;
#endif

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
  /// \brief Obtains a parameter from sdf.
  /// \param[in] _sdf Pointer to the sdf object.
  /// \param[in] _name Name of the parameter.
  /// \param[out] _param Param Variable to write the parameter to.
  /// \param[in] _default_value Default value, if the parameter not available.
  /// \param[in] _verbose If true, gzerror if the parameter is not available.
  /// \return True if the parameter was found in _sdf, false otherwise.
  template<class T>
  bool getSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
    T &_param, const T &_defaultValue, const bool &_verbose = false)
  {
    if (_sdf->HasElement(_name))
    {
      _param = _sdf->GetElement(_name)->Get<T>();
      return true;
    }

    _param = _defaultValue;
    if (_verbose)
    {
      gzerr << "[ArduPilotPlugin] Please specify a value for parameter ["
        << _name << "].\n";
    }
    return false;
  }

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

    /// \brief Irlock address
    public: std::string irlock_addr;

    /// \brief Irlock port for receiver socket
    public: uint16_t irlock_port;

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
  #ifndef _WIN32
  // Windows does not support FD_CLOEXEC
  fcntl(this->dataPtr->handle, F_SETFD, FD_CLOEXEC);
  #endif
  int one = 1;
  setsockopt(this->dataPtr->handle, IPPROTO_TCP, TCP_NODELAY,
      reinterpret_cast<const char *>(&one), sizeof(one));
  setsockopt(this->dataPtr->handle, SOL_SOCKET, SO_REUSEADDR,
      reinterpret_cast<const char *>(&one), sizeof(one));

  #ifdef _WIN32
  u_long on = 1;
  ioctlsocket(this->dataPtr->handle, FIONBIO,
      reinterpret_cast<u_long FAR *>(&on));
  #else
  fcntl(this->dataPtr->handle, F_SETFL,
      fcntl(this->dataPtr->handle, F_GETFL, 0) | O_NONBLOCK);
  #endif
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
  getSdfParam<std::string>(_sdf, "irlock_addr",
      this->dataPtr->irlock_addr, "127.0.0.1");
  getSdfParam<uint16_t>(_sdf, "irlock_port",
      this->dataPtr->irlock_port, 9005);

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
        vis->WorldPose().Pos(), camera);

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
      this->Publish(vis->Name(), pt.X(), pt.Y());
    }
  }
}

/////////////////////////////////////////////////
void ArduCopterIRLockPlugin::Publish(const std::string &/*_fiducial*/,
    unsigned int _x, unsigned int _y)
{
  rendering::CameraPtr camera = this->dataPtr->parentSensor->Camera();

  const double imageWidth = this->dataPtr->parentSensor->ImageWidth();
  const double imageHeight = this->dataPtr->parentSensor->ImageHeight();
  const double hfov = camera->HFOV().Radian();
  const double vfov = camera->VFOV().Radian();
  const double pixelsPerRadianX = imageWidth / hfov;
  const double pixelsPerRadianY = imageHeight / vfov;
  const float angleX = static_cast<float>(
    (static_cast<double>(_x) - (imageWidth * 0.5)) / pixelsPerRadianX);
  const float angleY = static_cast<float>(
    -((imageHeight * 0.5) - static_cast<double>(_y)) / pixelsPerRadianY);

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
  sockaddr.sin_port = htons(this->dataPtr->irlock_port);
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = inet_addr(this->dataPtr->irlock_addr.c_str());
  ::sendto(this->dataPtr->handle,
           reinterpret_cast<raw_type *>(&pkt),
           sizeof(pkt), 0,
           (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}
