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
#include <functional>
#include <fcntl.h>
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

#include <mutex>
#include <string>
#include <vector>
#include <sdf/sdf.hh>
#include <ignition/math/Filter.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "plugins/ArduPilotPlugin.hh"

#define MAX_MOTORS 255

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ArduPilotPlugin)

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

/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  /// should rename to servo_command here and in ArduPilot SIM_Gazebo.cpp
  float motorSpeed[MAX_MOTORS] = {0.0f};
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduPilot
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];
};

/// \brief Control class
class Control
{
  /// \brief Constructor
  public: Control()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

  /// \brief control id / channel
  public: int channel = 0;

  /// \brief Next command to be applied to the propeller
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: common::PID pid;

  /// \brief Control type. Can be:
  /// VELOCITY control velocity of joint
  /// POSITION control position of joint
  /// EFFORT control effort of joint
  public: std::string type;

  /// \brief Control propeller joint.
  public: std::string jointName;

  /// \brief Control propeller joint.
  public: physics::JointPtr joint;

  /// \brief direction multiplier for this control
  public: double multiplier = 1;

  /// \brief input command offset
  public: double offset = 0;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: ignition::math::OnePole<double> filter;

  public: static double kDefaultRotorVelocitySlowdownSim;
  public: static double kDefaultFrequencyCutoff;
  public: static double kDefaultSamplingRate;
};

double Control::kDefaultRotorVelocitySlowdownSim = 10.0;
double Control::kDefaultFrequencyCutoff = 5.0;
double Control::kDefaultSamplingRate = 0.2;

// Private data class
class gazebo::ArduPilotSocketPrivate
{
  /// \brief constructor
  public: ArduPilotSocketPrivate()
  {
    // initialize socket udp socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    #ifndef _WIN32
    // Windows does not support FD_CLOEXEC
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    #endif
  }

  /// \brief destructor
  public: ~ArduPilotSocketPrivate()
  {
    if (fd != -1)
    {
      ::close(fd);
      fd = -1;
    }
  }

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Connect to an adress and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.
  public : bool Connect(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  public: void MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr)
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  public: ssize_t Send(const void *_buf, size_t _size)
  {
    return send(this->fd, _buf, _size, 0);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->fd, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->fd+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->fd, reinterpret_cast<char *>(_buf), _size, 0);
    #else
    return recv(this->fd, _buf, _size, 0);
    #endif
  }

  /// \brief Socket handle
  private: int fd;
};

// Private data class
class gazebo::ArduPilotPluginPrivate
{
  /// \brief Pointer to the update event connection.
  public: event::ConnectionPtr updateConnection;

  /// \brief Pointer to the model;
  public: physics::ModelPtr model;

  /// \brief String of the model name;
  public: std::string modelName;

  /// \brief array of propellers
  public: std::vector<Control> controls;

  /// \brief keep track of controller update sim-time.
  public: gazebo::common::Time lastControllerUpdateTime;

  /// \brief Controller update mutex.
  public: std::mutex mutex;

  /// \brief Ardupilot Socket for receive motor command on gazebo
  public: ArduPilotSocketPrivate socket_in;

  /// \brief Ardupilot Socket to send state to Ardupilot
  public: ArduPilotSocketPrivate socket_out;

  /// \brief Ardupilot address
  public: std::string fdm_addr;

  /// \brief The Ardupilot listen address
  public: std::string listen_addr;

  /// \brief Ardupilot port for receiver socket
  public: uint16_t fdm_port_in;

  /// \brief Ardupilot port for sender socket
  public: uint16_t fdm_port_out;

  /// \brief Pointer to an IMU sensor
  public: sensors::ImuSensorPtr imuSensor;

  /// \brief false before ardupilot controller is online
  /// to allow gazebo to continue without waiting
  public: bool arduPilotOnline;

  /// \brief number of times ArduCotper skips update
  public: int connectionTimeoutCount;

  /// \brief number of times ArduCotper skips update
  /// before marking ArduPilot offline
  public: int connectionTimeoutMaxCount;
};

/////////////////////////////////////////////////
ArduPilotPlugin::ArduPilotPlugin()
  : dataPtr(new ArduPilotPluginPrivate)
{
  this->dataPtr->arduPilotOnline = false;
  this->dataPtr->connectionTimeoutCount = 0;
}

/////////////////////////////////////////////////
ArduPilotPlugin::~ArduPilotPlugin()
{
}

/////////////////////////////////////////////////
void ArduPilotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ArduPilotPlugin _model pointer is null");
  GZ_ASSERT(_sdf, "ArduPilotPlugin _sdf pointer is null");

  this->dataPtr->model = _model;
  this->dataPtr->modelName = this->dataPtr->model->GetName();

  // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
  // x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->modelXYZToAirplaneXForwardZDown =
    ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
  if (_sdf->HasElement("modelXYZToAirplaneXForwardZDown"))
  {
    this->modelXYZToAirplaneXForwardZDown =
        _sdf->Get<ignition::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
  }

  // gazeboXYZToNED: from gazebo model frame: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->gazeboXYZToNED = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
  if (_sdf->HasElement("gazeboXYZToNED"))
  {
    this->gazeboXYZToNED = _sdf->Get<ignition::math::Pose3d>("gazeboXYZToNED");
  }

  // per control channel
  sdf::ElementPtr controlSDF;
  if (_sdf->HasElement("control"))
  {
    controlSDF = _sdf->GetElement("control");
  }
  else if (_sdf->HasElement("rotor"))
  {
    gzwarn << "[" << this->dataPtr->modelName << "] "
           << "please deprecate <rotor> block, use <control> block instead.\n";
    controlSDF = _sdf->GetElement("rotor");
  }

  while (controlSDF)
  {
    Control control;

    if (controlSDF->HasAttribute("channel"))
    {
      control.channel =
        atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
    }
    else if (controlSDF->HasAttribute("id"))
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             <<  "please deprecate attribute id, use channel instead.\n";
      control.channel =
        atoi(controlSDF->GetAttribute("id")->GetAsString().c_str());
    }
    else
    {
      control.channel = this->dataPtr->controls.size();
      gzwarn << "[" << this->dataPtr->modelName << "] "
             <<  "id/channel attribute not specified, use order parsed ["
             << control.channel << "].\n";
    }

    if (controlSDF->HasElement("type"))
    {
      control.type = controlSDF->Get<std::string>("type");
    }
    else
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            <<  "Control type not specified,"
            << " using velocity control by default.\n";
      control.type = "VELOCITY";
    }

    if (control.type != "VELOCITY" &&
        control.type != "POSITION" &&
        control.type != "EFFORT")
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "Control type [" << control.type
             << "] not recognized, must be one of VELOCITY, POSITION, EFFORT."
             << " default to VELOCITY.\n";
      control.type = "VELOCITY";
    }

    if (controlSDF->HasElement("jointName"))
    {
      control.jointName = controlSDF->Get<std::string>("jointName");
    }
    else
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "Please specify a jointName,"
            << " where the control channel is attached.\n";
    }

    // Get the pointer to the joint.
    control.joint = _model->GetJoint(control.jointName);
    if (control.joint == nullptr)
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "Couldn't find specified joint ["
            << control.jointName << "]. This plugin will not run.\n";
      return;
    }

    if (controlSDF->HasElement("multiplier"))
    {
      // overwrite turningDirection, deprecated.
      control.multiplier = controlSDF->Get<double>("multiplier");
    }
    else if (controlSDF->HasElement("turningDirection"))
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "<turningDirection> is deprecated. Please use"
             << " <multiplier>. Map 'cw' to '-1' and 'ccw' to '1'.\n";
      std::string turningDirection = controlSDF->Get<std::string>(
          "turningDirection");
      // special cases mimic from controls_gazebo_plugins
      if (turningDirection == "cw")
      {
        control.multiplier = -1;
      }
      else if (turningDirection == "ccw")
      {
        control.multiplier = 1;
      }
      else
      {
        gzdbg << "[" << this->dataPtr->modelName << "] "
              << "not string, check turningDirection as float\n";
        control.multiplier = controlSDF->Get<double>("turningDirection");
      }
    }
    else
    {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "<multiplier> (or deprecated <turningDirection>) not specified,"
            << " Default 1 (or deprecated <turningDirection> 'ccw').\n";
      control.multiplier = 1;
    }

    if (controlSDF->HasElement("offset"))
    {
      control.offset = controlSDF->Get<double>("offset");
    }
    else
    {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "<offset> not specified, default to 0.\n";
      control.offset = 0;
    }

    getSdfParam<double>(controlSDF, "rotorVelocitySlowdownSim",
        control.rotorVelocitySlowdownSim, 1);

    if (ignition::math::equal(control.rotorVelocitySlowdownSim, 0.0))
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "control for joint [" << control.jointName
             << "] rotorVelocitySlowdownSim is zero,"
             << " assume no slowdown.\n";
      control.rotorVelocitySlowdownSim = 1.0;
    }

    getSdfParam<double>(controlSDF, "frequencyCutoff",
        control.frequencyCutoff, control.frequencyCutoff);
    getSdfParam<double>(controlSDF, "samplingRate",
        control.samplingRate, control.samplingRate);

    // use gazebo::math::Filter
    control.filter.Fc(control.frequencyCutoff, control.samplingRate);

    // initialize filter to zero value
    control.filter.Set(0.0);

    // note to use this filter, do
    // stateFiltered = filter.Process(stateRaw);

    // Overload the PID parameters if they are available.
    double param;
    // carry over from ArduCopter plugin
    getSdfParam<double>(controlSDF, "vel_p_gain", param,
      control.pid.GetPGain());
    control.pid.SetPGain(param);

    getSdfParam<double>(controlSDF, "vel_i_gain", param,
      control.pid.GetIGain());
    control.pid.SetIGain(param);

    getSdfParam<double>(controlSDF, "vel_d_gain", param,
       control.pid.GetDGain());
    control.pid.SetDGain(param);

    getSdfParam<double>(controlSDF, "vel_i_max", param,
      control.pid.GetIMax());
    control.pid.SetIMax(param);

    getSdfParam<double>(controlSDF, "vel_i_min", param,
      control.pid.GetIMin());
    control.pid.SetIMin(param);

    getSdfParam<double>(controlSDF, "vel_cmd_max", param,
        control.pid.GetCmdMax());
    control.pid.SetCmdMax(param);

    getSdfParam<double>(controlSDF, "vel_cmd_min", param,
        control.pid.GetCmdMin());
    control.pid.SetCmdMin(param);

    // new params, overwrite old params if exist
    getSdfParam<double>(controlSDF, "p_gain", param,
      control.pid.GetPGain());
    control.pid.SetPGain(param);

    getSdfParam<double>(controlSDF, "i_gain", param,
      control.pid.GetIGain());
    control.pid.SetIGain(param);

    getSdfParam<double>(controlSDF, "d_gain", param,
       control.pid.GetDGain());
    control.pid.SetDGain(param);

    getSdfParam<double>(controlSDF, "i_max", param,
      control.pid.GetIMax());
    control.pid.SetIMax(param);

    getSdfParam<double>(controlSDF, "i_min", param,
      control.pid.GetIMin());
    control.pid.SetIMin(param);

    getSdfParam<double>(controlSDF, "cmd_max", param,
        control.pid.GetCmdMax());
    control.pid.SetCmdMax(param);

    getSdfParam<double>(controlSDF, "cmd_min", param,
        control.pid.GetCmdMin());
    control.pid.SetCmdMin(param);

    // set pid initial command
    control.pid.SetCmd(0.0);

    this->dataPtr->controls.push_back(control);
    controlSDF = controlSDF->GetNextElement("control");
  }

  // Get sensors
  std::string imuName;
  getSdfParam<std::string>(_sdf, "imuName", imuName, "imu_sensor");
  // std::string imuScopedName = this->dataPtr->model->GetWorld()->Name()
  //     + "::" + this->dataPtr->model->GetScopedName()
  //     + "::" + imuName;
  std::vector<std::string> imuScopedName =
    this->dataPtr->model->SensorScopedName(imuName);
  if (imuScopedName.size() > 1)
  {
    gzwarn << "[" << this->dataPtr->modelName << "] "
           << "multiple names match [" << imuName << "] using first found"
           << " name.\n";
    for (unsigned k = 0; k < imuScopedName.size(); ++k)
    {
      gzwarn << "  sensor " << k << " [" << imuScopedName[k] << "].\n";
    }
  }

  if (imuScopedName.size() > 0)
  {
    this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(imuScopedName[0]));
  }

  if (!this->dataPtr->imuSensor)
  {
    if (imuScopedName.size() > 1)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "first imu_sensor scoped name [" << imuScopedName[0]
             << "] not found, trying the rest of the sensor names.\n";
      for (unsigned k = 1; k < imuScopedName.size(); ++k)
      {
        this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
          (sensors::SensorManager::Instance()->GetSensor(imuScopedName[k]));
        if (this->dataPtr->imuSensor)
        {
          gzwarn << "found [" << imuScopedName[k] << "]\n";
          break;
        }
      }
    }

    if (!this->dataPtr->imuSensor)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "imu_sensor scoped name [" << imuName
             << "] not found, trying unscoped name.\n" << "\n";
      // TODO: this fails for multi-nested models.
      // TODO: and transforms fail for rotated nested model,
      //       joints point the wrong way.
      this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
        (sensors::SensorManager::Instance()->GetSensor(imuName));
    }

    if (!this->dataPtr->imuSensor)
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "imu_sensor [" << imuName
            << "] not found, abort ArduPilot plugin.\n" << "\n";
      return;
    }
  }

  // Controller time control.
  this->dataPtr->lastControllerUpdateTime = 0;

  // Initialise ardupilot sockets
  if (!InitArduPilotSockets(_sdf))
  {
    return;
  }

  // Missed update count before we declare arduPilotOnline status false
  getSdfParam<int>(_sdf, "connectionTimeoutMaxCount",
    this->dataPtr->connectionTimeoutMaxCount, 10);

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ArduPilotPlugin::OnUpdate, this));

  gzlog << "[" << this->dataPtr->modelName << "] "
        << "ArduPilot ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void ArduPilotPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  const gazebo::common::Time curTime =
    this->dataPtr->model->GetWorld()->SimTime();

  // Update the control surfaces and publish the new state.
  if (curTime > this->dataPtr->lastControllerUpdateTime)
  {
    this->ReceiveMotorCommand();
    if (this->dataPtr->arduPilotOnline)
    {
      this->ApplyMotorForces((curTime -
        this->dataPtr->lastControllerUpdateTime).Double());
      this->SendState();
    }
  }

  this->dataPtr->lastControllerUpdateTime = curTime;
}

/////////////////////////////////////////////////
void ArduPilotPlugin::ResetPIDs()
{
  // Reset velocity PID for controls
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    this->dataPtr->controls[i].cmd = 0;
    // this->dataPtr->controls[i].pid.Reset();
  }
}

/////////////////////////////////////////////////
bool ArduPilotPlugin::InitArduPilotSockets(sdf::ElementPtr _sdf) const
{
  getSdfParam<std::string>(_sdf, "fdm_addr",
      this->dataPtr->fdm_addr, "127.0.0.1");
  getSdfParam<std::string>(_sdf, "listen_addr",
      this->dataPtr->listen_addr, "127.0.0.1");
  getSdfParam<uint16_t>(_sdf, "fdm_port_in",
      this->dataPtr->fdm_port_in, 9002);
  getSdfParam<uint16_t>(_sdf, "fdm_port_out",
      this->dataPtr->fdm_port_out, 9003);

  if (!this->dataPtr->socket_in.Bind(this->dataPtr->listen_addr.c_str(),
      this->dataPtr->fdm_port_in))
  {
    gzerr << "[" << this->dataPtr->modelName << "] "
          << "failed to bind with " << this->dataPtr->listen_addr
          << ":" << this->dataPtr->fdm_port_in << " aborting plugin.\n";
    return false;
  }

  if (!this->dataPtr->socket_out.Connect(this->dataPtr->fdm_addr.c_str(),
      this->dataPtr->fdm_port_out))
  {
    gzerr << "[" << this->dataPtr->modelName << "] "
          << "failed to bind with " << this->dataPtr->fdm_addr
          << ":" << this->dataPtr->fdm_port_out << " aborting plugin.\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void ArduPilotPlugin::ApplyMotorForces(const double _dt)
{
  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    if (this->dataPtr->controls[i].type == "VELOCITY")
    {
      const double velTarget = this->dataPtr->controls[i].cmd /
        this->dataPtr->controls[i].rotorVelocitySlowdownSim;
      const double vel = this->dataPtr->controls[i].joint->GetVelocity(0);
      const double error = vel - velTarget;
      const double force = this->dataPtr->controls[i].pid.Update(error, _dt);
      this->dataPtr->controls[i].joint->SetForce(0, force);
    }
    else if (this->dataPtr->controls[i].type == "POSITION")
    {
      const double posTarget = this->dataPtr->controls[i].cmd;
      const double pos = this->dataPtr->controls[i].joint->Position();
      const double error = pos - posTarget;
      const double force = this->dataPtr->controls[i].pid.Update(error, _dt);
      this->dataPtr->controls[i].joint->SetForce(0, force);
    }
    else if (this->dataPtr->controls[i].type == "EFFORT")
    {
      const double force = this->dataPtr->controls[i].cmd;
      this->dataPtr->controls[i].joint->SetForce(0, force);
    }
    else
    {
      // do nothing
    }
  }
}

/////////////////////////////////////////////////
void ArduPilotPlugin::ReceiveMotorCommand()
{
  // Added detection for whether ArduPilot is online or not.
  // If ArduPilot is detected (receive of fdm packet from someone),
  // then socket receive wait time is increased from 1ms to 1 sec
  // to accomodate network jitter.
  // If ArduPilot is not detected, receive call blocks for 1ms
  // on each call.
  // Once ArduPilot presence is detected, it takes this many
  // missed receives before declaring the FCS offline.

  ServoPacket pkt;
  uint32_t waitMs = 1;
  if (this->dataPtr->arduPilotOnline)
  {
    // increase timeout for receive once we detect a packet from
    // ArduPilot FCS.
    waitMs = 1000;
  }

  ssize_t recvSize =
    this->dataPtr->socket_in.Recv(&pkt, sizeof(ServoPacket), waitMs);

  // Drain the socket in the case we're backed up
  int counter = 0;
  ServoPacket last_pkt;
  while (true)
  {
    // last_pkt = pkt;
    const ssize_t recvSize_last =
      this->dataPtr->socket_in.Recv(&last_pkt, sizeof(ServoPacket), 0ul);
    if (recvSize_last == -1)
    {
      break;
    }
    counter++;
    pkt = last_pkt;
    recvSize = recvSize_last;
  }
  if (counter > 0)
  {
    gzdbg << "[" << this->dataPtr->modelName << "] "
          << "Drained n packets: " << counter << std::endl;
  }

  if (recvSize == -1)
  {
    // didn't receive a packet
    // gzdbg << "no packet\n";
    gazebo::common::Time::NSleep(100);
    if (this->dataPtr->arduPilotOnline)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "Broken ArduPilot connection, count ["
             << this->dataPtr->connectionTimeoutCount
             << "/" << this->dataPtr->connectionTimeoutMaxCount
             << "]\n";
      if (++this->dataPtr->connectionTimeoutCount >
        this->dataPtr->connectionTimeoutMaxCount)
      {
        this->dataPtr->connectionTimeoutCount = 0;
        this->dataPtr->arduPilotOnline = false;
        gzwarn << "[" << this->dataPtr->modelName << "] "
               << "Broken ArduPilot connection, resetting motor control.\n";
        this->ResetPIDs();
      }
    }
  }
  else
  {
    const ssize_t expectedPktSize =
    sizeof(pkt.motorSpeed[0]) * this->dataPtr->controls.size();
    if (recvSize < expectedPktSize)
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "got less than model needs. Got: " << recvSize
            << "commands, expected size: " << expectedPktSize << "\n";
    }
    const ssize_t recvChannels = recvSize / sizeof(pkt.motorSpeed[0]);
    // for(unsigned int i = 0; i < recvChannels; ++i)
    // {
    //   gzdbg << "servo_command [" << i << "]: " << pkt.motorSpeed[i] << "\n";
    // }

    if (!this->dataPtr->arduPilotOnline)
    {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "ArduPilot controller online detected.\n";
      // made connection, set some flags
      this->dataPtr->connectionTimeoutCount = 0;
      this->dataPtr->arduPilotOnline = true;
    }

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->dataPtr->controls.size(); ++i)
    {
      if (i < MAX_MOTORS)
      {
        if (this->dataPtr->controls[i].channel < recvChannels)
        {
          // bound incoming cmd between 0 and 1
          const double cmd = ignition::math::clamp(
            pkt.motorSpeed[this->dataPtr->controls[i].channel],
            -1.0f, 1.0f);
          this->dataPtr->controls[i].cmd =
            this->dataPtr->controls[i].multiplier *
            (this->dataPtr->controls[i].offset + cmd);
          // gzdbg << "apply input chan[" << this->dataPtr->controls[i].channel
          //       << "] to control chan[" << i
          //       << "] with joint name ["
          //       << this->dataPtr->controls[i].jointName
          //       << "] raw cmd ["
          //       << pkt.motorSpeed[this->dataPtr->controls[i].channel]
          //       << "] adjusted cmd [" << this->dataPtr->controls[i].cmd
          //       << "].\n";
        }
        else
        {
          gzerr << "[" << this->dataPtr->modelName << "] "
                << "control[" << i << "] channel ["
                << this->dataPtr->controls[i].channel
                << "] is greater than incoming commands size["
                << recvChannels
                << "], control not applied.\n";
        }
      }
      else
      {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "too many motors, skipping [" << i
              << " > " << MAX_MOTORS << "].\n";
      }
    }
  }
}

/////////////////////////////////////////////////
void ArduPilotPlugin::SendState() const
{
  // send_fdm
  fdmPacket pkt;

  pkt.timestamp = this->dataPtr->model->GetWorld()->SimTime().Double();

  // asssumed that the imu orientation is:
  //   x forward
  //   y right
  //   z down

  // get linear acceleration in body frame
  const ignition::math::Vector3d linearAccel =
    this->dataPtr->imuSensor->LinearAcceleration();

  // copy to pkt
  pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
  pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
  // gzerr << "lin accel [" << linearAccel << "]\n";

  // get angular velocity in body frame
  const ignition::math::Vector3d angularVel =
    this->dataPtr->imuSensor->AngularVelocity();

  // copy to pkt
  pkt.imuAngularVelocityRPY[0] = angularVel.X();
  pkt.imuAngularVelocityRPY[1] = angularVel.Y();
  pkt.imuAngularVelocityRPY[2] = angularVel.Z();

  // get inertial pose and velocity
  // position of the uav in world frame
  // this position is used to calcualte bearing and distance
  // from starting location, then use that to update gps position.
  // The algorithm looks something like below (from ardupilot helper
  // libraries):
  //   bearing = to_degrees(atan2(position.y, position.x));
  //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
  //   (self.latitude, self.longitude) = util.gps_newpos(
  //    self.home_latitude, self.home_longitude, bearing, distance)
  // where xyz is in the NED directions.
  // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
  // around.
  // orientation of the uav in world NED frame -
  // assuming the world NED frame has xyz mapped to NED,
  // imuLink is NED - z down

  // model world pose brings us to model,
  // which for example zephyr has -y-forward, x-left, z-up
  // adding modelXYZToAirplaneXForwardZDown rotates
  //   from: model XYZ
  //   to: airplane x-forward, y-left, z-down
  const ignition::math::Pose3d gazeboXYZToModelXForwardZDown =
    this->modelXYZToAirplaneXForwardZDown +
    this->dataPtr->model->WorldPose();

  // get transform from world NED to Model frame
  const ignition::math::Pose3d NEDToModelXForwardZUp =
    gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

  // gzerr << "ned to model [" << NEDToModelXForwardZUp << "]\n";

  // N
  pkt.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

  // E
  pkt.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

  // D
  pkt.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

  // imuOrientationQuat is the rotation from world NED frame
  // to the uav frame.
  pkt.imuOrientationQuat[0] = NEDToModelXForwardZUp.Rot().W();
  pkt.imuOrientationQuat[1] = NEDToModelXForwardZUp.Rot().X();
  pkt.imuOrientationQuat[2] = NEDToModelXForwardZUp.Rot().Y();
  pkt.imuOrientationQuat[3] = NEDToModelXForwardZUp.Rot().Z();

  // gzdbg << "imu [" << gazeboXYZToModelXForwardZDown.rot.GetAsEuler()
  //       << "]\n";
  // gzdbg << "ned [" << this->gazeboXYZToNED.rot.GetAsEuler() << "]\n";
  // gzdbg << "rot [" << NEDToModelXForwardZUp.rot.GetAsEuler() << "]\n";

  // Get NED velocity in body frame *
  // or...
  // Get model velocity in NED frame
  const ignition::math::Vector3d velGazeboWorldFrame =
    this->dataPtr->model->GetLink()->WorldLinearVel();
  const ignition::math::Vector3d velNEDFrame =
    this->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
  pkt.velocityXYZ[0] = velNEDFrame.X();
  pkt.velocityXYZ[1] = velNEDFrame.Y();
  pkt.velocityXYZ[2] = velNEDFrame.Z();

  this->dataPtr->socket_out.Send(&pkt, sizeof(pkt));
}
