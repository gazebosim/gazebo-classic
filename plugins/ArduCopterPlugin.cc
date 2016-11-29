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
#include "plugins/ArduCopterPlugin.hh"

#define MAX_MOTORS 255

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ArduCopterPlugin)

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
    gzerr << "[ArduCopterPlugin] Please specify a value for parameter ["
      << _name << "].\n";
  }
  return false;
}

/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  float motorSpeed[MAX_MOTORS];
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduCopter
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

/// \brief Rotor class
class Rotor
{
  /// \brief Constructor
  public: Rotor()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

  /// \brief rotor id
  public: int id = 0;

  /// \brief Max rotor propeller RPM.
  public: double maxRpm = 838.0;

  /// \brief Next command to be applied to the propeller
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: common::PID pid;

  /// \brief Control propeller joint.
  public: std::string jointName;

  /// \brief Control propeller joint.
  public: physics::JointPtr joint;

  /// \brief direction multiplier for this rotor
  public: double multiplier = 1;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: ignition::math::OnePole<double> velocityFilter;

  public: static double kDefaultRotorVelocitySlowdownSim;
  public: static double kDefaultFrequencyCutoff;
  public: static double kDefaultSamplingRate;
};

double Rotor::kDefaultRotorVelocitySlowdownSim = 10.0;
double Rotor::kDefaultFrequencyCutoff = 5.0;
double Rotor::kDefaultSamplingRate = 0.2;

// Private data class
class gazebo::ArduCopterPluginPrivate
{
  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->handle, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->handle, 0);
      #ifdef _WIN32
      closesocket(this->handle);
      #else
      close(this->handle);
      #endif
      return false;
    }
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

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->handle, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->handle+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->handle, reinterpret_cast<char *>(_buf), _size, 0);
    #else
    return recv(this->handle, _buf, _size, 0);
    #endif
  }

  /// \brief Pointer to the update event connection.
  public: event::ConnectionPtr updateConnection;

  /// \brief Pointer to the model;
  public: physics::ModelPtr model;

  /// \brief array of propellers
  public: std::vector<Rotor> rotors;

  /// \brief keep track of controller update sim-time.
  public: gazebo::common::Time lastControllerUpdateTime;

  /// \brief Controller update mutex.
  public: std::mutex mutex;

  /// \brief Socket handle
  public: int handle;

  /// \brief Pointer to an IMU sensor
  public: sensors::ImuSensorPtr imuSensor;

  /// \brief false before ardupilot controller is online
  /// to allow gazebo to continue without waiting
  public: bool arduCopterOnline;

  /// \brief number of times ArduCotper skips update
  public: int connectionTimeoutCount;

  /// \brief number of times ArduCotper skips update
  /// before marking ArduCopter offline
  public: int connectionTimeoutMaxCount;
};

////////////////////////////////////////////////////////////////////////////////
ArduCopterPlugin::ArduCopterPlugin()
  : dataPtr(new ArduCopterPluginPrivate)
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

  if (!this->dataPtr->Bind("127.0.0.1", 9002))
  {
    gzerr << "failed to bind with 127.0.0.1:9002, aborting plugin.\n";
    return;
  }

  this->dataPtr->arduCopterOnline = false;

  this->dataPtr->connectionTimeoutCount = 0;

  setsockopt(this->dataPtr->handle, SOL_SOCKET, SO_REUSEADDR,
     reinterpret_cast<const char *>(&one), sizeof(one));

  #ifdef _WIN32
  u_long on = 1;
  ioctlsocket(this->dataPtr->handle, FIONBIO, reinterpret_cast<u_long FAR *>(&on));
  #else
  fcntl(this->dataPtr->handle, F_SETFL,
      fcntl(this->dataPtr->handle, F_GETFL, 0) | O_NONBLOCK);
  #endif
}

/////////////////////////////////////////////////
ArduCopterPlugin::~ArduCopterPlugin()
{
}

/////////////////////////////////////////////////
void ArduCopterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ArduCopterPlugin _model pointer is null");
  GZ_ASSERT(_sdf, "ArduCopterPlugin _sdf pointer is null");

  this->dataPtr->model = _model;

  // per rotor
  if (_sdf->HasElement("rotor"))
  {
    sdf::ElementPtr rotorSDF = _sdf->GetElement("rotor");

    while (rotorSDF)
    {
      Rotor rotor;

      if (rotorSDF->HasAttribute("id"))
      {
        rotor.id = rotorSDF->GetAttribute("id")->Get(rotor.id);
      }
      else
      {
        rotor.id = this->dataPtr->rotors.size();
        gzwarn << "id attribute not specified, use order parsed ["
               << rotor.id << "].\n";
      }

      if (rotorSDF->HasElement("jointName"))
      {
        rotor.jointName = rotorSDF->Get<std::string>("jointName");
      }
      else
      {
        gzerr << "Please specify a jointName,"
          << " where the rotor is attached.\n";
      }

      // Get the pointer to the joint.
      rotor.joint = _model->GetJoint(rotor.jointName);
      if (rotor.joint == nullptr)
      {
        gzerr << "Couldn't find specified joint ["
            << rotor.jointName << "]. This plugin will not run.\n";
        return;
      }

      if (rotorSDF->HasElement("turningDirection"))
      {
        std::string turningDirection = rotorSDF->Get<std::string>(
            "turningDirection");
        // special cases mimic from rotors_gazebo_plugins
        if (turningDirection == "cw")
          rotor.multiplier = -1;
        else if (turningDirection == "ccw")
          rotor.multiplier = 1;
        else
        {
          gzdbg << "not string, check turningDirection as float\n";
          rotor.multiplier = rotorSDF->Get<double>("turningDirection");
        }
      }
      else
      {
        rotor.multiplier = 1;
        gzerr << "Please specify a turning"
          << " direction multiplier ('cw' or 'ccw'). Default 'ccw'.\n";
      }

      getSdfParam<double>(rotorSDF, "rotorVelocitySlowdownSim",
          rotor.rotorVelocitySlowdownSim, 1);

      if (ignition::math::equal(rotor.rotorVelocitySlowdownSim, 0.0))
      {
        gzerr << "rotor for joint [" << rotor.jointName
              << "] rotorVelocitySlowdownSim is zero,"
              << " aborting plugin.\n";
        return;
      }

      getSdfParam<double>(rotorSDF, "frequencyCutoff",
          rotor.frequencyCutoff, rotor.frequencyCutoff);
      getSdfParam<double>(rotorSDF, "samplingRate",
          rotor.samplingRate, rotor.samplingRate);

      // use gazebo::math::Filter
      rotor.velocityFilter.Fc(rotor.frequencyCutoff, rotor.samplingRate);

      // initialize filter to zero value
      rotor.velocityFilter.Set(0.0);

      // note to use this
      // rotorVelocityFiltered = velocityFilter.Process(rotorVelocityRaw);

      // Overload the PID parameters if they are available.
      double param;
      getSdfParam<double>(rotorSDF, "vel_p_gain", param, rotor.pid.GetPGain());
      rotor.pid.SetPGain(param);

      getSdfParam<double>(rotorSDF, "vel_i_gain", param, rotor.pid.GetIGain());
      rotor.pid.SetIGain(param);

      getSdfParam<double>(rotorSDF, "vel_d_gain", param,  rotor.pid.GetDGain());
      rotor.pid.SetDGain(param);

      getSdfParam<double>(rotorSDF, "vel_i_max", param, rotor.pid.GetIMax());
      rotor.pid.SetIMax(param);

      getSdfParam<double>(rotorSDF, "vel_i_min", param, rotor.pid.GetIMin());
      rotor.pid.SetIMin(param);

      getSdfParam<double>(rotorSDF, "vel_cmd_max", param,
          rotor.pid.GetCmdMax());
      rotor.pid.SetCmdMax(param);

      getSdfParam<double>(rotorSDF, "vel_cmd_min", param,
          rotor.pid.GetCmdMin());
      rotor.pid.SetCmdMin(param);

      // set pid initial command
      rotor.pid.SetCmd(0.0);

      this->dataPtr->rotors.push_back(rotor);
      rotorSDF = rotorSDF->GetNextElement("rotor");
    }
  }

  // Get sensors
  std::string imuName;
  getSdfParam<std::string>(_sdf, "imuName", imuName, "imu_sensor");
  std::string imuScopedName = this->dataPtr->model->GetWorld()->GetName()
      + "::" + this->dataPtr->model->GetScopedName()
      + "::" + imuName;
  this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
    (sensors::SensorManager::Instance()->GetSensor(imuScopedName));

  if (!this->dataPtr->imuSensor)
  {
    gzerr << "imu_sensor [" << imuScopedName
          << "] not found, abort ArduCopter plugin.\n" << "\n";
    return;
  }

  // Controller time control.
  this->dataPtr->lastControllerUpdateTime = 0;

  // Missed update count before we declare arduCopterOnline status false
  getSdfParam<int>(_sdf, "connectionTimeoutMaxCount",
    this->dataPtr->connectionTimeoutMaxCount, 10);

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ArduCopterPlugin::OnUpdate, this));

  gzlog << "ArduCopter ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void ArduCopterPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gazebo::common::Time curTime = this->dataPtr->model->GetWorld()->GetSimTime();

  // Update the control surfaces and publish the new state.
  if (curTime > this->dataPtr->lastControllerUpdateTime)
  {
    this->ReceiveMotorCommand();
    if (this->dataPtr->arduCopterOnline)
    {
      this->ApplyMotorForces((curTime -
        this->dataPtr->lastControllerUpdateTime).Double());
      this->SendState();
    }
  }

  this->dataPtr->lastControllerUpdateTime = curTime;
}

/////////////////////////////////////////////////
void ArduCopterPlugin::ResetPIDs()
{
  // Reset velocity PID for rotors
  for (size_t i = 0; i < this->dataPtr->rotors.size(); ++i)
  {
    this->dataPtr->rotors[i].cmd = 0;
    // this->dataPtr->rotors[i].pid.Reset();
  }
}

/////////////////////////////////////////////////
void ArduCopterPlugin::ApplyMotorForces(const double _dt)
{
  // update velocity PID for rotors and apply force to joint
  for (size_t i = 0; i < this->dataPtr->rotors.size(); ++i)
  {
    double velTarget = this->dataPtr->rotors[i].multiplier *
      this->dataPtr->rotors[i].cmd /
      this->dataPtr->rotors[i].rotorVelocitySlowdownSim;
    double vel = this->dataPtr->rotors[i].joint->GetVelocity(0);
    double error = vel - velTarget;
    double force = this->dataPtr->rotors[i].pid.Update(error, _dt);
    this->dataPtr->rotors[i].joint->SetForce(0, force);
  }
}

/////////////////////////////////////////////////
void ArduCopterPlugin::ReceiveMotorCommand()
{
  // Added detection for whether ArduCopter is online or not.
  // If ArduCopter is detected (receive of fdm packet from someone),
  // then socket receive wait time is increased from 1ms to 1 sec
  // to accomodate network jitter.
  // If ArduCopter is not detected, receive call blocks for 1ms
  // on each call.
  // Once ArduCopter presence is detected, it takes this many
  // missed receives before declaring the FCS offline.

  ServoPacket pkt;
  int waitMs = 1;
  if (this->dataPtr->arduCopterOnline)
  {
    // increase timeout for receive once we detect a packet from
    // ArduCopter FCS.
    waitMs = 1000;
  }
  else
  {
    // Otherwise skip quickly and do not set control force.
    waitMs = 1;
  }
  ssize_t recvSize = this->dataPtr->Recv(&pkt, sizeof(ServoPacket), waitMs);
  ssize_t expectedPktSize =
    sizeof(pkt.motorSpeed[0])*this->dataPtr->rotors.size();
  if ((recvSize == -1) || (recvSize < expectedPktSize))
  {
    // didn't receive a packet
    // gzerr << "no packet\n";
    if (recvSize != -1)
    {
      gzerr << "received bit size (" << recvSize << ") to small,"
            << " controller expected size (" << expectedPktSize << ").\n";
    }

    gazebo::common::Time::NSleep(100);
    if (this->dataPtr->arduCopterOnline)
    {
      gzwarn << "Broken ArduCopter connection, count ["
             << this->dataPtr->connectionTimeoutCount
             << "/" << this->dataPtr->connectionTimeoutMaxCount
             << "]\n";
      if (++this->dataPtr->connectionTimeoutCount >
        this->dataPtr->connectionTimeoutMaxCount)
      {
        this->dataPtr->connectionTimeoutCount = 0;
        this->dataPtr->arduCopterOnline = false;
        gzwarn << "Broken ArduCopter connection, resetting motor control.\n";
        this->ResetPIDs();
      }
    }
  }
  else
  {
    if (!this->dataPtr->arduCopterOnline)
    {
      gzdbg << "ArduCopter controller online detected.\n";
      // made connection, set some flags
      this->dataPtr->connectionTimeoutCount = 0;
      this->dataPtr->arduCopterOnline = true;
    }

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->dataPtr->rotors.size(); ++i)
    {
      if (i < MAX_MOTORS)
      {
        // std::cout << i << ": " << pkt.motorSpeed[i] << "\n";
        this->dataPtr->rotors[i].cmd = this->dataPtr->rotors[i].maxRpm *
          pkt.motorSpeed[i];
      }
      else
      {
        gzerr << "too many motors, skipping [" << i
              << " > " << MAX_MOTORS << "].\n";
      }
    }
  }
}

/////////////////////////////////////////////////
void ArduCopterPlugin::SendState() const
{
  // send_fdm
  fdmPacket pkt;

  pkt.timestamp = this->dataPtr->model->GetWorld()->GetSimTime().Double();

  // asssumed that the imu orientation is:
  //   x forward
  //   y right
  //   z down

  // get linear acceleration in body frame
  ignition::math::Vector3d linearAccel =
    this->dataPtr->imuSensor->LinearAcceleration();

  // copy to pkt
  pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
  pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
  // gzerr << "lin accel [" << linearAccel << "]\n";

  // get angular velocity in body frame
  ignition::math::Vector3d angularVel =
    this->dataPtr->imuSensor->AngularVelocity();

  // copy to pkt
  pkt.imuAngularVelocityRPY[0] = angularVel.X();
  pkt.imuAngularVelocityRPY[1] = angularVel.Y();
  pkt.imuAngularVelocityRPY[2] = angularVel.Z();

  // get inertial pose and velocity
  // position of the quadrotor in world frame
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
  // orientation of the quadrotor in world NED frame -
  // assuming the world NED frame has xyz mapped to NED,
  // imuLink is NED - z down

  // gazeboToNED brings us from gazebo model: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  ignition::math::Pose3d gazeboToNED(0, 0, 0, IGN_PI, 0, 0);

  // model world pose brings us to model, x-forward, y-left, z-up
  // adding gazeboToNED gets us to the x-forward, y-right, z-down
  ignition::math::Pose3d worldToModel = gazeboToNED +
    this->dataPtr->model->GetWorldPose().Ign();

  // get transform from world NED to Model frame
  ignition::math::Pose3d NEDToModel = worldToModel - gazeboToNED;

  // gzerr << "ned to model [" << NEDToModel << "]\n";

  // N
  pkt.positionXYZ[0] = NEDToModel.Pos().X();

  // E
  pkt.positionXYZ[1] = NEDToModel.Pos().Y();

  // D
  pkt.positionXYZ[2] = NEDToModel.Pos().Z();

  // imuOrientationQuat is the rotation from world NED frame
  // to the quadrotor frame.
  pkt.imuOrientationQuat[0] = NEDToModel.Rot().W();
  pkt.imuOrientationQuat[1] = NEDToModel.Rot().X();
  pkt.imuOrientationQuat[2] = NEDToModel.Rot().Y();
  pkt.imuOrientationQuat[3] = NEDToModel.Rot().Z();

  // gzdbg << "imu [" << worldToModel.rot.GetAsEuler() << "]\n";
  // gzdbg << "ned [" << gazeboToNED.rot.GetAsEuler() << "]\n";
  // gzdbg << "rot [" << NEDToModel.rot.GetAsEuler() << "]\n";

  // Get NED velocity in body frame *
  // or...
  // Get model velocity in NED frame
  ignition::math::Vector3d velGazeboWorldFrame =
    this->dataPtr->model->GetLink()->GetWorldLinearVel().Ign();
  ignition::math::Vector3d velNEDFrame =
    gazeboToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
  pkt.velocityXYZ[0] = velNEDFrame.X();
  pkt.velocityXYZ[1] = velNEDFrame.Y();
  pkt.velocityXYZ[2] = velNEDFrame.Z();

  struct sockaddr_in sockaddr;
  this->dataPtr->MakeSockAddr("127.0.0.1", 9003, sockaddr);

  ::sendto(this->dataPtr->handle,
           reinterpret_cast<raw_type *>(&pkt),
           sizeof(pkt), 0,
           (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}
