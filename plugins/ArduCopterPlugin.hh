/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_ARDUCOPTER_PLUGIN_HH_
#define _GAZEBO_ARDUCOPTER_PLUGIN_HH_

// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
// #include <sys/select.h>

#include <array>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/math/Filter.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name,
  T& param, const T& default_value, const bool& verbose = false)
{
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[ArduCopterPlugin] Please specify a value for parameter ["
            << name << "].\n";
  }
  return false;
}

namespace gazebo
{
  /// \brief Interface ArduCopter from ardupilot stack
  /// modeled after SITL/SIM_*
  ///
  /// The plugin requires the following parameters:
  /// <controller_ip>       controller is on this ip
  /// <state_port>          This plugin publishes states on this port
  /// <command_port>        This plugin receives states on this port
  ///
  /// The following parameters are optional:
  /// <param.xml>           use params stored at this absolute file path
  ///
  class GAZEBO_VISIBLE ArduCopterPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ArduCopterPlugin();

    /// \brief Destructor.
    public: ~ArduCopterPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    private: void UpdatePIDs(double _dt);

    /// \brief Get motor commands from ArduCopter
    private: void GetMotorCommand();

    /// \brief Publish ArduCopter state.
    private: void SendState();

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    // private: transport::NodePtr node;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    private: class Rotor
    {
      public: Rotor()
      {
        this->maxRpm = 838.0;
        this->cmd = 0;
        this->multiplier = 1;

        // most of these coefficients are not used yet.
        this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
        this->frequencyCutoff = this->kDefaultFrequencyCutoff;
        this->samplingRate = this->kDefaultSamplingRate;

        this->pGain = 0.10;
        this->iGain = 0;
        this->dGain = 0;
        this->cmdMax = 1.0;
        this->cmdMin = -1.0;
        this->iMax = 0;
        this->iMin = 0;
        this->pid.Init(this->pGain, this->iGain, this->dGain,
                       this->iMax, this->iMin,
                       this->cmdMax, this->cmdMin);
      }

      public: ~Rotor() {}

      /// \brief rotor id
      protected: int id;

      /// \brief Max rotor propeller RPM.
      protected: double maxRpm;

      /// \brief Next command to be applied to the propeller
      protected: double cmd;

      /// \brief Velocity PID gains for motor control
      protected: double pGain;
      protected: double iGain;
      protected: double dGain;
      protected: double cmdMax;
      protected: double cmdMin;
      protected: double iMax;
      protected: double iMin;

      /// \brief Velocity PID for motor control
      protected: common::PID pid;

      /// \brief Control propeller joint.
      private: std::string jointName;

      /// \brief Control propeller joint.
      private: physics::JointPtr joint;

      /// \brief Control propeller link.
      private: std::string linkName;

      /// \brief Control propeller link.
      private: physics::LinkPtr link;

      /// \brief direction multiplier for this rotor
      protected: double multiplier;

      /// \brief unused coefficients
      protected: double rotorVelocitySlowdownSim;
      protected: double frequencyCutoff;
      protected: double samplingRate;
      protected: math::OnePole<double> velocityFilter;

      private: static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;
      private: static constexpr double kDefaultFrequencyCutoff = 5.0;
      private: static constexpr double kDefaultSamplingRate = 0.2;

      friend class ArduCopterPlugin;
    };

    /// \brief array of propellers
    private: std::vector<Rotor> rotors;

    /// \brief keep track of controller update sim-time.
    private: gazebo::common::Time lastControllerUpdateTime;

    /// \brief Controller update mutex.
    private: std::mutex mutex;

    private: bool bind(const char *address, uint16_t port)
    {
        struct sockaddr_in sockaddr;
        make_sockaddr(address, port, sockaddr);

        if (::bind(this->handle,
          (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
            return false;
        }
        return true;
    }

    // socket stuff
    private: void make_sockaddr(const char *address, uint16_t port,
      struct sockaddr_in &sockaddr)
    {
      memset(&sockaddr, 0, sizeof(sockaddr));

      #ifdef HAVE_SOCK_SIN_LEN
        sockaddr.sin_len = sizeof(sockaddr);
      #endif

      sockaddr.sin_port = htons(port);
      sockaddr.sin_family = AF_INET;
      sockaddr.sin_addr.s_addr = inet_addr(address);
    }

    private: ssize_t recv(void *buf, size_t size, uint32_t timeout_ms)
    {
        fd_set fds;
        struct timeval tv;

        FD_ZERO(&fds);
        FD_SET(this->handle, &fds);

        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000UL;

        if (select(this->handle+1, &fds, NULL, NULL, &tv) != 1) {
            return -1;
        }
        
        return ::recv(this->handle, buf, size, 0);
    }

    private: struct servo_packet
    {
      float motor_speed[4];
    };

    private: struct fdm_packet
    {
      double timestamp;
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_quat[4];
      double velocity_xyz[3];
      double position_xyz[3];
    };
        
    private: int handle;

    private: physics::LinkPtr imuLink;

    private: boost::shared_ptr<sensors::ImuSensor> imuSensor;
  };
}
#endif
