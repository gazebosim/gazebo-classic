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
        this->maxRpm = 8000;
        this->cmd = 0;
        this->multiplier = 1;

        // most of these coefficients are not used yet.
        this->rotorDragCoefficient = this->kDefaultRotorDragCoefficient;
        this->rollingMomentCoefficient = this->kDefaultRollingMomentCoefficient;
        this->maxRotVelocity = this->kDefaultMaxRotVelocity;
        this->motorConstant = this->kDefaultMotorConstant;
        this->momentConstant = this->kDefaultMomentConstant;
        this->timeConstantUp = this->kDefaultTimeConstantUp;
        this->timeConstantDown = this->kDefaultTimeConstantDown;
        this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
        this->frequencyCutoff = this->kDefaultFrequencyCutoff;
        this->samplingRate = this->kDefaultSamplingRate;
        this->maxForce = this->kDefaultMaxForce;

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
      protected: int maxRpm;

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

      /// \brief max joint force
      protected: double maxForce;

      /// \brief unused coefficients
      protected: double rotorDragCoefficient;
      protected: double rollingMomentCoefficient;
      protected: double maxRotVelocity;
      protected: double motorConstant;
      protected: double momentConstant;
      protected: double timeConstantUp;
      protected: double timeConstantDown;
      protected: double rotorVelocitySlowdownSim;
      protected: double frequencyCutoff;
      protected: double samplingRate;
      protected: math::OnePole<double> velocityFilter;

      private: static constexpr double kDefaultMaxForce =
        std::numeric_limits<double>::max();
      private: static constexpr double kDefaultMaxRotVelocity = 853;
      private: static constexpr double kDefaultMotorConstant = 8.54858e-06;
      private: static constexpr double kDefaultMomentConstant = 0.016;
      private: static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
      private: static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
      private: static constexpr double kDefaulMaxRotVelocity = 838.0;
      private: static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
      private: static constexpr double kDefaultRollingMomentCoefficient = 1e-6;
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
  };
}
#endif
