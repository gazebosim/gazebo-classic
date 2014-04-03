/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_DRC_VEHICLE_PLUGIN_HH
#define GAZEBO_DRC_VEHICLE_PLUGIN_HH

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>

namespace gazebo
{
  /// \addtogroup drc_plugin
  /// \{
  class DRCVehiclePlugin : public ModelPlugin
  {
    /// \enum DirectionType
    /// \brief Direction selector switch type.
    public: enum DirectionType {
              /// \brief Reverse
              REVERSE = -1,
              /// \brief Neutral
              NEUTRAL = 0,
              /// \brief Forward
              FORWARD = 1
            };

    /// \enum KeyType
    /// \brief Key switch type.
    public: enum KeyType {
              /// \brief On, but hasn't seen Neutral yet
              ON_FR = -1,
              /// \brief Off
              OFF   = 0,
              /// \brief On
              ON    = 1
            };

    /// \brief Constructor.
    public: DRCVehiclePlugin();

    /// \brief Destructor.
    public: virtual ~DRCVehiclePlugin();

    /// \brief Load the controller.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller.
    private: void UpdateStates();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Sets DRC Vehicle control inputs, the vehicle internal model
    ///        will decide the overall motion of the vehicle.
    /// \param[in] _handWheelPosition steering wheel position in radians.
    /// \param[in] _gasPedalPosition gas pedal position in meters.
    /// \param[in] _brakePedalPosition brake pedal position in meters.
    /// \param[in] _key key state.
    /// \param[in] _direction direction state.
    public: void SetVehicleState(double _handWheelPosition,
                                 double _gasPedalPosition,
                                 double _brakePedalPosition,
                                 KeyType _key,
                                 DirectionType _direction);

    /// \brief Returns the state of the key switch.
    /// \return Current key state.
    public: KeyType GetKeyState();

    /// \brief Sets the key switch to ON, may become ON_FR if not in NEUTRAL.
    public: void SetKeyOn();

    /// \brief Sets the key switch to OFF.
    public: void SetKeyOff();

    /// \brief Returns the state of the direction switch.
    /// \return Current direction state.
    public: DirectionType GetDirectionState();

    /// \brief Sets the state of the direction switch.
    /// \param[in] _direction Desired direction state.
    public: void SetDirectionState(DirectionType _direction);

    /// \brief Set the steering wheel angle; this will also update the front
    ///        wheel steering angle.
    /// \param[in] _position Steering wheel angle in radians.
    public: void SetHandWheelState(double _position);

    /// \brief Sets the lower and upper limits of the steering wheel angle.
    /// \param[in] _min Lower limit of steering wheel angle (radians).
    /// \param[in] _max Upper limit of steering wheel angle (radians).
    public: void SetHandWheelLimits(const math::Angle &_min,
                                    const math::Angle &_max);

    /// \brief Returns the lower and upper limits of the steering wheel angle.
    /// \param[out] _min Lower steering wheel limit (radians).
    /// \param[out] _max Upper steering wheel limit (radians).
    public: void GetHandWheelLimits(math::Angle &_min, math::Angle &_max);

    /// \brief Returns the steering wheel angle (rad).
    public: double GetHandWheelState();

    /// \brief Computes the front wheel angle / steering wheel angle ratio.
    public: void UpdateHandWheelRatio();

    /// \brief Returns the front wheel angle / steering wheel angle ratio.
    public: double GetHandWheelRatio();


    // TODO: fix handbrake documentation
    /// \brief Set the hand-brake angle.
    /// \param[in] _position Hand-brake angle in radians.
    public: void SetHandBrakeState(double _position);

    /// \brief Sets the lower and upper limits of the hand brake angle.
    /// \param[in] _min Lower limit of hand-brake angle (radians).
    /// \param[in] _max Upper limit of hand-brake angle (radians).
    public: void SetHandBrakeLimits(double &_min, double &_max);

    /// \brief Returns the lower and upper limits of the hand-brake angle.
    /// \param[out] _min Lower hand-brake limit (radians).
    /// \param[out] _max Upper hand-brake limit (radians).
    public: void GetHandBrakeLimits(double &_min, double &_max);

    /// \brief Returns the lower and upper limits of the FNR switch angle.
    /// \param[out] _min Lower FNR switch brake limit (radians).
    /// \param[out] _max Upper FNR switch brake limit (radians).
    public: void GetFNRSwitchLimits(double &_min, double &_max);


    /// \brief Specify front wheel orientation in radians (Note: this sets
    /// the vehicle wheels as oppsed to the steering wheel angle set by
    /// SetHandWheelState).
    /// Zero setting results in vehicle traveling in a straight line.
    /// Positive steering angle results in a left turn in forward motion.
    /// Negative steering angle results in a right turn in forward motion.
    /// Setting front wheel steering angle will also update the
    /// handWheel steering angle.
    /// \param[in] _position Desired angle of front steered wheels in radians.
    public: void SetSteeredWheelState(double _position);

    /// \brief Sets the lower and upper limits of the steering angle (rad).
    /// \param[in] _min Lower limit of steered wheel angle (radians).
    /// \param[in] _max Upper limit of steered wheel angle (radians).
    public: void SetSteeredWheelLimits(const math::Angle &_min,
                                       const math::Angle &_max);

    /// \brief Returns the steering angle of the steered wheels (rad).
    public: double GetSteeredWheelState();

    /// \brief Returns the lower and upper limits of the steering angle
    ///        of the steered wheels (rad).
    /// \param[out] _min Lower limit of steered wheel angle (radians).
    /// \param[out] _max Upper limit of steered wheel angle (radians).
    public: void GetSteeredWheelLimits(math::Angle &_min, math::Angle &_max);

    /// \brief Specify gas pedal position in meters.
    /// \param[in] _position Desired gas pedal position in meters.
    public: void SetGasPedalState(double _position);

    /// \brief Specify gas pedal position limits in meters.
    /// \param[in] _min Lower limit of gas pedal position (meters).
    /// \param[in] _max Upper limit of gas pedal position (meters).
    public: void SetGasPedalLimits(double _min, double _max);

    /// \brief Returns gas pedal position limits in meters.
    /// \param[out] _min Lower limit of gas pedal position (meters).
    /// \param[out] _max Upper limit of gas pedal position (meters).
    public: void GetGasPedalLimits(double &_min, double &_max);

    /// \brief Returns the gas pedal position in meters.
    public: double GetGasPedalState();

    /// \brief Returns the percent utilization of the gas pedal relative to
    ///        joint limits.
    public: double GetGasPedalPercent();

    /// \brief Specify brake pedal position in meters.
    /// \param[in] _position Desired brake pedal position in meters.
    public: void SetBrakePedalState(double _position);

    /// \brief Sets brake pedal position limits in meters.
    /// \param[in] _min Lower limit of brake pedal position (meters).
    /// \param[in] _max Upper limit of brake pedal position (meters).
    public: void SetBrakePedalLimits(double _min, double _max);

    /// \brief Returns brake pedal position limits in meters.
    /// \param[out] _min Lower limit of brake pedal position (meters).
    /// \param[out] _max Upper limit of brake pedal position (meters).
    public: void GetBrakePedalLimits(double &_min, double &_max);

    /// \brief Returns the brake pedal position in meters.
    public: double GetBrakePedalState();

    /// \brief Returns the percent utilization of the brake pedal relative to
    ///        joint limits.
    public: double GetBrakePedalPercent();

    /// Default plugin init call.
    public: virtual void Init();

    private: double GetGasTorqueMultiplier();
    private: double get_collision_radius(physics::CollisionPtr _collision);
    private: math::Vector3 get_collision_position(physics::LinkPtr _link,
                                                  unsigned int _id);

    /// \brief Transport node used for publishing visual messages.
    private: transport::NodePtr node;

    /// \brief Publisher for visual messages for FNR switch.
    private: transport::PublisherPtr visualPub;

    /// \brief Message for FNR switch visual indicating forward.
    private: msgs::Visual msgForward;

    /// \brief Message for FNR switch visual indicating reverse.
    private: msgs::Visual msgReverse;

    private: physics::JointPtr gasPedalJoint;
    private: physics::JointPtr brakePedalJoint;
    private: physics::JointPtr handWheelJoint;
    private: physics::JointPtr flWheelJoint;
    private: physics::JointPtr frWheelJoint;
    private: physics::JointPtr blWheelJoint;
    private: physics::JointPtr brWheelJoint;
    private: physics::JointPtr flWheelSteeringJoint;
    private: physics::JointPtr frWheelSteeringJoint;

    /// \brief The gas/brake pedals and handbrake apply torque to the wheels
    ///        based on their joint position as a percentage of the total
    ///        range of travel. The constant jointDeadbandPercent adds a small
    ///        deadband between the actual joint limits and the 0% and 100%
    ///        values reported by Get[GasPedal|BrakePedal|HandBrake]Percent()
    private: const double jointDeadbandPercent;

    // SDF parameters
    private: double frontTorque;
    private: double backTorque;
    private: double frontBrakeTorque;
    private: double backBrakeTorque;
    private: double tireAngleRange;
    private: double maxSpeed;
    private: double maxSteer;
    private: double aeroLoad;

    /// \brief Minimum braking percentage, used to approximate
    ///        rolling resistance and engine braking.
    private: double minBrakePercent;

    private: double steeringRatio;
    private: double pedalForce;
    private: double handWheelForce;
    private: double steeredWheelForce;

    protected: double gasPedalCmd;
    protected: double brakePedalCmd;
    protected: double handWheelCmd;
    private: double flWheelCmd;
    private: double frWheelCmd;
    private: double blWheelCmd;
    private: double brWheelCmd;
    private: double flWheelSteeringCmd;
    private: double frWheelSteeringCmd;

    private: common::PID gasPedalPID;
    private: common::PID brakePedalPID;
    private: common::PID handWheelPID;
    private: common::PID flWheelSteeringPID;
    private: common::PID frWheelSteeringPID;

    /// \brief Time of last update, used to detect resets.
    private: common::Time lastTime;

    /// joint information from model
    private: double gasPedalHigh;
    private: double gasPedalLow;
    private: double gasPedalRange;
    private: double brakePedalHigh;
    private: double brakePedalLow;
    private: double brakePedalRange;
    private: double handWheelHigh;
    private: double handWheelLow;
    private: double handWheelRange;
    private: double wheelRadius;
    private: double flWheelRadius;
    private: double frWheelRadius;
    private: double blWheelRadius;
    private: double brWheelRadius;
    private: double wheelbaseLength;
    private: double frontTrackWidth;
    private: double backTrackWidth;

    /// state of vehicle
    private: KeyType keyState;
    private: DirectionType directionState;
    private: double handWheelState;
    private: double flSteeringState;
    private: double frSteeringState;
    private: double gasPedalState;
    private: double brakePedalState;
    private: double flWheelState;
    private: double frWheelState;
    private: double blWheelState;
    private: double brWheelState;
  };
/// \}
}
#endif
