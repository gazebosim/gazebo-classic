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

#include <math.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Base.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/transport/transport.hh>

#include "DRCVehiclePlugin.hh"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
DRCVehiclePlugin::DRCVehiclePlugin()
  : jointDeadbandPercent(0.02)
{
  this->keyState = ON;
  this->directionState = FORWARD;
  this->gasPedalCmd = 0;
  this->brakePedalCmd = 0;
  this->handWheelCmd = 0;
  this->flWheelCmd = 0;
  this->frWheelCmd = 0;
  this->blWheelCmd = 0;
  this->brWheelCmd = 0;
  this->flWheelSteeringCmd = 0;
  this->frWheelSteeringCmd = 0;

  /// \TODO: get this from model
  this->wheelRadius = 0.1;
  this->flWheelRadius = 0.1;
  this->frWheelRadius = 0.1;
  this->blWheelRadius = 0.1;
  this->brWheelRadius = 0.1;
  this->pedalForce = 10;
  this->handWheelForce = 1;
  this->steeredWheelForce = 5000;

  this->frontTorque = 0;
  this->backTorque = 0;
  this->frontBrakeTorque = 0;
  this->backBrakeTorque = 0;
  this->tireAngleRange = 0;
  this->maxSpeed = 0;
  this->maxSteer = 0;
  this->aeroLoad = 0;
  this->minBrakePercent = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DRCVehiclePlugin::~DRCVehiclePlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize
void DRCVehiclePlugin::Init()
{
  this->node.reset(new transport::Node());
  this->node->Init(this->world->GetName());

  this->visualPub = this->node->Advertise<msgs::Visual>("~/visual");
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetVehicleState(double _handWheelPosition,
                                       double _gasPedalPosition,
                                       double _brakePedalPosition,
                                   DRCVehiclePlugin::KeyType _key,
                                   DRCVehiclePlugin::DirectionType _direction)
{
  // This function isn't currently looking at joint limits.
  this->handWheelCmd = _handWheelPosition;
  this->gasPedalCmd = _gasPedalPosition;
  this->brakePedalCmd = _brakePedalPosition;
  this->directionState = _direction;
  this->keyState = _key;
}

////////////////////////////////////////////////////////////////////////////////
DRCVehiclePlugin::DirectionType DRCVehiclePlugin::GetDirectionState()
{
  return this->directionState;
}

////////////////////////////////////////////////////////////////////////////////
DRCVehiclePlugin::KeyType DRCVehiclePlugin::GetKeyState()
{
  return this->keyState;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetDirectionState(
        DRCVehiclePlugin::DirectionType _direction)
{
  this->directionState = _direction;
  if (_direction == NEUTRAL && this->keyState == ON_FR)
    this->keyState = ON;
  //if (_direction == FORWARD)
  //{}
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetKeyOff()
{
  this->keyState = OFF;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetKeyOn()
{
  if (this->directionState == NEUTRAL)
    this->keyState = ON;
  else
    this->keyState = ON_FR;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasTorqueMultiplier()
{
  if (this->keyState == ON)
  {
    if (this->directionState == FORWARD)
      return 1.0;
    else if (this->directionState == REVERSE)
      return -1.0;
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelState(double _position)
{
  math::Angle min, max;
  this->GetHandWheelLimits(min, max);
  this->handWheelCmd = math::clamp(_position, min.Radian(), max.Radian());
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetHandWheelLimits(const math::Angle &_min,
                                          const math::Angle &_max)
{
  this->handWheelJoint->SetHighStop(0, _max);
  this->handWheelJoint->SetLowStop(0, _min);
  this->UpdateHandWheelRatio();
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetHandWheelLimits(math::Angle &_min, math::Angle &_max)
{
  _max = this->handWheelJoint->GetHighStop(0);
  _min = this->handWheelJoint->GetLowStop(0);
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandWheelState()
{
  return this->handWheelState;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::UpdateHandWheelRatio()
{
  // The total range the steering wheel can rotate
  this->handWheelHigh  = this->handWheelJoint->GetHighStop(0).Radian();
  this->handWheelLow   = this->handWheelJoint->GetLowStop(0).Radian();
  this->handWheelRange = this->handWheelHigh - this->handWheelLow;
  double high = std::min(this->flWheelSteeringJoint->GetHighStop(0).Radian(),
                         this->frWheelSteeringJoint->GetHighStop(0).Radian());
  high = std::min(high, this->maxSteer);
  double low = std::max(this->flWheelSteeringJoint->GetLowStop(0).Radian(),
                        this->frWheelSteeringJoint->GetLowStop(0).Radian());
  low = std::max(low, -this->maxSteer);
  this->tireAngleRange = high - low;

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = this->tireAngleRange / this->handWheelRange;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetHandWheelRatio()
{
  return this->steeringRatio;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelState(double _position)
{
  this->SetHandWheelState(_position / this->steeringRatio);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetSteeredWheelLimits(const math::Angle &_min,
                                         const math::Angle &_max)
{
  this->flWheelSteeringJoint->SetHighStop(0, _max);
  this->flWheelSteeringJoint->SetLowStop(0, _min);
  this->frWheelSteeringJoint->SetHighStop(0, _max);
  this->frWheelSteeringJoint->SetLowStop(0, _min);
  this->UpdateHandWheelRatio();
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetSteeredWheelState()
{
    return 0.5*(flSteeringState + frSteeringState);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetSteeredWheelLimits(math::Angle &_min,
  math::Angle &_max)
{
  _max = 0.5 * (this->flWheelSteeringJoint->GetHighStop(0).Radian() +
                this->frWheelSteeringJoint->GetHighStop(0).Radian());
  _min = 0.5 * (this->flWheelSteeringJoint->GetLowStop(0).Radian() +
                this->frWheelSteeringJoint->GetLowStop(0).Radian());
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetGasPedalState(double _position)
{
  double min, max;
  this->GetGasPedalLimits(min, max);
  this->gasPedalCmd = math::clamp(_position, min, max);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetGasPedalLimits(double _min, double _max)
{
  this->gasPedalJoint->SetHighStop(0, _max);
  this->gasPedalJoint->SetLowStop(0, _min);
  this->gasPedalHigh  = this->gasPedalJoint->GetHighStop(0).Radian();
  this->gasPedalLow   = this->gasPedalJoint->GetLowStop(0).Radian();
  this->gasPedalRange   = this->gasPedalHigh - this->gasPedalLow;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetGasPedalLimits(double &_min, double &_max)
{
  _max = this->gasPedalHigh;
  _min = this->gasPedalLow;
}

/// Returns the gas pedal position in meters.
////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasPedalState()
{
  return this->gasPedalState;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetGasPedalPercent()
{
  double min, max;
  this->GetGasPedalLimits(min, max);
  return math::clamp((this->gasPedalState - min) / (max-min), 0.0, 1.0);
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetBrakePedalPercent()
{
  double min, max;
  this->GetBrakePedalLimits(min, max);
  return math::clamp((this->brakePedalState - min) / (max-min), 0.0, 1.0);
}



////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetBrakePedalState(double _position)
{
  double min, max;
  this->GetBrakePedalLimits(min, max);
  this->brakePedalCmd = math::clamp(_position, min, max);
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::SetBrakePedalLimits(double _min, double _max)
{
  this->brakePedalJoint->SetHighStop(0, _max);
  this->brakePedalJoint->SetLowStop(0, _min);
  this->brakePedalHigh  = this->brakePedalJoint->GetHighStop(0).Radian();
  this->brakePedalLow   = this->brakePedalJoint->GetLowStop(0).Radian();
  this->brakePedalRange = this->brakePedalHigh - this->brakePedalLow;
}

////////////////////////////////////////////////////////////////////////////////
void DRCVehiclePlugin::GetBrakePedalLimits(double &_min, double &_max)
{
  _max = this->brakePedalHigh;
  _min = this->brakePedalLow;
}

////////////////////////////////////////////////////////////////////////////////
double DRCVehiclePlugin::GetBrakePedalState()
{
  return this->brakePedalState;
}


////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DRCVehiclePlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;

  // Get joints
  std::string gasPedalJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("gas_pedal");
  this->gasPedalJoint = this->model->GetJoint(gasPedalJointName);
  if (!this->gasPedalJoint)
    gzthrow("could not find gas pedal joint\n");

  std::string brakePedalJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("brake_pedal");
  this->brakePedalJoint = this->model->GetJoint(brakePedalJointName);
  if (!this->brakePedalJoint)
    gzthrow("could not find brake pedal joint\n");

  std::string handWheelJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("steering_wheel");
  this->handWheelJoint = this->model->GetJoint(handWheelJointName);
  if (!this->handWheelJoint)
    gzthrow("could not find steering wheel joint\n");


  std::string flWheelJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel");
  this->flWheelJoint = this->model->GetJoint(flWheelJointName);
  if (!this->flWheelJoint)
    gzthrow("could not find front left wheel joint\n");

  std::string frWheelJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel");
  this->frWheelJoint = this->model->GetJoint(frWheelJointName);
  if (!this->frWheelJoint)
    gzthrow("could not find front right wheel joint\n");

  std::string blWheelJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("back_left_wheel");
  this->blWheelJoint = this->model->GetJoint(blWheelJointName);
  if (!this->blWheelJoint)
    gzthrow("could not find back left wheel joint\n");

  std::string brWheelJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("back_right_wheel");
  this->brWheelJoint = this->model->GetJoint(brWheelJointName);
  if (!this->brWheelJoint)
    gzthrow("could not find back right wheel joint\n");

  std::string flWheelSteeringJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel_steering");
  this->flWheelSteeringJoint = this->model->GetJoint(flWheelSteeringJointName);
  if (!this->flWheelSteeringJoint)
    gzthrow("could not find front left steering joint\n");

  std::string frWheelSteeringJointName = this->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel_steering");
  this->frWheelSteeringJoint = this->model->GetJoint(frWheelSteeringJointName);
  if (!this->frWheelSteeringJoint)
    gzthrow("could not find front right steering joint\n");


  // Put some deadband at the end of range for gas and brake pedals
  // and hand brake
  double jointCenter;
  this->gasPedalHigh  = this->gasPedalJoint->GetHighStop(0).Radian();
  this->gasPedalLow   = this->gasPedalJoint->GetLowStop(0).Radian();
  jointCenter = (this->gasPedalHigh + this->gasPedalLow) / 2.0;
  this->gasPedalHigh = jointCenter +
    (1 - this->jointDeadbandPercent) * (this->gasPedalHigh - jointCenter);
  this->gasPedalLow = jointCenter +
    (1 - this->jointDeadbandPercent) * (this->gasPedalLow - jointCenter);
  this->gasPedalRange   = this->gasPedalHigh - this->gasPedalLow;

  this->brakePedalHigh  = this->brakePedalJoint->GetHighStop(0).Radian();
  this->brakePedalLow   = this->brakePedalJoint->GetLowStop(0).Radian();
  jointCenter = (this->brakePedalHigh + this->brakePedalLow) / 2.0;
  this->brakePedalHigh = jointCenter +
    (1 - this->jointDeadbandPercent) * (this->brakePedalHigh - jointCenter);
  this->brakePedalLow = jointCenter +
    (1 - this->jointDeadbandPercent) * (this->brakePedalLow - jointCenter);
  this->brakePedalRange   = this->brakePedalHigh - this->brakePedalLow;

  // get some vehicle parameters
  std::string paramName;
  double paramDefault;

  paramName = "front_torque";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->frontTorque = _sdf->Get<double>(paramName);
  else
    this->frontTorque = paramDefault;

  paramName = "back_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->backTorque = _sdf->Get<double>(paramName);
  else
    this->backTorque = paramDefault;

  paramName = "front_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->frontBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->frontBrakeTorque = paramDefault;

  paramName = "back_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->backBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->backBrakeTorque = paramDefault;

  paramName = "max_speed";
  paramDefault = 10;
  if (_sdf->HasElement(paramName))
    this->maxSpeed = _sdf->Get<double>(paramName);
  else
    this->maxSpeed = paramDefault;

  paramName = "max_steer";
  paramDefault = 0.6;
  if (_sdf->HasElement(paramName))
    this->maxSteer = _sdf->Get<double>(paramName);
  else
    this->maxSteer = paramDefault;

  paramName = "aero_load";
  paramDefault = 0.1;
  if (_sdf->HasElement(paramName))
    this->aeroLoad = _sdf->Get<double>(paramName);
  else
    this->aeroLoad = paramDefault;

  paramName = "min_brake_percent";
  paramDefault = 0.02;
  if (_sdf->HasElement(paramName))
    this->minBrakePercent = _sdf->Get<double>(paramName);
  else
    this->minBrakePercent = paramDefault;

  this->UpdateHandWheelRatio();

  // Simulate braking using joint stops with stop_erp = 0
  this->flWheelJoint->SetHighStop(0, 0);
  this->frWheelJoint->SetHighStop(0, 0);
  this->blWheelJoint->SetHighStop(0, 0);
  this->brWheelJoint->SetHighStop(0, 0);

  this->flWheelJoint->SetLowStop(0, 0);
  this->frWheelJoint->SetLowStop(0, 0);
  this->blWheelJoint->SetLowStop(0, 0);
  this->brWheelJoint->SetLowStop(0, 0);

  // stop_erp == 0 means no position correction torques will act
  this->flWheelJoint->SetAttribute("stop_erp", 0, 0.0);
  this->frWheelJoint->SetAttribute("stop_erp", 0, 0.0);
  this->blWheelJoint->SetAttribute("stop_erp", 0, 0.0);
  this->brWheelJoint->SetAttribute("stop_erp", 0, 0.0);

  // stop_cfm == 10 means the joints will initially have small damping
  this->flWheelJoint->SetAttribute("stop_cfm", 0, 10.0);
  this->frWheelJoint->SetAttribute("stop_cfm", 0, 10.0);
  this->blWheelJoint->SetAttribute("stop_cfm", 0, 10.0);
  this->brWheelJoint->SetAttribute("stop_cfm", 0, 10.0);

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id = 0;
  this->flWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->flWheelJoint->GetChild()->GetCollision(id));
  this->frWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->frWheelJoint->GetChild()->GetCollision(id));
  this->blWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->blWheelJoint->GetChild()->GetCollision(id));
  this->brWheelRadius = DRCVehiclePlugin::get_collision_radius(
                          this->brWheelJoint->GetChild()->GetCollision(id));
  // gzerr << this->flWheelRadius << " " << this->frWheelRadius << " "
  //       << this->blWheelRadius << " " << this->brWheelRadius << "\n";

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  math::Vector3 flCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->flWheelJoint->GetChild(), id);
  math::Vector3 frCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->frWheelJoint->GetChild(), id);
  math::Vector3 blCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->blWheelJoint->GetChild(), id);
  math::Vector3 brCenterPos = DRCVehiclePlugin::get_collision_position(
                                this->brWheelJoint->GetChild(), id);
  // track widths are computed first
  math::Vector3 vec3 = flCenterPos - frCenterPos;
  frontTrackWidth = vec3.GetLength();
  vec3 = flCenterPos - frCenterPos;
  backTrackWidth = vec3.GetLength();
  // to compute wheelbase, first position of axle centers are computed
  math::Vector3 frontAxlePos = (flCenterPos + frCenterPos) / 2;
  math::Vector3 backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  wheelbaseLength = vec3.GetLength();
  // gzerr << wheelbaseLength << " " << frontTrackWidth
  //       << " " << backTrackWidth << "\n";

  // initialize controllers for car
  /// \TODO: move PID parameters into SDF
  this->gasPedalPID.Init(800, 0, 0, 0, 0,
                         this->pedalForce, -this->pedalForce);
  this->brakePedalPID.Init(800, 0, 0, 0, 0,
                         this->pedalForce, -this->pedalForce);
  this->handWheelPID.Init(100, 0, 0, 0, 0,
                         this->handWheelForce, -this->handWheelForce);
  this->flWheelSteeringPID.Init(500, 0, 50, 0, 0,
                         this->steeredWheelForce, -this->steeredWheelForce);
  this->frWheelSteeringPID.Init(500, 0, 50, 0, 0,
                         this->steeredWheelForce, -this->steeredWheelForce);

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DRCVehiclePlugin::UpdateStates, this));

  this->lastTime = this->world->GetSimTime();
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCVehiclePlugin::UpdateStates()
{
  this->handWheelState = this->handWheelJoint->GetAngle(0).Radian();
  this->brakePedalState = this->brakePedalJoint->GetAngle(0).Radian();
  this->gasPedalState = this->gasPedalJoint->GetAngle(0).Radian();
  this->flSteeringState = this->flWheelSteeringJoint->GetAngle(0).Radian();
  this->frSteeringState = this->frWheelSteeringJoint->GetAngle(0).Radian();

  this->flWheelState = this->flWheelJoint->GetVelocity(0);
  this->frWheelState = this->frWheelJoint->GetVelocity(0);
  this->blWheelState = this->blWheelJoint->GetVelocity(0);
  this->brWheelState = this->brWheelJoint->GetVelocity(0);

  math::Vector3 linVel = this->model->GetRelativeLinearVel();
  math::Vector3 angVel = this->model->GetRelativeAngularVel();

  common::Time curTime = this->world->GetSimTime();
  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    // PID (position) steering
    double steerError = this->handWheelState - this->handWheelCmd;
    double steerCmd = this->handWheelPID.Update(steerError, dt);
    this->handWheelJoint->SetForce(0, steerCmd);

    // Bi-stable switching of FNR switch reference point

    // PID (position) gas pedal
    double gasError = this->gasPedalState - this->gasPedalCmd;
    double gasCmd = this->gasPedalPID.Update(gasError, dt);
    this->gasPedalJoint->SetForce(0, gasCmd);

    // PID (position) brake pedal
    double brakeError = this->brakePedalState - this->brakePedalCmd;
    double brakeCmd = this->brakePedalPID.Update(brakeError, dt);
    this->brakePedalJoint->SetForce(0, brakeCmd);

    // PID (position) steering joints based on steering position
    // Ackermann steering geometry here
    //  \TODO provide documentation for these equations
    double tanSteer = tan(this->handWheelState * this->steeringRatio);
    this->flWheelSteeringCmd = atan2(tanSteer,
        1 - frontTrackWidth/2/wheelbaseLength * tanSteer);
    this->frWheelSteeringCmd = atan2(tanSteer,
        1 + frontTrackWidth/2/wheelbaseLength * tanSteer);
    // this->flWheelSteeringCmd = this->handWheelState * this->steeringRatio;
    // this->frWheelSteeringCmd = this->handWheelState * this->steeringRatio;

    double flwsError =  this->flSteeringState - this->flWheelSteeringCmd;
    double flwsCmd = this->flWheelSteeringPID.Update(flwsError, dt);
    this->flWheelSteeringJoint->SetForce(0, flwsCmd);

    double frwsError = this->frSteeringState - this->frWheelSteeringCmd;
    double frwsCmd = this->frWheelSteeringPID.Update(frwsError, dt);
    this->frWheelSteeringJoint->SetForce(0, frwsCmd);

    // Let SDF parameters specify front/rear/all-wheel drive.

    // Gas pedal torque.
    // Map gas torques to individual wheels.
    // Cut off gas torque at a given wheel if max speed is exceeded.
    // Use directionState to determine direction of applied torque.
    // Note that definition of DirectionType allows multiplication to determine
    // torque direction.
    double gasPercent = this->GetGasPedalPercent();
    double gasMultiplier = this->GetGasTorqueMultiplier();
    double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
    // Apply equal torque at left and right wheels, which is an implicit model
    // of the differential.
    if ((fabs(this->flWheelState * this->flWheelRadius) < this->maxSpeed)
      && (fabs(this->frWheelState * this->frWheelRadius) < this->maxSpeed))
    {
      flGasTorque = gasPercent*this->frontTorque * gasMultiplier;
      frGasTorque = gasPercent*this->frontTorque * gasMultiplier;
    }
    if ( (fabs(this->blWheelState * this->blWheelRadius) < this->maxSpeed)
      && (fabs(this->brWheelState * this->brWheelRadius) < this->maxSpeed))
    {
      blGasTorque = gasPercent*this->backTorque * gasMultiplier;
      brGasTorque = gasPercent*this->backTorque * gasMultiplier;
    }

    // Brake pedal, hand-brake torque.
    // Compute percents and add together, saturating at 100%
    double brakePercent = this->GetBrakePedalPercent();
    brakePercent = math::clamp(brakePercent, this->minBrakePercent, 1.0);
    // Map brake torques to individual wheels.
    // Apply brake torque in opposition to wheel spin direction.
    double flBrakeTorque, frBrakeTorque, blBrakeTorque, brBrakeTorque;
    // Below the smoothing speed in rad/s, reduce applied brake torque
    double smoothingSpeed = 0.5;
    flBrakeTorque = -brakePercent*this->frontBrakeTorque *
      math::clamp(this->flWheelState / smoothingSpeed, -1.0, 1.0);
    frBrakeTorque = -brakePercent*this->frontBrakeTorque *
      math::clamp(this->frWheelState / smoothingSpeed, -1.0, 1.0);
    blBrakeTorque = -brakePercent*this->backBrakeTorque *
      math::clamp(this->blWheelState / smoothingSpeed, -1.0, 1.0);
    brBrakeTorque = -brakePercent*this->backBrakeTorque *
      math::clamp(this->brWheelState / smoothingSpeed, -1.0, 1.0);

    // Lock wheels if high braking applied at low speed
    if (brakePercent > 0.7 && fabs(this->flWheelState) < smoothingSpeed)
      this->flWheelJoint->SetAttribute("stop_cfm", 0, 0.0);
    else
      this->flWheelJoint->SetAttribute("stop_cfm", 0, 1.0);

    if (brakePercent > 0.7 && fabs(this->frWheelState) < smoothingSpeed)
      this->frWheelJoint->SetAttribute("stop_cfm", 0, 0.0);
    else
      this->frWheelJoint->SetAttribute("stop_cfm", 0, 1.0);

    if (brakePercent > 0.7 && fabs(this->blWheelState) < smoothingSpeed)
      this->blWheelJoint->SetAttribute("stop_cfm", 0, 0.0);
    else
      this->blWheelJoint->SetAttribute("stop_cfm", 0, 1.0);

    if (brakePercent > 0.7 && fabs(this->brWheelState) < smoothingSpeed)
      this->brWheelJoint->SetAttribute("stop_cfm", 0, 0.0);
    else
      this->brWheelJoint->SetAttribute("stop_cfm", 0, 1.0);

    this->flWheelJoint->SetForce(0, flGasTorque + flBrakeTorque);
    this->frWheelJoint->SetForce(0, frGasTorque + frBrakeTorque);
    this->blWheelJoint->SetForce(0, blGasTorque + blBrakeTorque);
    this->brWheelJoint->SetForce(0, brGasTorque + brBrakeTorque);

    // gzerr << "steer [" << this->handWheelState
    //       << "] range [" << this->handWheelRange
    //       << "] l [" << linVel
    //       << "] a [" << angVel
    //       << "] gas [" << this->gasPedalState
    //       << "] gas [" << gasCmd
    //       << "] brake [" << this->brakePedalState
    //       << "] brake [" << brakeCmd
    //       << "] bl gas [" << blGasTorque
    //       << "] bl brake [" << blBrakeTorque << "]\n";
    this->lastTime = curTime;
  }
  else if (dt < 0)
  {
    // has time been reset?
    this->lastTime = curTime;
  }
}

////////////////////////////////////////////////////////////////////////////////
// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double DRCVehiclePlugin::get_collision_radius(physics::CollisionPtr _coll)
{
  if (!_coll || !(_coll->GetShape()))
    return 0;
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(_coll->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// function that extracts the position of the collision object specified by _id
math::Vector3 DRCVehiclePlugin::get_collision_position(physics::LinkPtr _link,
                                                       unsigned int _id)
{
  if (!_link || !(_link->GetCollision(_id)))
    return math::Vector3::Zero;
  math::Pose pose = _link->GetCollision(_id)->GetWorldPose();
  return pose.pos;
}

GZ_REGISTER_MODEL_PLUGIN(DRCVehiclePlugin)
}
