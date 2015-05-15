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

#ifndef _GAZEBO_ELEVATOR_PLUGIN_HH_
#define _GAZEBO_ELEVATOR_PLUGIN_HH_

#include <mutex>
#include <string>
#include <list>

#include <sdf/sdf.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/util/system.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  /// \brief Plugin to control a elevator. This plugin will listen for
  /// door and lift events on a specified topic.
  /// \verbatim
  ///    <plugin filename="libSimEventsPlugin.so" name="event_plugin">
  /// \endverbatim
  class GAZEBO_VISIBLE ElevatorPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ElevatorPlugin();

    /// \brief Destructor.
    public: ~ElevatorPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the plugin once every iteration of simulation.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Reset the plugin
    public: virtual void Reset();

    /// \brief Receives messages on the elevator's topic.
    /// \param[in] _msg The string message that contains a command.
    private: void OnElevator(ConstGzStringPtr &_msg);

    /// \brief Pointer to the elevator model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint that lifts the elevator
    private: physics::JointPtr liftJoint;

    /// \brief Pointer to the joint that opens the door
    private: physics::JointPtr doorJoint;

    /// \brief SDF pointer.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Node for communication
    private: transport::NodePtr node;

    /// \brief Used to subscribe to command message. This will call the
    /// OnElevator function when a message arrives.
    private: transport::SubscriberPtr elevatorSub;

    /// \brief Controller for opening and closing the elevator door.
    private: class DoorController
             {
               /// \brief Door targets.
               public: enum Target {OPEN, CLOSE};

               /// \brief Door motion states
               public: enum State {MOVING, STATIONARY};

               /// \brief Constructor
               /// \param[in] _doorJoint Pointer to the joint that should be
               /// controlled.
               public: DoorController(physics::JointPtr _doorJoint);

               /// \brief Destructor
               public: virtual ~DoorController() = default;

               /// \brief Set the target for the door (OPEN or CLOSE).
               /// \param[in] _target The target for the door.
               public: void SetTarget(
                           ElevatorPlugin::DoorController::Target _target);

               /// \brief Get the current state.
               /// \return Current state.
               public: ElevatorPlugin::DoorController::State GetState() const;

               /// \brief Get the current target.
               /// \return Current target.
               public: ElevatorPlugin::DoorController::Target GetTarget() const;

               /// \brief Reset the controller
               public: void Reset();

               /// \brief Update the controller.
               /// \param[in] _info Update information provided by the server.
               /// \return True if the target has been reached.
               public: virtual bool Update(const common::UpdateInfo &_info);

               /// \brief Pointer to the door joint.
               public: physics::JointPtr doorJoint;

               /// \brief Current door state
               public: State state;

               /// \brief Current door target
               public: Target target;

               /// \brief PID controller for the door.
               public: common::PID doorPID;

               /// \brief Previous simulation time.
               public: common::Time prevSimTime;
             };

    /// \brief Controller for raising and lowering the elevator.
    private: class LiftController
             {
               /// \brief Lift stat
               public: enum State {MOVING, STATIONARY};

               /// \brief Constructor
               /// \param[in] _liftJoint Pointer to the joint that should be
               /// controlled.
               /// \param[in] _floorHeight Height of each floor.
               public: LiftController(physics::JointPtr _liftJoint,
                                      float _floorHeight);

               /// \brief Destructor
               public: virtual ~LiftController() = default;

               /// \brief Set the current floor to move to.
               /// \param[in] _floor Floor number.
               public: void SetFloor(int _floor);

               /// \brief Get the current floor.
               /// \return Floor number
               public: int GetFloor() const;

               /// \brief Get the current state.
               /// \return Current lift state.
               public: ElevatorPlugin::LiftController::State GetState() const;

               /// \brief Reset the controller
               public: void Reset();

               /// \brief Update the controller.
               /// \param[in] _info Update information provided by the server.
               /// \return True if the target has been reached.
               public: virtual bool Update(const common::UpdateInfo &_info);

               /// \brief State of the controller.
               public: State state;

               /// \brief Floor the elevator is on or moving to.
               public: int floor;

               /// \brief Height of each floor.
               public: float floorHeight;

               /// \brief Joint to control
               public: physics::JointPtr liftJoint;

               /// \brief PID controller.
               public: common::PID liftPID;

               /// \brief Previous simulation time.
               public: common::Time prevSimTime;
             };

    /// \brief State base class
    private: class State
             {
               /// \brief Constructor
               public: State() : started(false) {}

               /// \brief State name
               public: std::string name;

               /// \brief Used to start a state.
               public: virtual void Start() {}

               /// \brief Used to update a state.
               public: virtual bool Update() {return true;}

               /// \brief True when started.
               protected: bool started;
             };

    /// \brief State used to close the elevator door.
    private: class CloseState : public State
             {
               /// \brief Constructor.
               /// \param[in] _ctrl Elevator door controller
               public: CloseState(ElevatorPlugin::DoorController *_ctrl);

               // Documentation inherited
               public: virtual void Start();

               // Documentation inherited
               public: virtual bool Update();

               /// \brief Pointer to the door controller.
               public: ElevatorPlugin::DoorController *ctrl;
             };

    /// \brief State used to open the elevator door.
    private: class OpenState : public State
             {
               /// \brief Constructor.
               /// \param[in] _ctrl Elevator door controller
               public: OpenState(ElevatorPlugin::DoorController *_ctrl);

               // Documentation inherited
               public: virtual void Start();

               // Documentation inherited
               public: virtual bool Update();

               /// \brief Pointer to the door controller.
               public: ElevatorPlugin::DoorController *ctrl;
             };

    /// \brief State used to move the elevator to a floor.
    private: class MoveState : public State
             {
               /// \brief Constructor
               public: MoveState(int _floor, LiftController *_ctrl);

               // Documentation inherited
               public: virtual void Start();

               // Documentation inherited
               public: virtual bool Update();

               /// \brief Target floor number.
               public: int floor;

               /// \brief Lift controller.
               public: LiftController *ctrl;
             };

    /// \brief State used to make the elevetor wait.
    private: class WaitState : public State
             {
               /// \brief Constructor
               public: WaitState();

               // Documentation inherited
               public: virtual void Start();

               // Documentation inherited
               public: virtual bool Update();

               /// \brief Start time.
               public: common::Time start;
             };

    /// \brief Door controller.
    private: DoorController *doorController;

    /// \brief Lift controller.
    private: LiftController *liftController;

    /// \brief List of states that should be executed.
    private: std::list<State*> states;

    /// \brief Mutex to protect states.
    private: std::mutex stateMutex;
  };
}
#endif
