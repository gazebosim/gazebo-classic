/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_ELEVATOR_PLUGIN_PRIVATE_HH_
#define _GAZEBO_ELEVATOR_PLUGIN_PRIVATE_HH_

#include <list>
#include <mutex>
#include <string>

#include <sdf/sdf.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Timer.hh>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ElevatorPlugin
  class ElevatorPluginPrivate
  {
    /// \brief Constructor
    public: ElevatorPluginPrivate() = default;

    /// \brief Destructor
    public: virtual ~ElevatorPluginPrivate();

    /// \brief Controller for opening and closing the elevator door.
    public: class DoorController
    {
      /// \enum Target
      /// \brief Door targets.
      public: enum Target {
                /// \brief Open the door
                OPEN,

                /// \brief Close the door
                CLOSE
              };

      /// \enum State
      /// \brief Door motion states
      public: enum State {
                /// \brief The door is moving
                MOVING,

                /// \brief The door is stationary
                STATIONARY
              };

      /// \brief Constructor
      /// \param[in] _doorJoint Pointer to the joint that should be
      /// controlled.
      public: DoorController(physics::JointPtr _doorJoint);

      /// \brief Destructor
      public: virtual ~DoorController() = default;

      /// \brief Set the target for the door (OPEN or CLOSE).
      /// \param[in] _target The target for the door.
      public: void SetTarget(
                  ElevatorPluginPrivate::DoorController::Target _target);

      /// \brief Get the current state.
      /// \return Current state.
      public: ElevatorPluginPrivate::DoorController::State GetState() const;

      /// \brief Get the current target.
      /// \return Current target.
      public: ElevatorPluginPrivate::DoorController::Target GetTarget() const;

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
    public: class LiftController
    {
      /// \enum State
      /// \brief Lift state
      public: enum State {
                /// \brief The lift is moving
                MOVING,

                /// \brief The lift is stationary
                STATIONARY
              };

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
      public: ElevatorPluginPrivate::LiftController::State GetState() const;

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
    public: class State
    {
      /// \brief Constructor
      public: State() : started(false) {}

      /// \brief Destructor
      public: virtual ~State() = default;

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
    public: class CloseState : public State
    {
      /// \brief Constructor.
      /// \param[in] _ctrl Elevator door controller
      public: CloseState(ElevatorPluginPrivate::DoorController *_ctrl);

      // Documentation inherited
      public: virtual void Start();

      // Documentation inherited
      public: virtual bool Update();

      /// \brief Pointer to the door controller.
      public: ElevatorPluginPrivate::DoorController *ctrl;
    };

    /// \brief State used to open the elevator door.
    public: class OpenState : public State
    {
      /// \brief Constructor.
      /// \param[in] _ctrl Elevator door controller
      public: OpenState(ElevatorPluginPrivate::DoorController *_ctrl);

      // Documentation inherited
      public: virtual void Start();

      // Documentation inherited
      public: virtual bool Update();

      /// \brief Pointer to the door controller.
      public: ElevatorPluginPrivate::DoorController *ctrl;
    };

    /// \brief State used to move the elevator to a floor.
    public: class MoveState : public State
    {
      /// \brief Constructor
      /// \param[in] _floor Floor index to move to.
      /// \param[in] _ctrl Lift controller pointer.
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

    /// \brief State used to make the elevator wait.
    public: class WaitState : public State
    {
      /// \brief Constructor
      /// \param[in] _waitTime Length of the wait state
      public: WaitState(const common::Time &_waitTime);

      // Documentation inherited
      public: virtual void Start();

      // Documentation inherited
      public: virtual bool Update();

      /// \brief Timer to hold the door open.
      public: common::Timer waitTimer;
    };

    /// \brief Pointer to the elevator model.
    public: physics::ModelPtr model;

    /// \brief Pointer to the joint that lifts the elevator
    public: physics::JointPtr liftJoint;

    /// \brief Pointer to the joint that opens the door
    public: physics::JointPtr doorJoint;

    /// \brief SDF pointer.
    public: sdf::ElementPtr sdf;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief Node for communication
    public: transport::NodePtr node;

    /// \brief Used to subscribe to command message. This will call the
    /// OnElevator function when a message arrives.
    public: transport::SubscriberPtr elevatorSub;

    /// \brief Door controller.
    public: DoorController *doorController;

    /// \brief Lift controller.
    public: LiftController *liftController;

    /// \brief List of states that should be executed.
    public: std::list<State *> states;

    /// \brief Mutex to protect states.
    public: std::mutex stateMutex;

    /// \brief Time to hold the door in the open state.
    public: common::Time doorWaitTime;
  };
}
#endif
