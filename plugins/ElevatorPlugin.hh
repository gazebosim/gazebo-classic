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

#include <sdf/sdf.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/util/system.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE ElevatorPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ElevatorPlugin();

    /// \brief Destructor.
    public: ~ElevatorPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private: void Update();

    private: void OnElevator(ConstGzStringPtr &_msg);

    /// \brief World pointer.
    private: physics::ModelPtr model;
    private: physics::JointPtr liftJoint;
    private: physics::JointPtr doorJoint;

    /// \brief SDF pointer.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr elevatorSub;

    private: class DoorController
             {
               public: enum Target {OPEN, CLOSE};
               public: enum State {MOVING, STATIONARY};

               public: DoorController(physics::JointPtr _doorJoint);
               public: void SetTarget(
                           ElevatorPlugin::DoorController::Target _target);

               public: ElevatorPlugin::DoorController::State GetState() const;
               public: ElevatorPlugin::DoorController::Target GetTarget() const;

               public: virtual bool Update();

               public: physics::JointPtr doorJoint;
               public: State state;
               public: Target target;
               public: common::PID doorPID;
             };

    private: class LiftController
             {
               public: enum State {MOVING, STATIONARY};
               public: LiftController(physics::JointPtr _liftJoint);
               public: void SetFloor(int _floor);
               public: int GetFloor() const;
               public: ElevatorPlugin::LiftController::State GetState() const;

               public: virtual bool Update();

               public: State state;
               public: int floor;
               public: physics::JointPtr liftJoint;
               public: common::PID liftPID;
             };

    private: class State
             {
               public: State() : started(false) {}
               public: std::string name;
               public: virtual void Start() {}
               public: virtual bool Update() {return true;}
               protected: bool started;
             };

    private: class CloseState : public State
             {
               public: CloseState(ElevatorPlugin::DoorController *_ctrl);
               public: virtual void Start();
               public: virtual bool Update();
               public: ElevatorPlugin::DoorController *ctrl;
             };

    private: class OpenState : public State
             {
               public: OpenState(ElevatorPlugin::DoorController *_ctrl);
               public: virtual void Start();
               public: virtual bool Update();
               public: ElevatorPlugin::DoorController *ctrl;
             };

    private: class MoveState : public State
             {
               public: MoveState(int _floor, LiftController *_ctrl);
               public: virtual void Start();
               public: virtual bool Update();
               public: int floor;
               public: LiftController *ctrl;
             };

    private: class WaitState : public State
             {
               public: WaitState();
               public: virtual void Start();
               public: virtual bool Update();
               public: common::Time start;
             };

    private: DoorController *doorController;
    private: LiftController *liftController;

    private: std::list<State*> states;

    private: boost::mutex stateMutex;
  };
}
#endif
