/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <Simbody.h>

#include "UR10.h"

#include <iostream>

// Create our node for communication
gazebo::transport::NodePtr node;
gazebo::transport::PublisherPtr pub;

// task space classes
using namespace SimTK;
using namespace std;

//==============================================================================
//                             TASKS MEASURE
//==============================================================================
// This Measure is added to the modelRobot and is used to manage the tasks
// it is supposed to achieve and to return as its value the control torques
// that should be applied to the realRobot (that is, the simulated one).
// This should only be instantiated with T=Vector.
template <class T>
class TasksMeasure : public Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(TasksMeasure, Measure_<T>);

    TasksMeasure(UR10& modelRobot) 
    :   Measure_<T>(modelRobot.updForceSubsystem(), 
                    new Implementation(modelRobot),
                    AbstractMeasure::SetHandle()) {}


    const Vec3& getTarget() const { return getImpl().m_desiredTaskPosInGround; }
    Vec3& updTarget() { return updImpl().m_desiredTaskPosInGround; }
    void setTarget(Vec3 pos) { updImpl().m_desiredTaskPosInGround = pos; }

    void toggleGravityComp() {
        updImpl().m_compensateForGravity = !isGravityCompensationOn();}
    void toggleTask() {updImpl().m_controlTask = !getImpl().m_controlTask;}
    void toggleEndEffectorSensing() 
    {   updImpl().m_endEffectorSensing = !getImpl().m_endEffectorSensing;}

    bool isGravityCompensationOn() const 
    {   return getImpl().m_compensateForGravity; }
    bool isEndEffectorSensingOn() const 
    {   return getImpl().m_endEffectorSensing; }
    bool isTaskPointFollowingOn() const
    {   return getImpl().m_controlTask; }
    const Vec3& getTaskPointInEndEffector() const 
    {   return getImpl().m_taskPointInEndEffector; }

    SimTK_MEASURE_HANDLE_POSTSCRIPT(TasksMeasure, Measure_<T>);
};


template <class T>
class TasksMeasure<T>::Implementation : public Measure_<T>::Implementation {
public:
    Implementation(const UR10& modelRobot,
                   Vec3 taskPointInEndEffector=Vec3(0,0,0),
                   Real proportionalGain=225, double derivativeGain=30) 
    :   Measure_<T>::Implementation(T(UR10::NumCoords,NaN), 1),
        m_modelRobot(modelRobot),
        m_tspace1(m_modelRobot.getMatterSubsystem(), m_modelRobot.getGravity()),
        m_taskPointInEndEffector(taskPointInEndEffector),
        m_proportionalGain(proportionalGain),
        m_derivativeGain(derivativeGain),
        m_dampingGain(1),
        m_compensateForGravity(true),
        m_controlTask(true),
        m_endEffectorSensing(false),
        m_desiredTaskPosInGround(Vec3(-0.4, 0.1, 1)) // Z is up
    {       
        m_tspace1.addStationTask(m_modelRobot.getBody(UR10::EndEffector),
                         m_taskPointInEndEffector);
    }


    // Default copy constructor, destructor, copy assignment are fine.

    // Implementations of virtual methods.
    Implementation* cloneVirtual() const OVERRIDE_11
    {   return new Implementation(*this); }
    int getNumTimeDerivativesVirtual() const OVERRIDE_11 {return 0;}
    Stage getDependsOnStageVirtual(int order) const OVERRIDE_11
    {   return Stage::Velocity; }

    // This is the task space controller. It returns joint torques tau as the
    // value of the enclosing Measure.
    void calcCachedValueVirtual(const State& s, int derivOrder, T& tau) const
        OVERRIDE_11;

    // TaskSpace objects require some State resources; this call is the time
    // for doing that so forward on to the TaskSpace.
    void realizeMeasureTopologyVirtual(State& modelState) const OVERRIDE_11 {
        m_tspace1.realizeTopology(modelState);
    }
private:
friend class TasksMeasure<T>;

    const UR10&     m_modelRobot;
    TaskSpace       m_tspace1;

    const Vec3      m_taskPointInEndEffector;
    const Real      m_proportionalGain;
    const Real      m_derivativeGain;
    const Real      m_dampingGain;

    bool            m_compensateForGravity;
    bool            m_controlTask;
    bool            m_endEffectorSensing;
    Vec3            m_desiredTaskPosInGround;
};

//------------------------------------------------------------------------------
//                TASKS MEASURE :: CALC CACHED VALUE VIRTUAL
//------------------------------------------------------------------------------
// Given a modelState that has been updated from the real robot's sensors, 
// generate control torques as the TasksMeasure's value. This is the only part
// of the code that is actually doing task space operations.

template <class T>
void TasksMeasure<T>::Implementation::calcCachedValueVirtual
   (const State& modelState, int derivOrder, T& tau) const
{
    SimTK_ASSERT1_ALWAYS(derivOrder==0,
        "TasksMeasure::Implementation::calcCachedValueVirtual():"
        " derivOrder %d seen but only 0 allowed.", derivOrder);

    // Shorthands.
    // -----------
    const TaskSpace& p1 = m_tspace1;

    const int nu = tau.size();

    const Real& kd = m_derivativeGain;
    const Real& kp = m_proportionalGain;
    const Vec3& x1_des = m_desiredTaskPosInGround;

    // Abbreviate model state for convenience.
    const State& ms = modelState;

    // Compute control law in task space (F*).
    // ---------------------------------------
    Vec3 xd_des(0);
    Vec3 xdd_des(0);

    // Get info about the actual location, etc. of the model robot.
    Vec3 x1, x1d;
    p1.findStationLocationAndVelocityInGround(ms,
            TaskSpace::StationTaskIndex(0),
            m_taskPointInEndEffector, x1, x1d);
    std::cout << "debug x1: " << x1 << " x1d: " <<  x1d << "\n";

    if (m_endEffectorSensing)
        x1 = m_modelRobot.getSampledEndEffectorPos(ms);


    // Units of acceleration.
    Vec3 Fstar1 = xdd_des + kd * (xd_des - x1d) + kp * (x1_des - x1);

    // Compute task-space force that achieves the task-space control.
    // F = Lambda Fstar + p
    Vector F1 = p1.Lambda(ms) * Fstar1 + p1.mu(ms) + p1.p(ms);
    //Vector F2 = p2.calcInverseDynamics(ms, Fstar2);

    // Combine the reaching task with the gravity compensation and nullspace
    // damping.
    // Gamma = J1T F1 + N1T J1T F2 + N1T N2T (g - c u)
    const Vector& u = ms.getU();
    const Real c = m_dampingGain/2;
    tau.setToZero();
    //tau +=   p1.JT(s) * F1 
    //              + p1.NT(s) * (  p2.JT(s) * F2 
    //                            + p2.NT(s) * (p1.g(s) - c * u));


    if (m_controlTask) {
        tau += p1.JT(ms) * F1;
        if (m_compensateForGravity)
            tau += p1.NT(ms) * p1.g(ms);
        tau -= p1.NT(ms) * (c*u); // damping
    } else if (m_compensateForGravity) {
        tau += p1.g(ms) - (c*u);
    } else 
        tau -= c*u;

    UR10::clampToLimits(tau);
}

// global for now
UR10                 m_modelRobot;
TasksMeasure<Vector> m_modelTasks(m_modelRobot);
State                m_modelState;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstControlRequestPtr &_msg)
{
  // Dump the message contents to stdout.
  std::cout << _msg->DebugString();

  // get request data
  for (int i=0; i < UR10::NumCoords; ++i) {
      const UR10::Coords coord = UR10::Coords(i);
      m_modelRobot.setJointAngle(m_modelState, coord, _msg->joint_pos(i));
      m_modelRobot.setJointRate(m_modelState, coord, _msg->joint_vel(i));
  }

  // get target
  Vec3 target_pos(_msg->target_pos().x(),
                  _msg->target_pos().y(),
                  _msg->target_pos().z());
  m_modelTasks.setTarget(target_pos);

  // Optional: if real robot end effector location can be sensed, it can
  // be used to improve accuracy. Otherwise, estimate the end effector
  // location using the model robot.
  // const Vec3 sensedEEPos = 
  //     m_realRobot.getSampledEndEffectorPos(realState); // get from _msg
  // m_modelRobot.setSampledEndEffectorPos(m_modelState, sensedEEPos);

  m_modelRobot.realize(m_modelState, Stage::Velocity);
  const Vector& tau = m_modelTasks.getValue(m_modelState);

  // compute joint torques

  // publish joint torques
  gazebo::msgs::ControlResponse res;

  res.set_name("response");
  res.clear_torques();
  for (unsigned int i = 0; i < _msg->joint_pos().size(); ++i)
  {
    std::cout << i << " : " << tau[i] << "\n";
    res.add_torques(tau[i]);
  }
  pub->Publish(res);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::setupClient(_argc, _argv);

  node.reset(new gazebo::transport::Node());
  node->Init();
  pub = node->Advertise<gazebo::msgs::ControlResponse>("~/ur10/control_response");

  // initialize model state
  m_modelRobot.initialize(m_modelState);

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/ur10/control_request", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
  {
    gazebo::common::Time::MSleep(1000);
    std::cout << ".";
  }

  // Make sure to shut everything down.
  gazebo::shutdown();
}
