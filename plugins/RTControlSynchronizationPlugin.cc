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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <condition_variable>
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
#include "plugins/RTControlSynchronizationPlugin.hh"

#define MAX_MOTORS 255

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RTControlSynchronizationPlugin)

// Private data class
class gazebo::RTControlSynchronizationPluginPrivate
{
  ////////////////////////////////////////////////////////////////////////////
  //                                                                        //
  //                                                                        //
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////
  //                                                                        //
  //  Controller Synchronization Control                                    //
  //                                                                        //
  ////////////////////////////////////////////////////////////////////////////
  class Command
  {
    public: common::Time timestamp;
  };
  public: Command lastCommand;

  /// \brief enforce delay policy
  public: void EnforceSynchronizationDelay(const common::Time &_curTime,
    int _timeoutMs)
  {
    if (_timeoutMs != 0)
    {
      common::Time curWallTime = common::Time::GetWallTime();
      // on initial entry, this->delayWindowStartTime = 0 sim time
      if (curWallTime >= this->delayWindowStartTime + this->delayWindowSize)
      {
        this->delayWindowStartTime = curWallTime;
        this->delayInWindow = common::Time(0.0);
      }

      gzdbg << "we have delayed this much time in current window: ["
            << this->delayInWindow.Double()
            << " / " <<  this->delayMaxPerWindow.Double() << "]\n";

      common::Time delayInStepSum(0.0);
      if (this->delayInWindow < this->delayMaxPerWindow)
      {

        while (delayInStepSum < this->delayMaxPerStep &&
               this->delayInWindow < this->delayMaxPerWindow)
        {
          gzdbg << "we have delayed this much time in current step: ["
                << delayInStepSum.Double()
                << " / " <<  this->delayMaxPerStep.Double()
                << "] total delay in window ["
                << this->delayInWindow.Double()
                << " / " <<  this->delayMaxPerWindow.Double() << "]\n";

          std::unique_lock<std::mutex> lock(this->mutex);

          // compute sim time age of last received command
          double age = _curTime.Double() - this->lastCommand.timestamp.Double();

          printf("age %f sec sim time. lastCommand stamp %f sec."
                 "timeout %f sec\n", age,
                this->lastCommand.timestamp.Double(), 0.001*_timeoutMs);
          fflush(stdout);

          // if age is small enough, skip, otherwise, wait finite amount
          // for command messages to catchup.
          if (age <= 0.001 * _timeoutMs)
            break;

          gzdbg << "age[" << age << "] > timeoutMs["
                << 0.001*_timeoutMs << "]\n";

          // calculate amount of time to wait based on rules
          // std::chrono::time_point<std::chrono::high_resolution_clock,
          //                         std::chrono::milliseconds>
          std::chrono::system_clock::time_point
            now = std::chrono::system_clock::now();

          long int delayUsInt = floor(1e6*std::min(
              (this->delayMaxPerStep - delayInStepSum).Double(),
              (this->delayMaxPerWindow - this->delayInWindow).Double()));
          // delayUsInt = 1000;
          std::chrono::microseconds delay(delayUsInt);

          std::chrono::time_point<std::chrono::system_clock> timeout =
            now + delay;

          gzdbg << "delayUsInt: " << delayUsInt << "\n";
          // gzdbg << "timeout: " << delayUsInt << "\n";

          // common::Time tmp(std::detail::get_timespec(timeout));
          // printf("timeout %f wall %f min(%f, %f)\n",
          //     tmp.Double(), delayTime.Double(),
          //     (this->delayMaxPerStep - delayInStepSum).Double(),
          //     (this->delayMaxPerWindow - this->delayInWindow).Double());

          // convert now to common::Time
          // common::Time delayTime(std::chrono::get_timespec(now));
          long int nowNano =
            std::chrono::time_point_cast<std::chrono::nanoseconds>(
                   now).time_since_epoch().count();
          gzdbg << "time_since_epoc nanoseconds: now: "
                << nowNano << "\n";
          timespec tv;
          tv.tv_nsec  = static_cast<int>(nowNano % 1000000000);
          tv.tv_sec = static_cast<int>((nowNano-tv.tv_nsec)/1000000000);
          common::Time delayTime(tv);
          gzdbg << "time_since_epoc sec: now: "
                << delayTime.Double() << "\n";

          gzdbg << "--------------- wait_until ---------------\n";
          if (this->delayCondition.wait_until(lock, timeout) !=
              std::cv_status::timeout)
          {
            delayTime = common::Time::GetWallTime() - delayTime;
            if ((this->delayInWindow >= this->delayMaxPerWindow) ||
                (delayInStepSum >= this->delayMaxPerStep))
            {
              gzdbg << "controller synchronization: "
                    << "phew, got command in time, but budget exhausted,"
                    << " simulation stops waiting for incoming command.\n";
            }
            // else
            {
              gzdbg << "controller synchronization: "
                    << "got message within specified duration window,"
                    << " all good.\n";
            }

            // printf("sim %f timed out with %f delayed %f\n",
            //       _curTime.Double()*1000,
            //       this->command.header.stamp.toSec()*1000,
            //       delayTime.Double()*1000);
            // fflush(stdout);
          }
          else
          {
            delayTime = common::Time::GetWallTime() - delayTime;
            gzwarn << "controller synchronization timeout: "
                   << "waited full duration budget [" << delayTime.Double()
                   << "] but timed_wait returned true.\n";
            if (delayTime >= this->delayMaxPerStep)
            {
            }
            else
            {
            }
            // printf("nsim %f otified with %f delayed %f\n",
            //       _curTime.Double()*1000,
            //       this->command.header.stamp.toSec()*1000,
            //       delayTime.Double()*1000);
            // fflush(stdout);
          }

          delayInStepSum += delayTime;
          this->delayInWindow += delayTime;

          printf("finished wait step delay(%f) window delay(%f)\n",
            delayInStepSum.Double(),
            this->delayInWindow.Double());

          // printf(" after (%f, %f)\n",
          //   delayInStepSum.Double(),
          //   this->delayInWindow.Double());
        }
        // printf(" out of while (%f < %f) (%f < %f)\n",
        //   delayInStepSum.Double(), this->delayMaxPerStep.Double(),
        //   this->delayInWindow.Double(), this->delayMaxPerWindow.Double());
      }

      gzdbg << "************* exiting update *************\n";
      /*
      this->delayStatistics.delay_in_step = delayInStepSum.Double();
      this->delayStatistics.delay_in_window = this->delayInWindow.Double();
      this->delayStatistics.delay_window_remain =
        ((this->delayWindowStartTime + this->delayWindowSize) -
         curWallTime).Double();
      this->pubDelayStatisticsQueue->push(
        this->delayStatistics, this->pubDelayStatistics);
      */
    }
  }

  public: void PublishConstrollerStatistics(const common::Time &_curTime)
  {
    /*
    /// publish controller statistics diagnostics, damages, etc.
    if (this->controllerStatsConnectCount > 0)
    {
      if ((_curTime - this->lastControllerStatisticsTime).Double() >=
        1.0/this->statsUpdateRate)
      {
        control_stats_msgs::ControllerStatistics msg;
        msg.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);
        msg.command_age = this->commandAge;
        msg.command_age_mean = this->commandAgeMean;
        msg.command_age_variance = this->commandAgeVariance /
          (this->commandAgeBuffer.size() - 1);
        msg.command_age_window_size = this->commandAgeBufferDuration;

        this->pubControllerStatisticsQueue->push(msg,
          this->pubControllerStatistics);
        this->lastControllerStatisticsTime = _curTime;
      }
    }
    */
  }

  /// \brief Special topic for advancing simulation by a ROS topic.
  public: gazebo::transport::SubscriberPtr subTic;

  /// \brief Condition variable for tic-ing simulation step.
  public: std::condition_variable delayCondition;

  /// \brief a non-moving window is used, every delayWindowSize-seconds
  /// the user is allotted delayMaxPerWindow seconds of delay budget.
  public: common::Time delayWindowSize;

  /// \brief Marks the start of a non-moving delay window.
  public: common::Time delayWindowStartTime;

  /// \brief Within each window, simulation will wait at
  /// most a total of delayMaxPerWindow seconds.
  public: common::Time delayMaxPerWindow;

  /// \brief Within each simulation step, simulation will wait at
  /// most delayMaxPerStep seconds to receive information from controller.
  public: common::Time delayMaxPerStep;

  /// \brief Within each window, simulation will wait at
  /// most a total of delayMaxPerWindow seconds.
  public: common::Time delayInWindow;

  ////////////////////////////////////////////////////////////////////////////
  //                                                                        //
  //  controller staticstics                                                //
  //                                                                        //
  ////////////////////////////////////////////////////////////////////////////

  /// \brief Publish controller synchronization delay information.
  /*
  public: gazebo::transport::Publisher pubDelayStatistics;
  public: PubQueue<control_stats_msgs::SynchronizationStatistics>::Ptr
    pubDelayStatisticsQueue;
  public: control_stats_msgs::SynchronizationStatistics delayStatistics;
  public: common::Time lastControllerStatisticsTime;
  */

  /// \brief Keep track of number of controller stats connections
  // public: int controllerStatsConnectCount;

  /// \brief Mutex to protect controllerStatsConnectCount.
  // public: std::mutex statsConnectionMutex;

  ////////////////////////////////////////////////////////////////////////////
  //                                                                        //
  //  connect to gazebo                                                     //
  //                                                                        //
  ////////////////////////////////////////////////////////////////////////////
  /// \brief Pointer to the update event connection.
  public: event::ConnectionPtr updateConnection;

  /// \brief Pointer to the model;
  public: physics::ModelPtr model;

  /// \brief keep track of controller update sim-time.
  public: gazebo::common::Time lastUpdateTime;

  /// \brief Controller update mutex.
  public: std::mutex mutex;
};

////////////////////////////////////////////////////////////////////////////////
RTControlSynchronizationPlugin::RTControlSynchronizationPlugin()
  : dataPtr(new RTControlSynchronizationPluginPrivate)
{
  // default control synchronization delay settings
  // to trigger synchronization delay, set
  // timeoutMs to non-zero
  this->timeoutMs = 1.0;

  // synchronization variables
  // rolling window for keeping track of timing enforcement and statistics
  this->dataPtr->delayWindowSize = common::Time(5.0);
  // max amount of delay in the rolling window
  this->dataPtr->delayMaxPerWindow = common::Time(5.0);
  // allowed jitter per simulation step
  this->dataPtr->delayMaxPerStep = common::Time(1.0);

  // beginning of the delay window in sim time
  this->dataPtr->delayWindowStartTime = common::Time(0.0);
  // current delay duration in the window (in sim time)
  this->dataPtr->delayInWindow = common::Time(0.0);
  // subscribers to the control stats
  // this->dataPtr->controllerStatsConnectCount = 0;
  this->dataPtr->lastCommand.timestamp = common::Time(0.0);
}

/////////////////////////////////////////////////
RTControlSynchronizationPlugin::~RTControlSynchronizationPlugin()
{
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "_model pointer is null");
  GZ_ASSERT(_sdf, "_sdf pointer is null");

  this->dataPtr->model = _model;

  // Connect to the update event.
  // This event is broadcast by gzserver on every simulation step.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RTControlSynchronizationPlugin::OnUpdate, this, _1));

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  std::string prefix = "~/" + this->dataPtr->model->GetName() + "/";

  // publish state
  gzdbg << "publishing stuff to [" << prefix + "state" << "/\n";
  this->statePub = this->node->Advertise<msgs::Any>(prefix + "state");

  // listen to incoming robot commands
  gzdbg << "subscribing to robot command on [" << prefix + "control" << "/\n";
  this->controlSub = this->node->Subscribe(prefix + "control",
    &RTControlSynchronizationPlugin::RobotCommandIn, this);
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gazebo::common::Time curTime = this->dataPtr->model->GetWorld()->GetSimTime();
  double dt = (curTime - this->dataPtr->lastUpdateTime).Double();
  this->dataPtr->lastUpdateTime = curTime;

  // Update the control surfaces and publish the new state.
  if (dt > 0.0)
  {
    gzdbg << "t[" << curTime.Double() << "]: RobotStateOut\n";
    this->RobotStateOut();
    gzdbg << "EnforceSynchronizationDelay(" << curTime.Double() << ", "
          << this->timeoutMs << " ms)\n";
    this->dataPtr->EnforceSynchronizationDelay(curTime, this->timeoutMs);
    gzdbg << "ApplyRobotCommandToSim\n";
    this->ApplyRobotCommandToSim(dt);
    gzdbg << "Publish stats\n";
    this->dataPtr->PublishConstrollerStatistics(curTime);
  }
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::RobotStateOut()
{
  msgs::Any msg;
  msg.set_type(msgs::Any_ValueType_DOUBLE);
  msg.set_double_value(0);
  this->statePub->Publish(msg);
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::ApplyRobotCommandToSim(const double _dt)
{
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::RobotCommandIn(ConstAnyPtr &_msg)
{
  // receive robot command from the outside world and tick simulation
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // do whatever needed to be done here to receive the robot command
  // from the controller, usually copy to a local buffer
  this->ReceiveRobotCommand();

  this->dataPtr->lastCommand.timestamp =
    this->dataPtr->model->GetWorld()->GetSimTime();

  // tic simulation
  this->dataPtr->delayCondition.notify_all();
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::ReceiveRobotCommand()
{
  // to be overloaded by user
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::SendRobotState()
{
  // to be overloaded by user
}

/////////////////////////////////////////////////
void RTControlSynchronizationPlugin::Tic()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->delayCondition.notify_all();
}

