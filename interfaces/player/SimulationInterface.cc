/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <time.h>

#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "player.h"

#include "gazebo/transport/transport.hh"
#include "gazebo/gazebo_client.hh"

#include "GazeboTime.hh"
#include "GazeboDriver.hh"
#include "SimulationInterface.hh"

boost::recursive_mutex *SimulationInterface::mutex = NULL;

extern PlayerTime* GlobalTime;

//////////////////////////////////////////////////
// Constructor
SimulationInterface::SimulationInterface(player_devaddr_t _addr,
    GazeboDriver *_driver, ConfigFile *_cf, int _section)
: GazeboInterface(_addr, _driver, _cf, _section)
{
  gazebo::client::setup();

  worldName = _cf->ReadString(_section, "world_name", "default");

  // steal the global clock - a bit aggressive, but a simple approach
  if (GlobalTime)
  {
    delete GlobalTime;
    GlobalTime = NULL;
  }

  GlobalTime = new GazeboTime();
  assert(GlobalTime != 0);

  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->worldName);
  this->statsSub =
  this->node->Subscribe("~/world_stats", &SimulationInterface::OnStats, this);

  this->modelPub = this->node->Advertise<gazebo::msgs::Model>("~/model/modify");

  this->responseQueue = NULL;

  memset(&this->pose3dReq, 0, sizeof(this->pose3dReq));
  memset(&this->pose2dReq, 0, sizeof(this->pose2dReq));

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
}

//////////////////////////////////////////////////
// Destructor
SimulationInterface::~SimulationInterface()
{
  gazebo::client::shutdown();
  if (this->responseQueue)
  {
    delete this->responseQueue;
    this->responseQueue = NULL;
  }
}

//////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int SimulationInterface::ProcessMessage(QueuePointer &_respQueue,
                                        player_msghdr_t *_hdr, void *_data)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->responseQueue)
    delete this->responseQueue;

  this->responseQueue = new QueuePointer(_respQueue);

  /// Set a 3D pose
  if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
                            PLAYER_SIMULATION_REQ_SET_POSE3D,
                            this->device_addr))
  {
    player_simulation_pose3d_req_t *req =
      static_cast<player_simulation_pose3d_req_t*>(_data);

    gazebo::math::Pose pose(
        gazebo::math::Vector3(req->pose.px, req->pose.py, req->pose.pz),
        gazebo::math::Quaternion(req->pose.proll, req->pose.ppitch,
                                 req->pose.pyaw));

    gazebo::msgs::Model msg;
    msg.set_name(req->name);
    gazebo::msgs::Set(msg.mutable_pose(), pose.Ign());
    this->modelPub->Publish(msg);

    this->driver->Publish(this->device_addr, _respQueue,
                          PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_SIMULATION_REQ_SET_POSE3D);
  }

  /// Set a 2D pose
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_SET_POSE2D,
                                 this->device_addr))
  {
    player_simulation_pose2d_req_t *req =
      static_cast<player_simulation_pose2d_req_t*>(_data);

    gazebo::math::Pose pose(
        gazebo::math::Vector3(req->pose.px, req->pose.py, 0),
        gazebo::math::Quaternion(0, 0, req->pose.pa));

    gazebo::msgs::Model msg;
    msg.set_name(req->name);
    gazebo::msgs::Set(msg.mutable_pose(), pose.Ign());
    this->modelPub->Publish(msg);

    this->driver->Publish(this->device_addr, _respQueue,
                          PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_SIMULATION_REQ_SET_POSE2D);
  }

  /// Get a 3d pose
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_GET_POSE3D,
                                 this->device_addr))
  {
    player_simulation_pose3d_req_t *req =
      static_cast<player_simulation_pose3d_req_t*>(_data);

    std::map<std::string, gazebo::math::Pose>::iterator iter;

    iter = this->entityPoses.find(req->name);
    if (iter != this->entityPoses.end())
    {
      snprintf(this->pose3dReq.name, strlen(this->pose3dReq.name),
          "%s", req->name);
      this->pose3dReq.name_count = strlen(this->pose3dReq.name);

      this->pose3dReq.pose.px = iter->second.pos.x;
      this->pose3dReq.pose.py = iter->second.pos.y;
      this->pose3dReq.pose.pz = iter->second.pos.z;

      this->pose3dReq.pose.proll = iter->second.rot.GetAsEuler().x;
      this->pose3dReq.pose.ppitch = iter->second.rot.GetAsEuler().y;
      this->pose3dReq.pose.pyaw = iter->second.rot.GetAsEuler().z;
    }

    this->driver->Publish(this->device_addr, *(this->responseQueue),
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_SIMULATION_REQ_GET_POSE3D,
        &this->pose3dReq, sizeof(this->pose3dReq), NULL);
  }

  /// Get a 2D pose
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_GET_POSE2D,
                                 this->device_addr))
  {
    player_simulation_pose2d_req_t *req =
      static_cast<player_simulation_pose2d_req_t*>(_data);

    std::map<std::string, gazebo::math::Pose>::iterator iter;

    iter = this->entityPoses.find(req->name);
    if (iter != this->entityPoses.end())
    {
      snprintf(this->pose3dReq.name, strlen(this->pose3dReq.name),
          "%s", req->name);
      this->pose3dReq.name_count = strlen(this->pose3dReq.name);

      this->pose2dReq.pose.px = iter->second.pos.x;
      this->pose2dReq.pose.py = iter->second.pos.y;
      this->pose2dReq.pose.pa = iter->second.rot.GetAsEuler().z;
    }
    this->driver->Publish(this->device_addr, *(this->responseQueue),
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_SIMULATION_REQ_GET_POSE2D,
        &this->pose2dReq, sizeof(this->pose2dReq), NULL);
  }
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_GET_PROPERTY,
                                 this->device_addr))
  {
    player_simulation_property_req_t *req =
      static_cast<player_simulation_property_req_t*>(_data);

    std::string name = req->name;
    std::string prop = req->prop;

    if (name == "world")
    {
      req->value = new char[sizeof(this->simTime)];
      req->value_count = sizeof(this->simTime);

      if (prop == "sim_time")
      {
        memcpy(req->value, &this->simTime, sizeof(this->simTime));
      }
      else if (prop == "pause_time")
      {
        memcpy(req->value, &this->pauseTime, sizeof(this->pauseTime));
      }
      else if (prop == "real_time")
      {
        memcpy(req->value, &this->realTime, sizeof(this->realTime));
      }
      else if (prop == "state")
      {
        if (this->paused)
          req->value[0] = 0;
        else
          req->value[0] = 1;
      }

      this->driver->Publish(this->device_addr, _respQueue,
                          PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_SIMULATION_REQ_GET_PROPERTY, req,
                          sizeof(*req), NULL);

      if (req->value)
      {
        delete [] req->value;
        req->value = NULL;
      }
    }
    else
    {
      if (prop == "fiducial_id")
      {
        // strcpy((char*)gzReq->name, req->name);
      }
      else
      {
        gzerr << "The object [" << name
          << "] does not have the property [" << prop << "].\n";
      }
    }
  }
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_CMD,
                                 PLAYER_SIMULATION_CMD_PAUSE,
                                 this->device_addr))
  {
    // TODO: Implement
  }
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_CMD,
                                 PLAYER_SIMULATION_CMD_RESET,
                                 this->device_addr))
  {
    // TODO: Implement
  }
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_CMD,
                                 PLAYER_SIMULATION_CMD_SAVE,
                                 this->device_addr))
  {
    // TODO: Implement
  }
  else
    printf("Unhandled Process message[%d][%d]\n", 0, 0);

  return 0;
}


//////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void SimulationInterface::Update()
{
      /*case libgazebo::SimulationRequestData::GET_MODEL_FIDUCIAL_ID:
        {
          player_simulation_property_req_t *req ;
          memset (req, 0, sizeof(player_simulation_property_req_t));

          memcpy(req->value, (const void *) response->uintValue, sizeof(response->uintValue));
          req->value_count = sizeof(response->uintValue);
          this->driver->Publish(this->device_addr, *(this->responseQueue),
                          PLAYER_MSGTYPE_RESP_ACK, PLAYER_SIMULATION_REQ_GET_PROPERTY, req, sizeof(*req), NULL);
          break;
        }
        */

  return;
}

//////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void SimulationInterface::Subscribe()
{
}

//////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void SimulationInterface::Unsubscribe()
{
}

void SimulationInterface::OnStats(ConstWorldStatisticsPtr &_msg)
{
  this->simTime  = gazebo::msgs::Convert(_msg->sim_time()).Double();
  this->realTime  = gazebo::msgs::Convert(_msg->real_time()).Double();
  this->pauseTime  = gazebo::msgs::Convert(_msg->pause_time()).Double();
  this->paused  = _msg->paused();
}
