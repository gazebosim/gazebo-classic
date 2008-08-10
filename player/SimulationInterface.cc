/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Simulation Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

#include <time.h>

#include "gazebo.h"
#include "GazeboError.hh"
#include "GazeboDriver.hh"
#include "SimulationInterface.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Constructor
SimulationInterface::SimulationInterface(player_devaddr_t addr, GazeboDriver *driver, ConfigFile *cf, int section)
    : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "server_id", "default"));

  // ID of the server
  int serverId = atoi((char*)cf->ReadString(section,"server_id","default"));

  // Initialize the Client. Creates the SHM connection

  GazeboClient::Init(serverId, "");

  this->iface = new SimulationIface();

  this->responseQueue = NULL;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
SimulationInterface::~SimulationInterface()
{
  delete this->iface;

  if (this->responseQueue)
  {
    delete this->responseQueue;
    this->responseQueue = NULL;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int SimulationInterface::ProcessMessage(QueuePointer &respQueue,
                                        player_msghdr_t *hdr, void *data)
{
  if (this->responseQueue)
    delete this->responseQueue;

  this->responseQueue = new QueuePointer(respQueue);

  /// Set a 3D pose
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                            PLAYER_SIMULATION_REQ_SET_POSE3D, this->device_addr))
  {
    gazebo::SimulationRequestData *gzReq = NULL;
    player_simulation_pose3d_req_t *req =
      (player_simulation_pose3d_req_t*)(data);

    this->iface->Lock(1);

    gzReq = &(this->iface->data->requests[ this->iface->data->requestCount++ ]);

    gzReq->type = gazebo::SimulationRequestData::SET_POSE3D;
    strcpy((char*)gzReq->modelName, req->name);

    gzReq->modelPose.pos.x = req->pose.px;
    gzReq->modelPose.pos.y = req->pose.py;
    gzReq->modelPose.pos.z = req->pose.pz;

    gzReq->modelPose.roll = req->pose.proll;
    gzReq->modelPose.pitch = req->pose.ppitch;
    gzReq->modelPose.yaw = req->pose.pyaw;

    this->iface->Unlock();

    this->driver->Publish(this->device_addr, respQueue,
                          PLAYER_MSGTYPE_RESP_ACK, 
                          PLAYER_SIMULATION_REQ_SET_POSE3D);

  }

  /// Set a 2D pose
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_SET_POSE2D, 
                                 this->device_addr))
  {
    gazebo::SimulationRequestData *gzReq = NULL;

    player_simulation_pose2d_req_t *req =
      (player_simulation_pose2d_req_t*)(data);

    this->iface->Lock(1);

    gzReq = &(this->iface->data->requests[ this->iface->data->requestCount++]);

    gzReq->type = gazebo::SimulationRequestData::SET_POSE2D;

    strcpy((char*)gzReq->modelName, req->name);

    gzReq->modelPose.pos.x = req->pose.px;
    gzReq->modelPose.pos.y = req->pose.py;
    gzReq->modelPose.yaw = req->pose.pa;

    this->iface->Unlock();

    this->driver->Publish(this->device_addr, respQueue,
                          PLAYER_MSGTYPE_RESP_ACK, 
                          PLAYER_SIMULATION_REQ_SET_POSE2D);
  }

  /// Get a 3d pose
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_GET_POSE3D, 
                                 this->device_addr))
  {
    gazebo::SimulationRequestData *gzReq = NULL;
    player_simulation_pose3d_req_t *req =
      (player_simulation_pose3d_req_t*)(data);

    this->iface->Lock(1);

    gzReq = &(this->iface->data->requests[this->iface->data->requestCount++]);

    gzReq->type = gazebo::SimulationRequestData::GET_POSE3D;

    strcpy((char*)gzReq->modelName, req->name);

    this->iface->Unlock();
  }

  /// Get a 2D pose
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_GET_POSE2D, this->device_addr))
  {
    gazebo::SimulationRequestData *gzReq = NULL;
    player_simulation_pose2d_req_t *req =
      (player_simulation_pose2d_req_t*)(data);

    this->iface->Lock(1);

    gzReq = &(this->iface->data->requests[this->iface->data->requestCount++]);

    gzReq->type = gazebo::SimulationRequestData::GET_POSE2D;

    strcpy((char*)gzReq->modelName, req->name);

    this->iface->Unlock();
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_SIMULATION_REQ_GET_PROPERTY, 
                                 this->device_addr))
  {
    player_simulation_property_req_t *req =
      (player_simulation_property_req_t*)(data);

    std::string name = req->name;
    std::string prop = req->prop;

    this->iface->Lock(1);
    if (name == "world")
    {
      req->value = new char[ sizeof(double) ];
      req->value_count = sizeof(double);

      if (prop == "sim_time")
      {
        memcpy(req->value, &this->iface->data->simTime, sizeof(double));
      }
      else if (prop == "pause_time")
      {
        memcpy(req->value, &this->iface->data->pauseTime, sizeof(double));
      }
      else if (prop == "real_time")
      {
        memcpy(req->value, &this->iface->data->realTime, sizeof(double));
      }
    }
    else
    {
      std::cerr << "Invalid Name[" << name << "]. Must be \"world\".\n";
    }

    this->iface->Unlock();

    this->driver->Publish(this->device_addr, respQueue,
                          PLAYER_MSGTYPE_RESP_ACK, PLAYER_SIMULATION_REQ_GET_PROPERTY, req, sizeof(*req), NULL);

    if (req->value)
    {
      delete [] req->value;
      req->value = NULL;
    }
  }

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void SimulationInterface::Update()
{
  gazebo::SimulationRequestData *response = NULL;
  this->iface->Lock(1);

  for (unsigned int i=0; i < this->iface->data->responseCount; i++)
  {
    response = &(this->iface->data->responses[i]);

    switch (response->type)
    {
      case gazebo::SimulationRequestData::PAUSE:
      case gazebo::SimulationRequestData::RESET:
      case gazebo::SimulationRequestData::SAVE:
      case gazebo::SimulationRequestData::SET_POSE2D:
      case gazebo::SimulationRequestData::SET_POSE3D:
        break;

      case gazebo::SimulationRequestData::GET_POSE3D:
        {
          player_simulation_pose3d_req_t req;

          strcpy(req.name, response->modelName);
          req.name_count = strlen(req.name);

          req.pose.px = response->modelPose.pos.x;
          req.pose.py = response->modelPose.pos.y;
          req.pose.pz = response->modelPose.pos.z;

          req.pose.proll = response->modelPose.roll;
          req.pose.ppitch = response->modelPose.pitch;
          req.pose.pyaw = response->modelPose.yaw;

          this->driver->Publish(this->device_addr, *(this->responseQueue),
              PLAYER_MSGTYPE_RESP_ACK, PLAYER_SIMULATION_REQ_GET_POSE3D,
              &req, sizeof(req), NULL);

          break;
        }
      case gazebo::SimulationRequestData::GET_POSE2D:
        {
          player_simulation_pose2d_req_t req;

          strcpy(req.name, response->modelName);
          req.name_count = strlen(req.name);

          req.pose.px = response->modelPose.pos.x;
          req.pose.py = response->modelPose.pos.y;
          req.pose.pa = response->modelPose.yaw;

          this->driver->Publish(this->device_addr, *(this->responseQueue),
              PLAYER_MSGTYPE_RESP_ACK, PLAYER_SIMULATION_REQ_GET_POSE2D,
              &req, sizeof(req), NULL);

          break;
        }
    }
  }

  this->iface->data->responseCount = 0;

  this->iface->Unlock();

  return;
}

///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void SimulationInterface::Subscribe()
{
  // Open the interface
  try
  {
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string e)
  {
    //std::ostringstream stream;
    std::cout <<"Error Subscribing to Gazebo Simulation Interface\n"
    << e << "\n";
    //gzthrow(stream.str());
    exit(0);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void SimulationInterface::Unsubscribe()
{
  this->iface->Close();
}
