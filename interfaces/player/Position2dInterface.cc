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
/* Desc: Position Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

/**
@addtogroup player
@par Position2d Interface
- PLAYER_POSITION2D_CMD_VEL
- PLAYER_POSITION2D_REQ_SET_ODOM
- PLAYER_POSITION2D_REQ_MOTOR_POWER
*/

/* TODO
- PLAYER_POSITION2D_REQ_GET_GEOM
- PLAYER_POSITION2D_REQ_RESET_ODOM
*/
#include <math.h>
#include <iostream>

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/math/gzmath.hh"
#include "GazeboDriver.hh"
#include "Position2dInterface.hh"

//////////////////////////////////////////////////
// Constructor
Position2dInterface::Position2dInterface(player_devaddr_t _addr,
    GazeboDriver *_driver, ConfigFile *_cf, int _section)
    : GazeboInterface(_addr, _driver, _cf, _section)
{
  this->datatime = -1;
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->worldName);
  this->modelName = _cf->ReadString(_section, "model_name", "default");

  this->velPub = this->node->Advertise<gazebo::msgs::Pose>(
      std::string("~/") + this->modelName + "/vel_cmd");
}

//////////////////////////////////////////////////
// Destructor
Position2dInterface::~Position2dInterface()
{
}

//////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int Position2dInterface::ProcessMessage(QueuePointer &_respQueue,
                                        player_msghdr_t *_hdr, void *_data)
{
  int result = 0;

  // COMMAND VELOCITY:
  if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_POSITION2D_CMD_VEL, this->device_addr))
  {
    player_position2d_cmd_vel_t *cmd;
    cmd = static_cast<player_position2d_cmd_vel_t*>(_data);

    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(msg.mutable_position(),
                      gazebo::math::Vector3(cmd->vel.px, cmd->vel.py, 0));
    gazebo::msgs::Set(msg.mutable_orientation(),
                      gazebo::math::Quaternion(0, 0, -cmd->vel.pa));
    this->velPub->Publish(msg);

    result = 0;
  }

  // REQUEST SET ODOMETRY
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_POSITION2D_REQ_SET_ODOM, this->device_addr))
  {
    if (_hdr->size != sizeof(player_position2d_set_odom_req_t))
    {
      PLAYER_WARN("Arg to odometry set requestes wrong size; ignoring");
      result = -1;
    }
    else
    {
      /*
      player_position2d_set_odom_req_t *odom =
        (player_position2d_set_odom_req_t*)_data;

      this->iface->data->pose.pos.x = odom->pose.px;
      this->iface->data->pose.pos.y = odom->pose.py;
      this->iface->data->pose.yaw = odom->pose.pa;
      */

      this->driver->Publish(this->device_addr, _respQueue,
          PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_SET_ODOM);

      result = 0;
    }
  }

  // CMD Set Motor Power
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_POSITION2D_REQ_MOTOR_POWER, this->device_addr))
  {
    // TODO
    result = 0;
  }

  // REQUEST SET MOTOR POWER
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_POSITION2D_REQ_MOTOR_POWER, this->device_addr))
  {
    if (_hdr->size != sizeof(player_position2d_power_config_t))
    {
      PLAYER_WARN("Arg to motor set requestes wrong size; ignoring");
      result = -1;
    }
    else
    {
      // player_position2d_power_config_t *power;
      // power = (player_position2d_power_config_t*)_data;
      // this->iface->data->cmdEnableMotors = power->state;

      this->driver->Publish(this->device_addr, _respQueue,
          PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);

      result = 0;
    }
  }

  // REQUEST GET GEOMETRY
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_POSITION2D_REQ_GET_GEOM, this->device_addr))
  {
    if (_hdr->size != 0)
    {
      PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
      result = -1;
    }
    else
    {
      player_position2d_geom_t geom;

      // TODO: get correct dimensions; these are for the P2AT
      geom.pose.px = 0;
      geom.pose.py = 0;
      geom.pose.pz = 0;
      geom.pose.pyaw = 0;
      geom.pose.ppitch = 0;
      geom.pose.proll = 0;
      geom.size.sw = 0.53;
      geom.size.sl = 0.38;
      geom.size.sh = 0.31;

      this->driver->Publish(this->device_addr, _respQueue,
          PLAYER_MSGTYPE_RESP_ACK,
          PLAYER_POSITION2D_REQ_GET_GEOM,
          &geom, sizeof(geom), NULL);

      result = 0;
    }
  }

  // REQUEST RESET ODOMETRY
  else if (Message::MatchMessage(_hdr, PLAYER_MSGTYPE_REQ,
           PLAYER_POSITION2D_REQ_RESET_ODOM, this->device_addr))
  {
    if (_hdr->size != 0)
    {
      PLAYER_WARN("Arg reset position request is wrong size; ignoring");
      result = -1;
    }
    else
    {
      // TODO: Make this work!!
      this->driver->Publish(this->device_addr, _respQueue,
          PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_RESET_ODOM);
      result = 0;
    }
  }
  else
  {
    result = -1;
  }

  return result;
}

//////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void Position2dInterface::Update()
{
}


//////////////////////////////////////////////////
void Position2dInterface::Subscribe()
{
  this->poseSub = this->node->Subscribe("~/pose/info",
      &Position2dInterface::OnPoseMsg, this);
}

//////////////////////////////////////////////////
void Position2dInterface::Unsubscribe()
{
  this->poseSub.reset();
}

//////////////////////////////////////////////////
void Position2dInterface::OnPoseMsg(ConstPosesStampedPtr &_msg)
{
  // Iterate through all the pose messages
  for (int i = 0; i < _msg->pose_size(); ++i)
  {
    if (_msg->pose(i).name() != this->modelName)
      continue;

    player_position2d_data_t data;
    memset(&data, 0, sizeof(data));

    this->datatime = gazebo::msgs::Convert(_msg->time()).Double();

    data.pos.px = _msg->pose(i).position().x();
    data.pos.py = _msg->pose(i).position().y();
    gazebo::math::Quaternion quat =
      gazebo::msgs::Convert(_msg->pose(i).orientation());
    data.pos.pa = quat.GetYaw();

    this->driver->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA,
        PLAYER_POSITION2D_DATA_STATE,
        static_cast<void*>(&data), sizeof(data), &this->datatime);
  }
}
