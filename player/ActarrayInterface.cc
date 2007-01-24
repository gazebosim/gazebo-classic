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
/* Desc: Actarray Interface for Player-Gazebo
 * Author: Alexis Maldonado
 * Date: 19 September 2006
 */

/**
@addtogroup player
@par Actarray Interface
- PLAYER_ACTARRAY_DATA_STATE
*/


#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "ActarrayInterface.hh"

//
// The data for the interface (gz_actarray_data_t in gazebo.h) contains information about each joint
//  plus variables to store the commands. When a new command is sent, it modifies the cmd_pos, cmd_speed, ...
//  variables and sets new_cmd to 1. The gazebo model can look at this variable to find out if a new command came
// Since joints in gazebo only support velocity control, the model has to implement at least a P controller
//  to implement position commands.
// The model should support position and velocity commands. 
// Home commands, brake requests, and power requests are simply translated to a sensible meaning in velocity or position commands.
// If the models need it, this interface should be extended.


///////////////////////////////////////////////////////////////////////////////
// Constructor
ActarrayInterface::ActarrayInterface(player_devaddr_t addr,
                                     GazeboDriver *driver, ConfigFile *cf, int section)
		: GazeboInterface(addr, driver, cf, section) {
	// Get the ID of the interface
	gz_id = (char*) calloc(1024, sizeof(char));
	strcat(gz_id, GazeboClient::prefixId);
	strcat(gz_id, cf->ReadString(section, "gz_id", ""));

	// Allocate a Position Interface
	iface = gz_actarray_alloc();

	datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
ActarrayInterface::~ActarrayInterface() {
	// Release this interface
	gz_actarray_free(iface);
}


///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int ActarrayInterface::ProcessMessage(MessageQueue *respQueue,
                                      player_msghdr_t *hdr, void *data) {

	//printf("ProcessMessage.\n");

	if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
	                          PLAYER_ACTARRAY_POS_CMD, device_addr)) {

		assert(hdr->size >= sizeof(player_actarray_position_cmd_t));

		player_actarray_position_cmd_t *cmd;
		cmd = (player_actarray_position_cmd_t*) data;

		gz_actarray_lock(iface, 1);
		iface->data->cmd_pos[cmd->joint]=cmd->position;
		iface->data->joint_mode[cmd->joint]=GAZEBO_ACTARRAY_JOINT_POSITION_MODE;
		iface->data->new_cmd=1;
		gz_actarray_unlock(iface);

		return(0);

	} else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
	                                 PLAYER_ACTARRAY_SPEED_CMD, device_addr) ) {

		assert(hdr->size >= sizeof(player_actarray_speed_cmd_t));

		player_actarray_speed_cmd_t *cmd;
		cmd = (player_actarray_speed_cmd_t*) data;

		gz_actarray_lock(iface, 1);
		iface->data->cmd_speed[cmd->joint]=cmd->speed;
		iface->data->joint_mode[cmd->joint]=GAZEBO_ACTARRAY_JOINT_SPEED_MODE;

		iface->data->new_cmd=1;
		gz_actarray_unlock(iface);
		return(0);


	} else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
	                                 PLAYER_ACTARRAY_HOME_CMD, device_addr)) {
		assert(hdr->size >= sizeof(player_actarray_home_cmd_t));
		player_actarray_home_cmd_t *cmd;
		cmd = (player_actarray_home_cmd_t*) data;

		gz_actarray_lock(iface, 1);

		iface->data->cmd_pos[cmd->joint]=0.0;
		iface->data->joint_mode[cmd->joint]=GAZEBO_ACTARRAY_JOINT_POSITION_MODE;
		iface->data->new_cmd=1;
		gz_actarray_unlock(iface);
		return(0);

	}  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_BRAKES_REQ, device_addr)) {  // Is it a request for actarray geometry?

		assert(hdr->size >= sizeof(player_actarray_brakes_config_t));
		player_actarray_brakes_config_t *req;
		req = (player_actarray_brakes_config_t*) data;

		//If brakes=on -> Stop all the joints. If they are off, don't do anything
		if (req->value == 1) {
			gz_actarray_lock(iface, 1);

			for (unsigned int i = 0 ; i != GAZEBO_ACTARRAY_NUM_ACTUATORS ; ++i) {
				iface->data->cmd_speed[i]=0.0;
				iface->data->joint_mode[i]=GAZEBO_ACTARRAY_JOINT_SPEED_MODE;
				iface->data->new_cmd=1;

			}
			gz_actarray_unlock(iface);
		}


		driver->Publish(device_addr, respQueue,
		                PLAYER_MSGTYPE_RESP_ACK, PLAYER_ACTARRAY_BRAKES_REQ);
		return(0);
	} else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_ACTARRAY_POWER_REQ, device_addr)) {  // Is it a request for actarray geometry?

		assert(hdr->size >= sizeof(player_actarray_power_config_t));
		player_actarray_power_config_t *req;
		req = (player_actarray_power_config_t*) data;

		//If power=off -> Stop all the joints. If power=on, don't do anything
		if (req->value == 0) {
			gz_actarray_lock(iface, 1);

			for (unsigned int i = 0 ; i != GAZEBO_ACTARRAY_NUM_ACTUATORS ; ++i) {
				iface->data->cmd_speed[i]=0.0;
				iface->data->joint_mode[i]=GAZEBO_ACTARRAY_JOINT_SPEED_MODE;
				iface->data->new_cmd=1;

			}
			gz_actarray_unlock(iface);
		}


		driver->Publish(device_addr, respQueue,
		                PLAYER_MSGTYPE_RESP_ACK, PLAYER_ACTARRAY_POWER_REQ);
		return(0);
	}



	return -1; //This indicates that the message wasn't processed
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void ActarrayInterface::Update() {
	player_actarray_data_t data;
	memset(&data, 0, sizeof(data));

	struct timeval ts;

	gz_actarray_lock(iface, 1);

	// Only Update when new data is present
	if (iface->data->time > datatime) {

		//Update the local time so we know when new info comes
		datatime = iface->data->time;

		// The number of actuators in the array.
		data.actuators_count=iface->data->actuators_count;

		for (unsigned int i=0; i<data.actuators_count ; ++i ) {

			float pos=iface->data->actuators_data[i].position;
			float speed=iface->data->actuators_data[i].speed;
			//float current=iface->data->actuators_data[i].current;
			uint8_t report_state=iface->data->actuators_data[i].state;
			data.actuators[i].position=pos;
			data.actuators[i].speed=speed;
			//data.actuators[i].current=current;
			data.actuators[i].state=report_state;
		}


		driver->Publish( device_addr, NULL,
		                 PLAYER_MSGTYPE_DATA,
		                 PLAYER_ACTARRAY_DATA_STATE,
		                 (void*)&data, sizeof(data), &datatime );


	}

	gz_actarray_unlock(iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void ActarrayInterface::Subscribe() {
	// Open the interface
	if (gz_actarray_open(iface, GazeboClient::client, gz_id) != 0) {
		printf("Error Subscribing to Gazebo Actarray Interface\n");
	}
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void ActarrayInterface::Unsubscribe() {
	gz_actarray_close(iface);
}


