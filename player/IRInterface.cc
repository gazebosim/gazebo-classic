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
/* Desc: Laser Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id: IRInterface.cc 6296 2008-04-10 21:24:57Z natepak $
 */

/* TODO
PLAYER_LASER_REQ_GET_GEOM
*/

#include <math.h>
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "gz.h"
#include "GazeboDriver.hh"
#include "IRInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *IRInterface::mutex = NULL;

///////////////////////////////////////////////////////////////////////////////
// Constructor
IRInterface::IRInterface(player_devaddr_t addr,
                               GazeboDriver *driver, ConfigFile *cf, int section)
    : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new IRIface();

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
IRInterface::~IRInterface()
{
  player_ir_data_t_cleanup(&this->data);
  
  // Release this interface
  delete this->iface;
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int IRInterface::ProcessMessage(QueuePointer &respQueue,
                                   player_msghdr_t *hdr, void *data)
{

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_IR_REQ_POSE, this->device_addr))
  {
    player_ir_pose_t rep;

    // TODO: get geometry from somewhere
	rep.poses_count =  this->iface->data->ir_count;
	rep.poses = new player_pose3d_t[rep.poses_count];
	
	for(unsigned int i=0;i<rep.poses_count;i++)
	{
		rep.poses[i].px = this->iface->data->poses[i].pos.x;
		rep.poses[i].py = this->iface->data->poses[i].pos.y;
		rep.poses[i].pz = this->iface->data->poses[i].pos.z;
		rep.poses[i].proll = this->iface->data->poses[i].roll;
		rep.poses[i].ppitch = this->iface->data->poses[i].pitch;
		rep.poses[i].pyaw = this->iface->data->poses[i].yaw;
	}

    this->driver->Publish(this->device_addr, respQueue,
                          PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_IR_REQ_POSE,
                          &rep, sizeof(rep), NULL);
    delete []rep.poses;
    return 0;
  }
 
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void IRInterface::Update()
{
  struct timeval ts;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

    // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
    {
      this->datatime = this->iface->data->head.time;

      ts.tv_sec = (int) (this->iface->data->head.time);
      ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);
      

      int oldCount = this->data.ranges_count;
      this->data.ranges_count = this->iface->data->ir_count;
      this->data.voltages_count = this->iface->data->ir_count;
      
      if(oldCount != (int)this->data.ranges_count)
      {
      	delete []this->data.ranges;
      	delete []this->data.voltages;
      	
      	this->data.ranges = new float[data.ranges_count];
      	this->data.voltages = new float[data.ranges_count];
      }
      

      for (unsigned int i = 0; i < this->data.ranges_count; i++)
      {
         this->data.ranges[i] = (float)this->iface->data->ranges[i];
         this->data.voltages[i]=0;
      }

      this->driver->Publish( this->device_addr,
                             PLAYER_MSGTYPE_DATA,
                             PLAYER_IR_DATA_RANGES,
                             (void*)&this->data, sizeof(this->data), &this->datatime );
    }

    this->iface->Unlock();

}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void IRInterface::Subscribe()
{
  // Open the interface
  try
  {
    boost::recursive_mutex::scoped_lock lock(*this->mutex);
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string e)
  {
    //std::ostringstream stream;
    std::cout << "Error Subscribing to Gazebo IR Interface\n"
    << e << "\n";
    //gzthrow(stream.str());
    exit(0);
  }
  
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void IRInterface::Unsubscribe()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
}
