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
/* Desc: Fiducial Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

/* TODO
- PLAYER_FIDUCIAL_REQ_GET_GEOM
- PLAYER_FIDUCIAL_REQ_SET_ID
- PLAYER_FIDUCIAL_REQ_GET_ID
*/


#include <math.h>
#include <libplayerxdr/playerxdr.h>

#include "GazeboError.hh"
#include "gazebo.h"
#include "GazeboDriver.hh"
#include "FiducialInterface.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Constructor
FiducialInterface::FiducialInterface(player_devaddr_t addr,
                                     GazeboDriver *driver, ConfigFile *cf, 
                                     int section)
    : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new FiducialIface();

  this->datatime = -1;

  memset(&this->data, 0, sizeof(this->data));

}

///////////////////////////////////////////////////////////////////////////////
// Destructor
FiducialInterface::~FiducialInterface()
{
  player_fiducial_data_t_cleanup(&this->data);

  // Release this interface
  delete this->iface;
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int FiducialInterface::ProcessMessage(QueuePointer &respQueue,
                                      player_msghdr_t *hdr, void *data)
{
  // Request the pose and size of the fiducial device
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                            PLAYER_FIDUCIAL_REQ_GET_GEOM, this->device_addr))
  {
    player_fiducial_geom_t rep;

    rep.pose.px = 0.0;
    rep.pose.py = 0.0;
    rep.pose.pz = 0.0;

    rep.pose.proll = 0.0;
    rep.pose.ppitch = 0.0;
    rep.pose.pyaw = 0.0;

    rep.size.sw = 0.0;
    rep.size.sl = 0.0;
    rep.size.sh = 0.0;

    rep.fiducial_size.sw = 0.05;
    rep.fiducial_size.sl = 0.50;

    this->driver->Publish(this->device_addr, respQueue,
                          PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_FIDUCIAL_REQ_GET_GEOM,
                          &rep, sizeof(rep),NULL);

    return 0;
  }

  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_FIDUCIAL_REQ_SET_ID, this->device_addr))
  {
    if ( hdr->size == sizeof(player_fiducial_id_t) )
    {
      // TODO: Implement me

      player_fiducial_id_t pid;
      pid.id = 0;

      // acknowledge, including the new ID
      this->driver->Publish(this->device_addr, respQueue,
                            PLAYER_MSGTYPE_RESP_ACK,
                            PLAYER_FIDUCIAL_REQ_SET_ID,
                            (void*)&pid, sizeof(pid) );

      return 0;
    }
    else
    {
      printf("Incorrect packet size setting fiducial ID (%d/%d)",
             (int)hdr->size, (int)sizeof(player_fiducial_id_t) );

      return -1; // error - NACK is sent automatically
    }
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_FIDUCIAL_REQ_GET_ID, this->device_addr))
  {

    // TODO: Implement me

    // fill in the data formatted player-like
    player_fiducial_id_t pid;
    pid.id = 0;

    // acknowledge, including the new ID
    this->driver->Publish(this->device_addr, respQueue,
                          PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_FIDUCIAL_REQ_GET_ID,
                          (void*)&pid, sizeof(pid) );

    return 0;
  }

  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void FiducialInterface::Update()
{
  struct timeval ts;

  this->iface->Lock(1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
  {
    int i;
    FiducialFid *fid;

    this->datatime = this->iface->data->head.time;

    ts.tv_sec = (int) (this->iface->data->head.time);
    ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

    unsigned int oldCount = data.fiducials_count;

    data.fiducials_count = i;

    if (oldCount != this->data.fiducials_count)
    {
      delete [] this->data.fiducials;

      this->data.fiducials = new player_fiducial_item_t[this->data.fiducials_count];
    }

    for (i = 0; i < this->iface->data->count; i++)
    {
      fid = this->iface->data->fids + i;

      this->data.fiducials[i].id = (int16_t) fid->id;

      this->data.fiducials[i].pose.px = fid->pose.pos.x;
      this->data.fiducials[i].pose.py = fid->pose.pos.y;
      this->data.fiducials[i].pose.pz = fid->pose.pos.z;
      this->data.fiducials[i].pose.proll = fid->pose.roll;
      this->data.fiducials[i].pose.ppitch = fid->pose.pitch;
      this->data.fiducials[i].pose.pyaw = fid->pose.yaw;
    }

    this->driver->Publish( this->device_addr,
                           PLAYER_MSGTYPE_DATA,
                           PLAYER_FIDUCIAL_DATA_SCAN,
                           reinterpret_cast<void*>(&data), sizeof(this->data), 
                           &this->datatime );
                           
  }

  this->iface->Unlock();
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void FiducialInterface::Subscribe()
{
  // Open the interface
  try
  {
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string e)
  {
    //std::ostringstream stream;
    std::cout << "Error Subscribing to Gazebo Fiducial Interface\n"
    << e << "\n";
    //gzthrow(stream.str());
    exit(0);
  }

}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void FiducialInterface::Unsubscribe()
{
  this->iface->Close();
}
