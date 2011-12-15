/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Camera Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

#include <math.h>
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "gz.h"
#include "GazeboDriver.hh"
#include "CameraInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *CameraInterface::mutex = NULL;

///////////////////////////////////////////////////////////////////////////////
// Constructor
CameraInterface::CameraInterface(player_devaddr_t addr,
                                 GazeboDriver *driver, ConfigFile *cf, int section)
    : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Camera Interface
  this->iface = new CameraIface();

  // Save frames?
  this->save = cf->ReadInt(section, "save", 0);
  this->frameno = 0;

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
CameraInterface::~CameraInterface()
{
  // Delete this interface
  delete this->iface;
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int CameraInterface::ProcessMessage(QueuePointer &respQueue,
                                    player_msghdr_t *hdr, void *data)
{
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void CameraInterface::Update()
{
  size_t size;
  char filename[256];

  struct timeval ts;

  // Thread safe locking
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime )
  {
    this->datatime = this->iface->data->head.time;

    ts.tv_sec = (int) (this->iface->data->head.time);
    ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

    // Set the image properties
    this->data.width = this->iface->data->width;
    this->data.height = this->iface->data->height;
    this->data.bpp = 24;
    this->data.fdiv = 1;
    this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
    this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;

    unsigned int oldCount = this->data.image_count;
    this->data.image_count = this->iface->data->image_size;

    if (oldCount != this->data.image_count)
    {
      delete this->data.image;
      this->data.image = new uint8_t[this->data.image_count];
    }

    // Set the image pixels
    memcpy(this->data.image, this->iface->data->image,
           this->iface->data->image_size);

    size = sizeof(this->data) - sizeof(this->data.image) +
           this->iface->data->image_size;

    // Send data to server
    this->driver->Publish(this->device_addr,
                          PLAYER_MSGTYPE_DATA,
                          PLAYER_CAMERA_DATA_STATE,
                          (void*)&this->data, size, &this->datatime);

    // Save frames
    if (this->save)
    {
      snprintf(filename, sizeof(filename), "click-%04d.ppm",this->frameno++);
      this->SaveFrame(filename);
    }
  }

  // Done with the interface
  this->iface->Unlock();

}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void CameraInterface::Subscribe()
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
    std::cout << "Error Subscribing to Gazebo Camera Interface[" 
      << this->gz_id << "]\n" << e << "\n";
    //gzthrow(stream.str());
    exit(0);
  }

}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void CameraInterface::Unsubscribe()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  this->iface->Close();
}

////////////////////////////////////////////////////////////////////////////////
// Save an image frame
void CameraInterface::SaveFrame(const char *filename)
{
  int i, width, height;
  FILE *file;

  file = fopen(filename, "w+");
  if (!file)
    return;

  width = this->data.width;
  height = this->data.height;

  int pixelSize = 3;
  int rowSize = width * pixelSize;

  if (this->data.format == PLAYER_CAMERA_FORMAT_RGB888)
  {
    // Write ppm
    fprintf(file, "P6\n%d %d\n%d\n", width, height, 255);
    for (i = 0; i < height; i++)
      fwrite(this->data.image + i * rowSize, rowSize, 1, file);
  }
  else
  {
    PLAYER_WARN("unsupported format for saving");
  }

  fclose(file);
}
