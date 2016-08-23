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
/* Desc: Camera Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#include <boost/algorithm/string.hpp>
#include <math.h>
#include <iostream>

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "GazeboDriver.hh"
#include "CameraInterface.hh"

/////////////////////////////////////////////////
CameraInterface::CameraInterface(player_devaddr_t _addr,
    GazeboDriver *_driver, ConfigFile *_cf, int _section)
: GazeboInterface(_addr, _driver, _cf, _section)
{
  this->datatime = -1;

  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->worldName);
  this->cameraName = _cf->ReadString(_section, "camera_name", "default");

  memset(&this->data, 0, sizeof(this->data));

  // Save frames?
  this->save = _cf->ReadInt(_section, "save", 0);
  this->frameno = 0;
}

/////////////////////////////////////////////////
CameraInterface::~CameraInterface()
{
}

/////////////////////////////////////////////////
int CameraInterface::ProcessMessage(QueuePointer & /*_respQueue*/,
    player_msghdr_t * /*_hdr*/, void * /*_data*/)
{
  return -1;
}

/////////////////////////////////////////////////
void CameraInterface::Update()
{
}

/////////////////////////////////////////////////
void CameraInterface::OnImage(ConstImageStampedPtr &_msg)
{
  // char filename[256];

  size_t size;

  this->datatime = gazebo::msgs::Convert(_msg->time()).Double();

  // Set the image properties
  this->data.width = _msg->image().width();
  this->data.height = _msg->image().height();
  this->data.bpp = (_msg->image().step() / _msg->image().width()) * 8;
  this->data.fdiv = 1;
  this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
  this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;

  unsigned int oldCount = this->data.image_count;
  this->data.image_count = _msg->image().data().size();

  if (oldCount != this->data.image_count)
  {
    delete this->data.image;
    this->data.image = new uint8_t[this->data.image_count];
  }

  // Set the image pixels
  memcpy(this->data.image, _msg->image().data().c_str(),
         _msg->image().data().size());

  size = sizeof(this->data) - sizeof(this->data.image) +
         _msg->image().data().size();

  // Send data to server
  this->driver->Publish(this->device_addr, PLAYER_MSGTYPE_DATA,
                        PLAYER_CAMERA_DATA_STATE,
                        static_cast<void*>(&this->data), size, &this->datatime);

  // Save frames
  /*if (this->save)
  {
    snprintf(filename, sizeof(filename), "click-%04d.ppm", this->frameno++);
    this->SaveFrame(filename);
  }*/
}

/////////////////////////////////////////////////
void CameraInterface::Subscribe()
{
  std::string topic = "~/";
  topic += this->cameraName + "/image";
  boost::replace_all(topic, "::", "/");

  this->cameraSub = this->node->Subscribe(topic,
                                          &CameraInterface::OnImage, this);
}

/////////////////////////////////////////////////
void CameraInterface::Unsubscribe()
{
  this->cameraSub->Unsubscribe();
  this->cameraSub.reset();
}

/////////////////////////////////////////////////
// void CameraInterface::SaveFrame(const char * /*filename*/) const
// {
//   /*
//   int width, height;
//   FILE *file;
//
//   file = fopen(filename, "w+");
//   if (!file)
//     return;
//
//   width = this->data.width;
//   height = this->data.height;
//
//   int pixelSize = 3;
//   int rowSize = width * pixelSize;
//
//   if (this->data.format == PLAYER_CAMERA_FORMAT_RGB888)
//   {
//     // Write ppm
//     fprintf(file, "P6\n%d %d\n%d\n", width, height, 255);
//     for (int i = 0; i < height; i++)
//       fwrite(this->data.image + i * rowSize, rowSize, 1, file);
//   }
//   else
//   {
//     PLAYER_WARN("unsupported format for saving");
//   }
//
//   fclose(file);
//   */
// }
