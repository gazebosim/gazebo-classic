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
/*
 * Desc: Stereo camera controller.
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN info: $Id: Stereo_Camera.cc 4436 2008-03-24 17:42:45Z robotos $
 */

#include <algorithm>
#include <assert.h>

#include "Sensor.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "Simulator.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "StereoCameraSensor.hh"
#include "Stereo_Camera.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("stereo_camera", Stereo_Camera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Stereo_Camera::Stereo_Camera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<StereoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Stereo_Camera controller requires a Stereo Camera Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Stereo_Camera::~Stereo_Camera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Stereo_Camera::LoadChild(XMLConfigNode *node)
{
  this->cameraIface = dynamic_cast<StereoCameraIface*>(this->ifaces[0]);

  if (!this->cameraIface)
    gzthrow("Stereo_Camera controller requires a StereoCameraIface");
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Stereo_Camera::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Stereo_Camera::UpdateChild(UpdateParams &params)
{
  this->PutCameraData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Stereo_Camera::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void Stereo_Camera::PutCameraData()
{
  StereoCameraData *data = this->cameraIface->data;
  const unsigned char *rgb_src = NULL;
  unsigned char *rgb_dst = NULL;

  const unsigned char *rgb_src2 = NULL;
  unsigned char *rgb_dst2 = NULL;

  const float *disp_src;
  float *disp_dst;
 
  int i, j, k;

  this->cameraIface->Lock(1);

  // Data timestamp
  data->time = Simulator::Instance()->GetSimTime();

  data->width = this->myParent->GetImageWidth();
  data->height = this->myParent->GetImageHeight();

  data->right_rgb_size = data->width * data->height * 3;
  data->left_rgb_size = data->width * data->height * 3;

  data->right_disparity_size = data->width * data->height;
  data->left_disparity_size = data->width * data->height;

  // Make sure there is room to store the image
  assert (data->right_rgb_size <= sizeof(data->right_rgb));
  assert (data->left_rgb_size <= sizeof(data->left_rgb));

  assert (data->right_disparity_size <= sizeof(data->right_disparity));
  assert (data->left_disparity_size <= sizeof(data->left_disparity));

  // Copy the left pixel data to the interface
  rgb_src = this->myParent->GetImageData(0);
  rgb_dst = data->left_rgb;
  memcpy(rgb_dst, rgb_src, data->left_rgb_size);

  // Copy the right pixel data to the interface
  rgb_src = this->myParent->GetImageData(1);
  rgb_dst = data->right_rgb;
  memcpy(rgb_dst, rgb_src, data->right_rgb_size);

  // Copy the left disparity data to the interface
  disp_src = this->myParent->GetDisparityData(0);
  disp_dst = data->left_disparity;
  memcpy(disp_dst, disp_src, data->left_disparity_size);

  // Copy the right disparity data to the interface
  disp_src = this->myParent->GetDisparityData(1);
  disp_dst = data->right_disparity;
  memcpy(disp_dst, disp_src, data->right_disparity_size);

  this->cameraIface->Unlock();

  // New data is available
  this->cameraIface->Post();
}
