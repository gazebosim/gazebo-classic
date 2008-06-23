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

GZ_REGISTER_STATIC_CONTROLLER("stereocamera", Stereo_Camera);

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
  std::vector<Iface*>::iterator iter;

  for (iter = this->ifaces.begin(); iter != this->ifaces.end(); iter++)
  {
    if ((*iter)->GetType() == "stereo")
      this->stereoIface = dynamic_cast<StereoCameraIface*>(*iter);
    else if ((*iter)->GetType() == "camera")
      this->cameraIface = dynamic_cast<CameraIface*>(*iter);
  }

  if (!this->stereoIface)
    gzthrow("Stereo_Camera controller requires a StereoCameraIface");
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Stereo_Camera::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Stereo_Camera::UpdateChild()
{
  if (this->cameraIface)
  {
    this->cameraIface->Lock(1);
    if (this->cameraIface->data->head.openCount > 0)
      this->PutCameraData();
    this->cameraIface->Unlock();

    // New data is available
    this->cameraIface->Post();
  }

  if (this->stereoIface)
  {
    this->stereoIface->Lock(1);
    if (this->stereoIface->data->head.openCount > 0)
      this->PutStereoData();
    this->stereoIface->Unlock();

    this->stereoIface->Post();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Stereo_Camera::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put stereo data to the interface
void Stereo_Camera::PutStereoData()
{

  printf("put stereo data\n");
  StereoCameraData *stereo_data = this->stereoIface->data;
  //const unsigned char *rgb_src = NULL;
  //unsigned char *rgb_dst = NULL;

  const float *disp_src;
  float *disp_dst;

  // Data timestamp
  stereo_data->head.time = Simulator::Instance()->GetSimTime();

  stereo_data->width = this->myParent->GetImageWidth();
  stereo_data->height = this->myParent->GetImageHeight();
  stereo_data->farClip = this->myParent->GetFarClip();
  stereo_data->nearClip = this->myParent->GetNearClip();

  stereo_data->hfov = this->myParent->GetFOV();

  //stereo_data->right_rgb_size = stereo_data->width * stereo_data->height * 3;
  //stereo_data->left_rgb_size = stereo_data->width * stereo_data->height * 3;

  stereo_data->right_disparity_size = stereo_data->width * stereo_data->height * sizeof(float);
  stereo_data->left_disparity_size = stereo_data->width * stereo_data->height * sizeof(float);

  // Make sure there is room to store the image
  //assert (stereo_data->right_rgb_size <= sizeof(stereo_data->right_rgb));
  //assert (stereo_data->left_rgb_size <= sizeof(stereo_data->left_rgb));

  assert (stereo_data->right_disparity_size <= sizeof(stereo_data->right_disparity));
  assert (stereo_data->left_disparity_size <= sizeof(stereo_data->left_disparity));

  // Copy the left pixel data to the interface
  /*rgb_src = this->myParent->GetImageData(0);
  rgb_dst = stereo_data->left_rgb;
  memcpy(rgb_dst, rgb_src, stereo_data->left_rgb_size);

  // Copy the right pixel data to the interface
  rgb_src = this->myParent->GetImageData(1);
  rgb_dst = stereo_data->right_rgb;
  memcpy(rgb_dst, rgb_src, stereo_data->right_rgb_size);
  */

  // Copy the left disparity data to the interface
  disp_src = this->myParent->GetDisparityData(0);
  disp_dst = stereo_data->left_disparity;
  memcpy(disp_dst, disp_src, stereo_data->left_disparity_size);

  // Copy the right disparity data to the interface
  disp_src = this->myParent->GetDisparityData(1);
  disp_dst = stereo_data->right_disparity;
  memcpy(disp_dst, disp_src, stereo_data->right_disparity_size);

}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void Stereo_Camera::PutCameraData()
{
  CameraData *camera_data = this->cameraIface->data;
  const unsigned char *rgb_src = NULL;
  unsigned char *rgb_dst = NULL;
  Pose3d cameraPose;

  camera_data->head.time = Simulator::Instance()->GetSimTime();

  camera_data->width = this->myParent->GetImageWidth();
  camera_data->height = this->myParent->GetImageHeight();
  camera_data->image_size = camera_data->width * camera_data->height * 3;
  assert (camera_data->image_size <= sizeof(camera_data->image));

  camera_data->hfov = this->myParent->GetFOV();
  // Set the pose of the camera
  cameraPose = this->myParent->GetWorldPose();
  camera_data->camera_pose.pos.x = cameraPose.pos.x;
  camera_data->camera_pose.pos.y = cameraPose.pos.y;
  camera_data->camera_pose.pos.z = cameraPose.pos.z;
  camera_data->camera_pose.roll = cameraPose.rot.GetRoll();
  camera_data->camera_pose.pitch = cameraPose.rot.GetPitch();
  camera_data->camera_pose.yaw = cameraPose.rot.GetYaw();

  // Copy the pixel data to the interface
  rgb_src = this->myParent->GetImageData(0);
  rgb_dst = camera_data->image;

  memcpy(rgb_dst, rgb_src, camera_data->image_size);

}

