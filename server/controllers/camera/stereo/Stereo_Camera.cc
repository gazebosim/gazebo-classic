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
 * SVN info: $Id$
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

  Param::Begin(&this->parameters);
  this->leftCameraNameP = new ParamT<std::string>("leftcamera","", 1);
  this->rightCameraNameP = new ParamT<std::string>("rightcamera","", 1);
  Param::End();

  if (!this->myParent)
    gzthrow("Stereo_Camera controller requires a Stereo Camera Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Stereo_Camera::~Stereo_Camera()
{
  delete this->leftCameraNameP;
  delete this->rightCameraNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Stereo_Camera::LoadChild(XMLConfigNode *node)
{
  CameraIface *ciface = NULL;

  this->stereoIface = dynamic_cast<StereoCameraIface*>(this->GetIface("stereo"));

  ciface = dynamic_cast<CameraIface*>(this->GetIface("camera",true,0));
  this->cameraIfaces[ciface->GetId()] = ciface;
  ciface = dynamic_cast<CameraIface*>(this->GetIface("camera",true,1));
  this->cameraIfaces[ciface->GetId()] = ciface;

  this->leftCameraNameP->Load(node);
  this->rightCameraNameP->Load(node);
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void Stereo_Camera::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftCameraNameP) << "\n";
  stream << prefix << *(this->rightCameraNameP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Stereo_Camera::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
/// True if a stereo iface is connected
bool Stereo_Camera::StereoIfaceConnected() const
{
  if (this->stereoIface)
    return this->stereoIface->GetOpenCount() > 0;
  else
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Stereo_Camera::UpdateChild()
{
  std::map< std::string, CameraIface*>::iterator iter;

  for (iter = this->cameraIfaces.begin(); 
       iter != this->cameraIfaces.end(); iter++)
  {
    iter->second->Lock(1);

    if (iter->second->data->head.openCount > 0)
    {
      if (**(this->leftCameraNameP) == iter->first)
        this->PutCameraData( iter->second->data, 0 );
      else
        this->PutCameraData( iter->second->data, 1 );
    }

    iter->second->Unlock();
    iter->second->Post();
  }

  if (this->stereoIface)
  {
    this->stereoIface->Lock(1);
    if (this->stereoIface->data->head.openCount > 0)
      this->PutStereoData();

    strcpy( this->stereoIface->data->left_camera_iface_name, 
            (**this->leftCameraNameP).c_str() );

    strcpy( this->stereoIface->data->right_camera_iface_name, 
            (**this->rightCameraNameP).c_str() );

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
  StereoCameraData *stereo_data = this->stereoIface->data;
  //const unsigned char *rgb_src = NULL;
  //unsigned char *rgb_dst = NULL;

  const float *disp_src;
  float *disp_dst;

  // Data timestamp
  stereo_data->head.time = Simulator::Instance()->GetSimTime().Double();

  stereo_data->width = this->myParent->GetImageWidth();
  stereo_data->height = this->myParent->GetImageHeight();
  stereo_data->farClip = this->myParent->GetFarClip();
  stereo_data->nearClip = this->myParent->GetNearClip();

  stereo_data->hfov = *(this->myParent->GetHFOV());
  stereo_data->vfov = *(this->myParent->GetVFOV());

  stereo_data->right_depth_size = stereo_data->width * stereo_data->height * sizeof(float);
  stereo_data->left_depth_size = stereo_data->width * stereo_data->height * sizeof(float);

  assert (stereo_data->right_depth_size <= sizeof(stereo_data->right_depth));
  assert (stereo_data->left_depth_size <= sizeof(stereo_data->left_depth));

  // Copy the left depth data to the interface
  disp_src = this->myParent->GetDepthData(0);
  disp_dst = stereo_data->left_depth;
  memcpy(disp_dst, disp_src, stereo_data->left_depth_size);

  // Copy the right depth data to the interface
  disp_src = this->myParent->GetDepthData(1);
  disp_dst = stereo_data->right_depth;
  memcpy(disp_dst, disp_src, stereo_data->right_depth_size);

}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void Stereo_Camera::PutCameraData(CameraData *camera_data, unsigned int camera)
{
  //CameraData *camera_data = this->cameraIface->data;
  const unsigned char *rgb_src = NULL;
  unsigned char *rgb_dst = NULL;
  Pose3d cameraPose;

  camera_data->head.time = Simulator::Instance()->GetSimTime().Double();

  camera_data->width = this->myParent->GetImageWidth();
  camera_data->height = this->myParent->GetImageHeight();
  camera_data->image_size = camera_data->width * camera_data->height * 3;
  assert (camera_data->image_size <= sizeof(camera_data->image));

  camera_data->hfov = *(this->myParent->GetHFOV());
  camera_data->vfov = *(this->myParent->GetVFOV());

  // Set the pose of the camera
  cameraPose = this->myParent->GetWorldPose();
  camera_data->camera_pose.pos.x = cameraPose.pos.x;
  camera_data->camera_pose.pos.y = cameraPose.pos.y;
  camera_data->camera_pose.pos.z = cameraPose.pos.z;
  camera_data->camera_pose.roll = cameraPose.rot.GetRoll();
  camera_data->camera_pose.pitch = cameraPose.rot.GetPitch();
  camera_data->camera_pose.yaw = cameraPose.rot.GetYaw();

  // Copy the pixel data to the interface
  rgb_src = this->myParent->GetImageData(camera);
  rgb_dst = camera_data->image;

  memcpy(rgb_dst, rgb_src, camera_data->image_size);
}

