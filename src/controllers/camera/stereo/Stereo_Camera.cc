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
/*
 * Desc: Stereo camera controller.
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN info: $Id$
 */

#include <algorithm>
#include <assert.h>

#include "Sensor.hh"
#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Simulator.hh"
#include "common/GazeboError.hh"
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
  libgazebo::CameraIface *ciface = NULL;

  this->stereoIface = dynamic_cast<libgazebo::StereoCameraIface*>(this->GetIface("stereo"));

  ciface = dynamic_cast<libgazebo::CameraIface*>(this->GetIface("camera",true,0));
  this->cameraIfaces[ciface->GetId()] = ciface;
  ciface = dynamic_cast<libgazebo::CameraIface*>(this->GetIface("camera",true,1));
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
  std::map< std::string, libgazebo::CameraIface*>::iterator iter;

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
  libgazebo::StereoCameraData *stereo_data = this->stereoIface->data;
  //const unsigned char *rgb_src = NULL;
  //unsigned char *rgb_dst = NULL;

  const float *disp_src;
  float *disp_dst;

  // Data timestamp
  stereo_data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

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
void Stereo_Camera::PutCameraData(libgazebo::CameraData *camera_data, unsigned int camera)
{
  //CameraData *camera_data = this->cameraIface->data;
  const unsigned char *rgb_src = NULL;
  unsigned char *rgb_dst = NULL;
  Pose3d cameraPose;

  camera_data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

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

