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
 * Desc: SickLMS200 Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include <algorithm>
#include <assert.h>

#include "World.hh"
#include "Sensor.hh"
#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Simulator.hh"
#include "common/Exception.hh"
#include "ControllerFactory.hh"
#include "MonoCameraSensor.hh"
#include "Generic_Camera.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("generic_camera", Generic_Camera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Generic_Camera::Generic_Camera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Generic_Camera controller requires a Camera Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Generic_Camera::~Generic_Camera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Generic_Camera::LoadChild(XMLConfigNode *node)
{
  this->cameraIface = dynamic_cast<libgazebo::CameraIface*>(this->GetIface("camera"));
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void Generic_Camera::SaveChild(std::string &prefix, std::ostream &stream)
{

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Generic_Camera::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Generic_Camera::UpdateChild()
{

  // do this first so there's chance for sensor to run 1 frame after activate
  if (this->myParent->IsActive())
    this->PutCameraData();

  // activate if iface open
  if (this->cameraIface->Lock(1))
  {
    if (this->cameraIface->GetOpenCount() > 0)
      this->myParent->SetActive(true);
    else
      this->myParent->SetActive(false);

    //std::cout << " camera open count " << this->cameraIface->GetOpenCount() << std::endl;
    this->cameraIface->Unlock();
  }
  //std::cout << " camera     active " << this->myParent->IsActive() << std::endl;

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Generic_Camera::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void Generic_Camera::PutCameraData()
{
  libgazebo::CameraData *data = this->cameraIface->data;
  const unsigned char *src;
  unsigned char *dst;
  Pose3d cameraPose;

  this->cameraIface->Lock(1);

  // Data timestamp
  data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

  data->width = this->myParent->GetCamera()->GetImageWidth();
  data->height = this->myParent->GetCamera()->GetImageHeight();
  data->image_size = data->width * data->height * this->myParent->GetCamera()->GetImageDepth();

  // GetFOV() returns radians
  data->hfov = *(this->myParent->GetCamera()->GetHFOV());
  data->vfov = *(this->myParent->GetCamera()->GetVFOV());

  // Set the pose of the camera
  cameraPose = this->myParent->GetWorldPose();
  data->camera_pose.pos.x = cameraPose.pos.x;
  data->camera_pose.pos.y = cameraPose.pos.y;
  data->camera_pose.pos.z = cameraPose.pos.z;
  data->camera_pose.roll = cameraPose.rot.GetRoll();
  data->camera_pose.pitch = cameraPose.rot.GetPitch();
  data->camera_pose.yaw = cameraPose.rot.GetYaw();

  // Make sure there is room to store the image
  assert (data->image_size <= sizeof(data->image));

  // Copy the pixel data to the interface
  src = this->myParent->GetCamera()->GetImageData(0);
  dst = data->image;

  // NATY
  //boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());
  memcpy(dst, src, data->image_size);

  this->myParent->GetCamera()->EnableSaveFrame( data->saveFrames );

  this->cameraIface->Unlock();

  // New data is available
  this->cameraIface->Post();
}

