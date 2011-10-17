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
/* Desc: A camera sensor using OpenGL
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#include <sstream>

#include "common/Events.hh"
#include "common/Exception.hh"

#include "transport/transport.h"

#include "rendering/DepthCamera.hh"
#include "rendering/Scene.hh"
#include "rendering/RenderEngine.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/DepthCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("depth", DepthCameraSensor)
 
//////////////////////////////////////////////////////////////////////////////
// Constructor
DepthCameraSensor::DepthCameraSensor()
    : Sensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
DepthCameraSensor::~DepthCameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera with SDF parameters
void DepthCameraSensor::Load( sdf::ElementPtr &_sdf )
{
  Sensor::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
/// Load the camera using default parameters
void DepthCameraSensor::Load()
{
  Sensor::Load();
  this->poseSub = this->node->Subscribe("~/pose", 
                                        &DepthCameraSensor::OnPose, this );
}
 
//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void DepthCameraSensor::InitChild()
{
  std::string worldName = this->sdf->GetWorldName();

  if (!worldName.empty())
  {
    rendering::ScenePtr scene = rendering::RenderEngine::Instance()->GetScene(worldName);
    if (!scene)
      scene = rendering::RenderEngine::Instance()->CreateScene(worldName);

    this->camera = scene->CreateDepthCamera(this->sdf->GetValueString("name"));
    if (!this->camera)
    {
      gzerr << "Unable to create depth camera sensor\n";
      return;
    }
    this->camera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->sdf->GetOrCreateElement("camera");
    this->camera->Load( cameraSdf );

    // Do some sanity checks
    if (this->camera->GetImageWidth() == 0 || 
        this->camera->GetImageHeight() == 0)
    {
      gzthrow("image has zero size");
    }

    this->camera->Init();

    //this->camera->SetWorldPosition(math::Vector3(0, 0, 5));
    //this->camera->SetWorldRotation( math::Quaternion::EulerToQuaternion(0, DTOR(15), 0) );
  }
  else
    gzerr << "No world name\n";
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void DepthCameraSensor::Fini()
{
  Sensor::Fini();
  this->camera->Fini();
}

//////////////////////////////////////////////////////////////////////////////
/// Set whether the sensor is active or not
void DepthCameraSensor::SetActive(bool value)
{
  Sensor::SetActive(value);
}

//////////////////////////////////////////////////////////////////////////////
// Render new data
/*void DepthCameraSensor::Render()
{
  //if (this->active || **this->alwaysActiveP)
  {
    //this->lastUpdate = this->GetWorld()->GetSimTime();
    //this->camera->Render();
    //this->camera->PostRender();
  }
  
}*/

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void DepthCameraSensor::Update(bool force)
{
  Sensor::Update(force);
  /*if (this->camera)
  {
    this->camera->Render();
    this->camera->PostRender();
  }*/

  // NATY
  //if (this->active || **this->alwaysActiveP)
    //this->camera->Update();

  // Only continue if the controller has an active interface. Or frames need
  // to be saved
  /*if ( (this->controller && !this->controller->IsConnected()) &&
       !this->saveFramesP->GetValue())
    return;

  // Or skip if user sets camera to inactive
  if (this->active)
    this->UpdateCam();
    */
}

void DepthCameraSensor::OnPose(const boost::shared_ptr<msgs::Pose const> &/*_msg*/)
{
  //if (_msg->header().str_id() == "world_1::camera_model")
  //  gzdbg << "On Pose[" << _msg->header().str_id() << "][" << _msg->position().z() << "]\n";
}
