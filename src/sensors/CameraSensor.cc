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

#include "physics/World.hh"

#include "rendering/Camera.hh"
#include "rendering/Scene.hh"
#include "rendering/RenderEngine.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor)
 
//////////////////////////////////////////////////////////////////////////////
// Constructor
CameraSensor::CameraSensor()
    : Sensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Set the parent of the sensor
void CameraSensor::SetParent( const std::string &_name )
{
  Sensor::SetParent(_name);
}
   
//////////////////////////////////////////////////////////////////////////////
// Load the camera with SDF parameters
void CameraSensor::Load( sdf::ElementPtr &_sdf )
{
  Sensor::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
/// Load the camera using default parameters
void CameraSensor::Load()
{
  Sensor::Load();
  this->poseSub = this->node->Subscribe("~/pose", 
                                        &CameraSensor::OnPose, this );
}
 
//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void CameraSensor::Init()
{
  std::string worldName = this->sdf->GetWorldName();

  if (!worldName.empty())
  {
    this->scene = rendering::RenderEngine::Instance()->GetScene(worldName);
    if (!this->scene)
      this->scene = rendering::RenderEngine::Instance()->CreateScene(worldName);

    this->camera = this->scene->CreateCamera(
        this->sdf->GetValueString("name"), false);

    if (!this->camera)
    {
      gzerr << "Unable to create camera sensor[mono_camera]\n";
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
    this->camera->CreateRenderTexture(this->GetName() + "_RttTex");
    this->camera->SetWorldPose( this->pose );
    this->camera->AttachToVisual( this->parentName, true );
  }
  else
    gzerr << "No world name\n";

  Sensor::Init();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void CameraSensor::Fini()
{
  Sensor::Fini();
  this->camera->Fini();
  this->camera.reset();
  this->scene.reset();
}

//////////////////////////////////////////////////////////////////////////////
/// Set whether the sensor is active or not
void CameraSensor::SetActive(bool value)
{
  Sensor::SetActive(value);
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void CameraSensor::UpdateImpl(bool /*_force*/)
{
  if (this->camera)
  {
    this->camera->Render();
    this->camera->PostRender();
    this->lastUpdateTime = this->world->GetSimTime();
  }
}

void CameraSensor::OnPose(const boost::shared_ptr<msgs::Pose const> &/*_msg*/)
{
}
