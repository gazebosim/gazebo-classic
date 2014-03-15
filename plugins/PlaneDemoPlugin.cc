/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "kbhit.h"

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/PlaneDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PlaneDemoPlugin)

/////////////////////////////////////////////////
PlaneDemoPlugin::PlaneDemoPlugin()
{
  // engine
  this->throttleState = 0;
}

/////////////////////////////////////////////////
PlaneDemoPlugin::~PlaneDemoPlugin()
{
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "PlaneDemoPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "PlaneDemoPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "PlaneDemoPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "PlaneDemoPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    this->linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(this->linkName);
  }

  if (_sdf->HasElement("engine"))
  {
    sdf::ElementPtr enginePtr = _sdf->GetElement("engine");
    if (enginePtr->HasElement("engine_joint"))
    {
      std::string ejn = enginePtr->Get<std::string>("engine_joint");
      this->engineJoint = this->model->GetJoint(ejn);
    }
  }

  if (_sdf->HasElement("control"))
  {
    sdf::ElementPtr controlPtr = _sdf->GetElement("control");
    if (controlPtr->HasElement("cl_inc_key"))
    {
      this->clIncKey =
        (int)(*(controlPtr->Get<std::string>("cl_inc_key").c_str()));
      gzerr << " clIncKey: " << this->clIncKey << "\n";
    }
  }
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&PlaneDemoPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void PlaneDemoPlugin::OnUpdate()
{
  char ch='x';
  if( _kbhit() )
  {
    gzerr << this->linkName << "\n";
    printf("you hit");
    do
    {
      ch = getchar();
      printf(" '%c'(%i)", isprint(ch)?ch:'?', (int)ch );
    } while( _kbhit() );
    // puts("");

    gzerr << " clIncKey: " << this->clIncKey << "\n";
    if ((int)ch == 97)
    {
      // spin up motor
      this->throttleState += 50;
      gzerr << "torque: " << this->throttleState << "\n";
    }
    else if ((int)ch == 122)
    {
      this->throttleState -= 50;
      gzerr << "torque: " << this->throttleState << "\n";
    }
    else if (((int)ch) == this->clIncKey)
    {
      // gzerr << "increasing lift " << this->jointAngle
      //       << " : " << this->clIncKey << "\n";
    }
    else
    {
      ungetc( ch, stdin );
      gzerr << (int)ch << " : " << this->clIncKey << "\n";
    }

  }

  // spin up engine
  if (this->engineJoint)
    this->engineJoint->SetForce(0, this->throttleState);
}
