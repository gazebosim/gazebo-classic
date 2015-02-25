/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/physics/World.hh"

#include "gazebo/physics/rtql8/RTQL8Physics.hh"
#include "gazebo/physics/rtql8/RTQL8Model.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Model::RTQL8Model(BasePtr _parent)
  : Model(_parent), rtql8SkeletonDynamics(NULL)
{
  
}

//////////////////////////////////////////////////
RTQL8Model::~RTQL8Model()
{
  if (rtql8SkeletonDynamics)
    delete rtql8SkeletonDynamics;
}

//////////////////////////////////////////////////
void RTQL8Model::Load(sdf::ElementPtr _sdf)
{
  if (rtql8SkeletonDynamics)
    delete rtql8SkeletonDynamics;

  rtql8SkeletonDynamics = new rtql8::dynamics::SkeletonDynamics();

  // add skeleton to world
  //boost::shared_dynamic_cast<RTQL8World>(this->world);
  // TODO: How to access to rtql8's world in here?
  RTQL8PhysicsPtr rtql8Physics =
      boost::shared_dynamic_cast<RTQL8Physics>(this->GetWorld()->GetPhysicsEngine());

  rtql8Physics->GetRTQL8World()->addSkeleton(rtql8SkeletonDynamics);

  Model::Load(_sdf);

  // TODO:
  // This should be set by sdf.
  rtql8SkeletonDynamics->setImmobileState(false);
}

//////////////////////////////////////////////////
void RTQL8Model::Init()
{
  Model::Init();
  
}


//////////////////////////////////////////////////
void RTQL8Model::Update()
{
  Model::Update();
  
}

//////////////////////////////////////////////////
void RTQL8Model::Fini()
{
  Model::Fini();
  
}

//////////////////////////////////////////////////
