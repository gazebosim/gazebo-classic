/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include "plugins/TransporterPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(TransporterPlugin)

/////////////////////////////////////////////////
TransporterPlugin::TransporterPlugin()
{
}

/////////////////////////////////////////////////
TransporterPlugin::~TransporterPlugin()
{
}

/////////////////////////////////////////////////
void TransporterPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "TransporterPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "TransporterPlugin sdf pointer is NULL");
  this->world = _world;
  this->sdf = _sdf;

  sdf::ElementPtr padElem = _sdf->GetElement("pad");
  while (padElem)
  {
    TransporterPlugin::Pad *pad = new TransporterPlugin::Pad;

    pad->name = padElem->Get<std::string>("name");
    pad->dest = padElem->Get<std::string>("destination");

    sdf::ElementPtr outElem = padElem->GetElement("outgoing");
    pad->outgoingPose = outElem->Get<math::Pose>("pose");
    std::cout << pad->outgoingPose << std::endl;
    /*outElem->Get<math::Vector3>("box") >> pad->outgoingBox;

    sdf::ElementPtr inElem = padElem->GetElement("incoming");
    inElem->Get<std::string>("pose") >> pad->incomingPose;
    inElem->Get<math::Vector3>("box") >> pad->incomingBox;
    */

    this->pads[pad->name] = pad;

    padElem = padElem->GetNextElement("pad");
  }
}
