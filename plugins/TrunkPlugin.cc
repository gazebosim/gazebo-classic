/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>
#include <string>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "plugins/TrunkPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TrunkPlugin)

/// \brief Private data class
class gazebo::TrunkPluginPrivate
{
  /// \brief Connection to World Update events.
  public: event::ConnectionPtr updateConnection;

  /// \brief Communication node
  public: ignition::transport::Node node;

  /// \brief Flag to indicate messsage is published
  public: bool done = false;

  // \brief Pointer to the world.
  public: physics::WorldPtr world;
};

/////////////////////////////////////////////////
TrunkPlugin::TrunkPlugin()
: dataPtr(new TrunkPluginPrivate)
{
}

/////////////////////////////////////////////////
TrunkPlugin::~TrunkPlugin()
{
}

/////////////////////////////////////////////////
void TrunkPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "TrunkPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "TrunkPlugin _sdf pointer is NULL");

  this->dataPtr->world = _model->GetWorld();
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrunkPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void TrunkPlugin::OnUpdate()
{
  if (this->dataPtr->done)
    return;

  if (this->dataPtr->world->SimTime().Double() < 3.0)
    return;
  // Message
  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("plot_trunk");
  markerMsg.set_id(111);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);

  // Material
  std::string mat = "Gazebo/Trunk";

  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name(mat);

  ignition::math::Vector3d point;
  double a,b;
  for(uint i = 0; i < 100; i++) {
    a = ((double) rand() / (RAND_MAX)) * 10.0;
    b = ((double) rand() / (RAND_MAX)) * 10.0;
    point.Set(a, b, 0.01);
    ignition::msgs::Set(markerMsg.add_point(), point);
    a = ((double) rand() / (RAND_MAX)) * 10.0;
    b = ((double) rand() / (RAND_MAX)) * 10.0;
    point.Set(a, b, 0.01);
    ignition::msgs::Set(markerMsg.add_point(), point);
    a = ((double) rand() / (RAND_MAX)) * 10.0;
    b = ((double) rand() / (RAND_MAX)) * 10.0;
    point.Set(a, b, 0.01);
    ignition::msgs::Set(markerMsg.add_point(), point);
  }

  bool result;
  ignition::msgs::StringMsg strMsg;
  this->dataPtr->node.Request(
        "/marker", markerMsg,
        10000,
        strMsg,
        result);
  std::cerr << "result: " << result << std::endl;
  /* ignition::math::Vector3d point;
  point.Set(0, 0, 0.01);
  ignition::msgs::Set(markerMsg.add_point(), point);
  point.Set(1, 0, 0.01);
  ignition::msgs::Set(markerMsg.add_point(), point);
  point.Set(0, 1, 0.01);
  ignition::msgs::Set(markerMsg.add_point(), point);*/

  this->dataPtr->node.Request("/marker", markerMsg);

  this->dataPtr->done = true;
}
