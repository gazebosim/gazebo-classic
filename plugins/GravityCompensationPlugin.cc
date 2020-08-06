/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <boost/filesystem.hpp>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/dart/DARTTypes.hh>

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/sdf/sdf.hpp>

#include <Eigen/Core>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include "GravityCompensationPlugin.hh"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GravityCompensationPlugin)

namespace gazebo
{
  /// \internal
  /// \brief Private data for the GravityCompensationPlugin.
  class GravityCompensationPluginPrivate
  {
    public: GravityCompensationPluginPrivate()
            {
            }

    /// \brief The gazebo model.
    public: physics::ModelPtr model;

    /// \brief A DART skeleton for the model.
    public: dart::dynamics::SkeletonPtr skel;

    /// \brief Connects to world update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief Node for communication.
    public: transport::NodePtr node;

    /// \brief Subscribe to the "~/physics" topic.
    public: transport::SubscriberPtr physicsSub;
  };
}

/////////////////////////////////////////////////
bool ModelResourceRetriever::exists(const dart::common::Uri &_uri)
{
  return LocalResourceRetriever::exists(this->resolve(_uri));
}

/////////////////////////////////////////////////
dart::common::ResourcePtr ModelResourceRetriever::retrieve(
    const dart::common::Uri &_uri)
{
  return LocalResourceRetriever::retrieve(this->resolve(_uri));
}

/////////////////////////////////////////////////
dart::common::Uri ModelResourceRetriever::resolve(const dart::common::Uri &_uri)
{
  dart::common::Uri uri;
  if (_uri.mScheme.get_value_or("model") == "model")
  {
    uri.mScheme.assign("file");
    std::string modelPath
        = sdf::findFile("model://" + _uri.mAuthority.get() + _uri.mPath.get());
    if (boost::filesystem::exists(modelPath))
    {
      if (boost::filesystem::is_directory(modelPath))
      {
        modelPath = sdf::getModelFilePath(modelPath);
      }
      uri.mPath.assign(modelPath);
    }
  }
  return uri;
}

/////////////////////////////////////////////////
GravityCompensationPlugin::GravityCompensationPlugin()
  : dataPtr(new GravityCompensationPluginPrivate)
{
}

/////////////////////////////////////////////////
GravityCompensationPlugin::~GravityCompensationPlugin()
{
}

/////////////////////////////////////////////////
void GravityCompensationPlugin::Load(physics::ModelPtr _model,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  GZ_ASSERT(_sdf, "SDF pointer is null");

  this->dataPtr->model = _model;

  // Load a DART skeleton model
  if (_sdf->HasElement("uri"))
  {
    this->dataPtr->skel = dart::utils::SdfParser::readSkeleton(
        _sdf->Get<std::string>("uri"),
        std::make_shared<ModelResourceRetriever>());
    if (this->dataPtr->skel == nullptr)
    {
      gzerr << "Error parsing " << _sdf->Get<std::string>("uri") << "\n";
      return;
    }
  }
  else
  {
    gzerr << "Must specify a model URI\n";
    return;
  }

  if (this->dataPtr->model->GetWorld())
  {
    // Set gravity
    ignition::math::Vector3d g = this->dataPtr->model->GetWorld()->Gravity();
    this->dataPtr->skel->setGravity(Eigen::Vector3d(g.X(), g.Y(), g.Z()));

    // Subscribe to physics messages in case gravity changes
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());
    this->dataPtr->physicsSub = this->dataPtr->node->Subscribe("~/physics",
        &GravityCompensationPlugin::OnPhysicsMsg, this);
  }
  else
  {
    gzwarn << "Unable to get world name. "
        << "GravityCompensationPlugin will not receive physics messages\n";
  }

  // Check that for each joint in the model the skeleton has a matching one.
  physics::Joint_V joints = this->dataPtr->model->GetJoints();
  for (auto joint : joints)
  {
    dart::dynamics::Joint *dtJoint =
        this->dataPtr->skel->getJoint(joint->GetName());
    if (dtJoint == nullptr)
    {
      gzerr << "Missing joint \"" << joint->GetName()
            << "\" in DART skeleton.\n";
      return;
    }
    else if (dtJoint->getNumDofs() != joint->DOF())
    {
      gzerr << "Inconsistent number of DOF for joint \"" << joint->GetName()
            << ".\" The Gazebo joint has " << joint->DOF() << " DOF while"
            << " the DART joint has " << dtJoint->getNumDofs() << " DOF\n";
      return;
    }
  }

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GravityCompensationPlugin::Update, this,
      std::placeholders::_1));
}

/////////////////////////////////////////////////
void GravityCompensationPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  dart::dynamics::Joint *dtJoint = this->dataPtr->skel->getRootJoint();
  if (dtJoint == nullptr)
  {
    gzerr << "Failed to find root joint of DART skeleton\n";
  }

  // A free (root) joint won't be in the Gazebo model, so handle it seperately.
  auto dtFreeJoint = dynamic_cast<dart::dynamics::FreeJoint *>(dtJoint);
  if (dtFreeJoint != nullptr)
  {
    // Set skeleton pose
    dtFreeJoint->setTransform(
        physics::DARTTypes::ConvPose(this->dataPtr->model->WorldPose()));
    // Get model velocity
    ignition::math::Vector3d linVel = this->dataPtr->model->WorldLinearVel();
    ignition::math::Vector3d angVel = this->dataPtr->model->WorldAngularVel();
    // Set skeleton velocity
    dtFreeJoint->setLinearVelocity(
        Eigen::Vector3d(linVel.X(), linVel.Y(), linVel.Z()));
    dtFreeJoint->setAngularVelocity(
        Eigen::Vector3d(angVel.X(), angVel.Y(), angVel.Z()));
  }

  // Set skeleton joint positions and velocities
  physics::Joint_V joints = this->dataPtr->model->GetJoints();
  for (auto joint : joints)
  {
    dtJoint = this->dataPtr->skel->getJoint(joint->GetName());
    for (size_t i = 0; i < joint->DOF(); ++i)
    {
      dtJoint->setPosition(i, joint->Position(i));
      dtJoint->setVelocity(i, joint->GetVelocity(i));
    }
  }

  // Gravity compensation
  Eigen::VectorXd forces = this->dataPtr->skel->getCoriolisAndGravityForces();
  for (auto joint : joints)
  {
    dtJoint = this->dataPtr->skel->getJoint(joint->GetName());
    for (size_t i = 0; i < joint->DOF(); ++i)
    {
      joint->SetForce(i, forces[dtJoint->getIndexInSkeleton(i)]);
    }
  }
}

/////////////////////////////////////////////////
void GravityCompensationPlugin::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  if (_msg->has_gravity())
  {
    ignition::math::Vector3d g = msgs::ConvertIgn(_msg->gravity());
    this->dataPtr->skel->setGravity(Eigen::Vector3d(g.X(), g.Y(), g.Z()));
  }
}
