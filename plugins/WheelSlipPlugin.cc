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
#include <map>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/physics/ode/ODETypes.hh>

#include "plugins/WheelSlipPlugin.hh"

namespace gazebo
{
  class WheelSlipPluginPrivate
  {
    public: class LinkSurfaceParams
    {
      /// \brief Pointer to wheel spin joint.
      public: physics::JointPtr joint = nullptr;

      /// \brief Pointer to ODESurfaceParams object.
      public: physics::ODESurfaceParamsPtr surface = nullptr;

      /// \brief Unitless wheel slip compliance in lateral direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLateral = 0;

      /// \brief Unitless wheel slip compliance in longitudinal direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLongitudinal = 0;

      /// \brief Wheel normal force estimate used to compute slip
      /// compliance for ODE, which takes units of 1/N.
      public: double wheelNormalForce = 0;

      /// \brief Wheel radius extracted from collision shape if not
      /// specified as xml parameter.
      public: double wheelRadius = 0;
    };

    /// \brief Model pointer.
    public: physics::ModelPtr model;

    /// \brief Link and surface pointers to update.
    public: std::map<physics::LinkPtr, LinkSurfaceParams> mapLinkSurfaceParams;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;
  };
}

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(WheelSlipPlugin)

/////////////////////////////////////////////////
WheelSlipPlugin::WheelSlipPlugin()
  : dataPtr(new WheelSlipPluginPrivate)
{
}

/////////////////////////////////////////////////
WheelSlipPlugin::~WheelSlipPlugin()
{
}

/////////////////////////////////////////////////
void WheelSlipPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "WheelSlipPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "WheelSlipPlugin sdf pointer is NULL");

  this->dataPtr->model = _model;

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return;
  }

  // Read each wheel element
  auto wheelElem = _sdf->GetElement("wheel");
  while (wheelElem)
  {
    if (!wheelElem->HasAttribute("link_name"))
    {
      gzerr << "wheel element missing link_name attribute" << std::endl;
      wheelElem = wheelElem->GetNextElement("wheel");
      continue;
    }

    // Get link name
    auto linkName = wheelElem->Get<std::string>("link_name");

    WheelSlipPluginPrivate::LinkSurfaceParams params;
    if (wheelElem->HasElement("slip_compliance_lateral"))
    {
      params.slipComplianceLateral =
        wheelElem->Get<double>("slip_compliance_lateral");
    }
    if (wheelElem->HasElement("slip_compliance_longitudinal"))
    {
      params.slipComplianceLongitudinal =
        wheelElem->Get<double>("slip_compliance_longitudinal");
    }
    if (wheelElem->HasElement("wheel_normal_force"))
    {
      params.wheelNormalForce = wheelElem->Get<double>("wheel_normal_force");
    }

    if (wheelElem->HasElement("wheel_radius"))
    {
      params.wheelRadius = wheelElem->Get<double>("wheel_radius");
    }

    // Get the next link element
    wheelElem = wheelElem->GetNextElement("wheel");

    auto link = _model->GetLink(linkName);
    if (link == nullptr)
    {
      gzerr << "Could not find link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto collisions = link->GetCollisions();
    if (collisions.empty() || collisions.size() != 1)
    {
      gzerr << "There should be 1 collision in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << collisions.size() << " were found"
            << std::endl;
      continue;
    }
    auto collision = collisions.front();
    if (collision == nullptr)
    {
      gzerr << "Could not find collision in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto surface = collision->GetSurface();
    auto odeSurface =
      boost::dynamic_pointer_cast<physics::ODESurfaceParams>(surface);
    if (odeSurface == nullptr)
    {
      gzerr << "Could not find ODE Surface "
            << "in collision named [" << collision->GetName()
            << "] in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.surface = odeSurface;

    auto joints = link->GetParentJoints();
    if (joints.empty() || joints.size() != 1)
    {
      gzerr << "There should be 1 parent joint for link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << joints.size() << " were found"
            << std::endl;
      continue;
    }
    auto joint = joints.front();
    if (joint == nullptr)
    {
      gzerr << "Could not find parent joint for link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.joint = joint;

    if (params.wheelRadius <= 0)
    {
      // get collision shape and extract radius if it is a cylinder or sphere
      auto shape = collision->GetShape();
      if (shape->HasType(physics::Base::CYLINDER_SHAPE))
      {
        auto cyl = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
        if (cyl != nullptr)
        {
          params.wheelRadius = cyl->GetRadius();
        }
      }
      else if (shape->HasType(physics::Base::SPHERE_SHAPE))
      {
        auto sphere = boost::dynamic_pointer_cast<physics::SphereShape>(shape);
        if (sphere != nullptr)
        {
          params.wheelRadius = sphere->GetRadius();
        }
      }

      // if that still didn't work, skip this link
      if (params.wheelRadius <= 0)
      {
        gzerr << "Found wheel radius [" << params.wheelRadius
              << "], which is not positive"
              << " in link named [" << linkName
              << "] in model [" << _model->GetScopedName() << "]"
              << std::endl;
        continue;
      }
    }

    if (params.wheelNormalForce <= 0)
    {
      gzerr << "Found wheel normal force [" << params.wheelNormalForce
            << "], which is not positive"
            << " in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    this->dataPtr->mapLinkSurfaceParams[link] = params;
  }

  // Connect to the update event
  if (this->dataPtr->mapLinkSurfaceParams.empty())
  {
    gzerr << "No ODE links and surfaces found, plugin is disabled" << std::endl;
    return;
  }
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&WheelSlipPlugin::Update, this));
}

/////////////////////////////////////////////////
physics::ModelPtr WheelSlipPlugin::GetParentModel() const
{
  return this->dataPtr->model;
}

/////////////////////////////////////////////////
void WheelSlipPlugin::SetSlipComplianceLateral(const double _compliance)
{
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipComplianceLateral = _compliance;
  }
}

/////////////////////////////////////////////////
void WheelSlipPlugin::SetSlipComplianceLongitudinal(const double _compliance)
{
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipComplianceLongitudinal = _compliance;
  }
}

/////////////////////////////////////////////////
void WheelSlipPlugin::Update()
{
  for (auto linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto params = linkSurface.second;
    double force = params.wheelNormalForce;
    auto omega = params.joint->GetVelocity(0);
    double speed = std::abs(omega) * params.wheelRadius;
    // auto speed = linkSurface.first->GetWorldLinearVel().Ign().Length();
    params.surface->slip1 = speed / force * params.slipComplianceLateral;
    params.surface->slip2 = speed / force * params.slipComplianceLongitudinal;
  }
}
