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
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
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
      /// \brief Pointer to ODESurfaceParams object.
      public: physics::ODESurfaceParamsPtr surface = nullptr;

      /// \brief Lateral wheel slip compliance, with units 1/N.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLateral = 0;

      /// \brief Longitudinal wheel slip compliance, with units 1/N.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLongitudinal = 0;
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
    if (collisions.empty())
    {
      gzerr << "Could not find collision in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
    }
    auto collision = *(collisions.begin());
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
void WheelSlipPlugin::Update()
{
  for (auto linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto speed = linkSurface.first->GetWorldLinearVel().Ign().Length();
    linkSurface.second.surface->slip1 =
        speed * linkSurface.second.slipComplianceLateral;
    linkSurface.second.surface->slip2 =
        speed * linkSurface.second.slipComplianceLongitudinal;
  }
}
