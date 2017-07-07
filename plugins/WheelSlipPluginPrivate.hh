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
#ifndef GAZEBO_WHEEL_SLIP_PLUGIN_PRIVATE_HH_
#define GAZEBO_WHEEL_SLIP_PLUGIN_PRIVATE_HH_

#include <map>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/physics/ode/ODETypes.hh>

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
#endif
