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
#ifndef _GAZEBO_TIRE_FRICTION_PLUGIN_HH_
#define _GAZEBO_TIRE_FRICTION_PLUGIN_HH_

#include <sdf/sdf.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  /// \brief Forward declare private data class.
  class TireFrictionPluginPrivate;

  /// \brief A plugin that simulates tire friction.
  class TireFrictionPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: TireFrictionPlugin();

    /// \brief Destructor.
    public: ~TireFrictionPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Function for computing friction from slip.
    /// \param[in] _slipSpeed Relative tangential speed in (m/s).
    /// \param[in] _referenceSpeed Reference speed in (m/s).
    /// \return Friction coefficient.
    public: virtual double ComputeFriction(const double _slipSpeed,
                                           const double _referenceSpeed);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Callback for contact filter.
    /// \param[in] _msg Contacts message.
    private: void OnContacts(ConstContactsPtr &_msg);

    /// \internal
    /// \brief Pointer to private data.
    protected: TireFrictionPluginPrivate *dataPtr;
  };
}
#endif
