/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Contact Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef GAZEBO_RAY_PLUGIN_HH
#define GAZEBO_RAY_PLUGIN_HH

#include "common/Plugin.hh"
#include "sensors/SensorTypes.hh"
#include "sensors/RaySensor.hh"
#include "gazebo.hh"

namespace gazebo
{
  /// \brief A Bumper controller
  class RayPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: RayPlugin();

    /// \brief Destructor
    public: virtual ~RayPlugin();

    // update callback
    public: virtual void OnNewLaserScans();
    private: event::ConnectionPtr newLaserScansConnection;

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world;

    /// \brief The parent sensor
    private: sensors::RaySensorPtr parentSensor;
  };
}

#endif

