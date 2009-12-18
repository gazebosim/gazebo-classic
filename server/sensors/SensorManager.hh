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
 * Desc: Class to manager all sensors
 * Author: Nate Koenig
 * Date: 18 Dec 2009
 * SVN info: $Id$
 */

#ifndef SENSORMANAGER_HH
#define SENSORMANAGER_HH

#include <list>

#include "SingletonT.hh"

namespace gazebo
{
  class Sensor;

  /// \brief Class to manage and update all sensors
  class SensorManager : public SingletonT<SensorManager>
  {
    /// \brief Constructor
    public: SensorManager();

    /// \brief Destructor
    public: virtual ~SensorManager();

    /// \brief Update all the sensors
    public: void Update();

    /// \brief Init all the sensors
    public: void Init();

    /// \brief Finalize all the sensors
    public: void Fini();

    /// \brief Add a sensor
    public: void AddSensor(Sensor *sensor);

    /// \brief Remove a sensor
    public: void RemoveSensor(Sensor *sensor);

    private: std::list<Sensor *> sensors;

    private: friend class DestroyerT<SensorManager>;
    private: friend class SingletonT<SensorManager>;
  };
}

#endif
