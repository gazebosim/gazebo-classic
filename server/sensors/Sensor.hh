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
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 * SVN: $Id$
 */

#ifndef SENSOR_HH
#define SENSOR_HH

#include "Entity.hh"
#include "UpdateParams.hh"

namespace gazebo
{
  class XMLConfigNode;
  class Body;
  class Controller;

  /// \addtogroup gazebo_sensor
  /// \brief Base class for sensors
  /// \{
  
  /// \brief Base class for sensors
  class Sensor : public Entity
  {
    /// \brief  Constructor
    public: Sensor(Body *body);
  
    /// \brief  Destructor
    public: virtual ~Sensor();
  
    ///  \brief Load the sensor
    /// \param node XMLConfigNode pointer
    public: virtual void Load(XMLConfigNode *node);
  
    /// \brief  Initialize the sensor
    public: void Init();
  
    /// \brief  Update the sensor
    public: void Update(UpdateParams &params);
  
    /// \brief  Finalize the sensor
    public: void Fini();

    /// \brief Set whether the sensor is active or not
    public: void SetActive(bool value);

    /// \brief  Load the child sensor
    protected: virtual void LoadChild(XMLConfigNode * /*node*/) {};
  
    /// \brief  Initialize the child
    protected: virtual void InitChild() {};
  
    /// \brief  Update the child
    protected: virtual void UpdateChild(UpdateParams & /*param*/) {};
  
    /// \brief Finalize the child
    protected: virtual void FiniChild() {};
  
    /// \brief Load a controller for this sensor
    /// \param node XML configure parameter node
    private: void LoadController(XMLConfigNode *node);
  
    /// The body this sensor is attached to
    protected: Body *body;

    /// \brief Pointer to the controller of the sensor
    protected: Controller *controller;

    /// \brief True if active
    protected: bool active;
  };
  
  /// \}
}
#endif
