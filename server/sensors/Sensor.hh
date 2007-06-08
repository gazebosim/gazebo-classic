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

class Sensor : public Entity
{
  /// Constructor
  public: Sensor(Body *body);

  /// Destructor
  public: virtual ~Sensor();

  /// Load the sensor
  /// \param node XMLConfigNode pointer
  public: virtual void Load(XMLConfigNode *node);

  /// Initialize the sensor
  public: void Init();

  /// Update the sensor
  public: void Update(UpdateParams &params);

  /// Finalize the sensor
  public: void Fini();

  /// Load the child sensor
  protected: virtual void LoadChild(XMLConfigNode * /*node*/) {};

  /// Initialize the child
  protected: virtual void InitChild() {};

  /// Update the child
  protected: virtual void UpdateChild(UpdateParams & /*param*/) {};

  /// Finalize the child
  protected: virtual void FiniChild() {};

  /// The body this sensor is attached to
  protected: Body *body;
};

}
#endif
