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
/* Desc: Contact sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * SVN: $Id$
*/

#ifndef CONTACTSENSOR_HH
#define CONTACTSENSOR_HH

#include <vector>

#include <stdint.h>

#include "Angle.hh"
#include "Sensor.hh"
#include "Body.hh"
#include "Contact.hh"

namespace gazebo
{

  class XMLConfigNode;

  /// \addtogroup gazebo_sensor
  /// \brief Contact sensor.
  /// \{
  /// \defgroup gazebo_ray Contact
  /// \brief Contact sensor.
  // \{
  
  /// \brief Contact sensor.
  ///
  /// This sensor detects and reports contacts between objects 
  class ContactSensor: public Sensor
  {
    /// \brief Constructor
    /// \param body The underlying collision test uses an ODE geom, so
    ///             ray sensors must be attached to a body.
    public: ContactSensor(Body *body);
  
    /// \brief Destructor
    public: virtual ~ContactSensor();

    /// Load the contact sensor using parameter from an XMLConfig node
    /// \param node The XMLConfig node
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// Initialize the sensor
    protected: virtual void InitChild();
  
    ///  Update sensed values
    protected: virtual void UpdateChild();

     /// \brief Save the sensor info in XML format
    protected: virtual void SaveChild(std::string &prefix,std::ostream &stream);
   
    /// Finalize the sensor
    protected: virtual void FiniChild();

    /// \brief Get the number of geoms that the sensor is observing
    public: unsigned int GetGeomCount() const;

    /// \brief Get a geom
    public: Geom *GetGeom(unsigned int index);

    /// \brief Return the number of contacts for an observed geom
    public: unsigned int GetGeomContactCount(unsigned int geomIndex) const;

    /// \brief Get a contact for a geom by index
    public: Contact GetGeomContact(unsigned int geom, unsigned int index) const;

    /// Geom name parameter
    private: std::vector< ParamT<std::string> *> geomNamesP;

    private: std::vector<Geom *> geoms;
  };
  /// \}
  /// \}
}

#endif
