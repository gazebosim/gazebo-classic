/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
    public: Geom *GetGeom(unsigned int index) const;

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
