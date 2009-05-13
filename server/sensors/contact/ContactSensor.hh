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

#include "Angle.hh"
#include "Sensor.hh"
#include "Body.hh"

#define GAZEBO_MAX_CONTACT_FB_DATA 10

namespace gazebo
{

  class XMLConfigNode;
  class ContactParams;

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

    /// \brief Return the number of contacts
    public: unsigned int GetContactCount() const;

    /// \brief Get a contact time
    public: double GetContactTime(unsigned int index) const;

    /// \brief Return a contact state
    public: uint8_t GetContactState(unsigned int index) const;

    /// \brief Return contact geometry name
    public: std::string GetContactGeomName(unsigned int index) const;

    /// \brief Return contact feedback, f1,f2,t1,t2
    public:  dJointFeedback GetContactFeedback(unsigned int index) const;

    /// \brief Return geometry name
    public: std::string GetGeomName(unsigned int index) const;

    /// \brief Reset the contact states
    public: void ResetContactStates();

    /// Load the contact sensor using parameter from an XMLConfig node
    /// \param node The XMLConfig node
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// \brief Save the sensor info in XML format
    protected: virtual void SaveChild(std::string &prefix,std::ostream &stream);

    /// Initialize the sensor
    protected: virtual void InitChild();
  
    ///  Update sensed values
    protected: virtual void UpdateChild();
    
    /// Finalize the sensor
    protected: virtual void FiniChild();

    /// \brief Contact callback
    private: void ContactCallback(Geom *g1, Geom *g2);

    /// Geom name parameter
    private: std::vector< ParamT<std::string> *> geomNamesP;

    private: std::vector<std::string> geomNames;
    private: uint8_t *contactStates;
    private: double *contactTimes;
    private: unsigned int contactCount;
    private: std::vector<std::string> contactNames;
    /// \brief a place for storing joint feedbacks, including contact joints
    private:  dJointFeedback contactFeedbacks[GAZEBO_MAX_CONTACT_FB_DATA];

  };
  /// \}
  /// \}
}

#endif
