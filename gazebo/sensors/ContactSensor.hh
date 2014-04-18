/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
*/

#ifndef _CONTACTSENSOR_HH_
#define _CONTACTSENSOR_HH_

#include <vector>
#include <map>
#include <list>
#include <string>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Angle.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class ContactSensor ContactSensor.hh sensors/sensors.hh
    /// \brief Contact sensor. This sensor detects and reports contacts between
    ///  objects
    class GAZEBO_VISIBLE ContactSensor: public Sensor
    {
      /// \brief Constructor.
      public: ContactSensor();

      /// \brief Destructor.
      public: virtual ~ContactSensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters.
      /// \param[in] _worldName Name of world to load from.
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the sensor.
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(bool _force);

      /// \brief Finalize the sensor.
      protected: virtual void Fini();

      /// \brief Get the number of collisions that the sensor is observing.
      /// \return Number of collisions.
      public: unsigned int GetCollisionCount() const;

      /// \brief Get a collision name at index _index.
      /// \param[in] _index Index of collision in collection of collisions.
      /// \return name of collision.
      public: std::string GetCollisionName(unsigned int _index) const;

      /// \brief Return the number of contacts for an observed collision.
      /// \param[in] _collisionName The name of the observed collision.
      /// \return The collision contact count.
      public: unsigned int GetCollisionContactCount(
                  const std::string &_collisionName) const;

      /// \brief Get all the contacts for the ContactSensor
      /// \return Message that contains contact information between collision
      /// pairs.
      ///
      /// During ODEPhysics::UpdateCollisions, all collision pairs in the
      /// world are pushed into a buffer within ContactManager.
      /// Subsequently, World::Update invokes ContactManager::PublishContacts
      /// to publish all contacts generated within a timestep onto
      /// Gazebo topic ~/physics/contacts.
      ///
      /// Each ContactSensor subscribes to the Gazebo ~/physics/contacts topic,
      /// retrieves all contact pairs in a time step and filters them wthin
      /// ContactSensor::OnContacts against <collision> body name
      /// specified by the ContactSensor SDF.
      /// All collision pairs between ContactSensor <collision> body and
      /// other bodies in the world are stored in an array inside
      /// contacts.proto.
      ///
      /// Within each element of the contact.proto array inside contacts.proto,
      /// list of collisions between collision bodies
      /// (collision1 and collision 2) are stored in an array of
      /// elements, (position, normal, depth, wrench).  A timestamp has also
      /// been added (time).  Details are described below:
      ///
      ///    \li string collision1  name of the first collision object.
      ///    \li string collision2  name of the second collision object.
      ///    \li Vector3d position  position of the contact joint in
      ///                           inertial frame.
      ///    \li Vector3d normal    normal of the contact joint in
      ///                           inertial frame.
      ///    \li double depth       intersection (penetration)
      ///                           depth of two collision bodies.
      ///    \li JointWrench wrench Forces and torques acting on both collision
      ///                           bodies.  See joint_wrench.proto for details.
      ///                           The forces and torques are applied at the
      ///                           CG of perspective links for each collision
      ///                           body, specified in the inertial frame.
      ///    \li Time time          time at which this contact happened.
      public: msgs::Contacts GetContacts() const;

      /// \brief Gets contacts of a collision
      /// \param[in] _collisionName Name of collision
      /// \return Container of contacts
      public: std::map<std::string, physics::Contact> GetContacts(
                  const std::string &_collisionName);

      // Documentation inherited.
      public: virtual bool IsActive();

      /// \brief Callback for contact messages from the physics engine.
      private: void OnContacts(ConstContactsPtr &_msg);

      /// \brief Collisions this sensor monitors for contacts
      private: std::vector<std::string> collisions;

      /// \brief Output contact information.
      private: transport::PublisherPtr contactsPub;

      /// \brief Subscription to contact messages from the physics engine
      private: transport::SubscriberPtr contactSub;

      /// \brief Mutex to protect reads and writes.
      private: mutable boost::mutex mutex;

      /// \brief Contacts message used to output sensor data.
      private: msgs::Contacts contactsMsg;

      typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;
      private: ContactMsgs_L incomingContacts;
    };
    /// \}
  }
}
#endif
