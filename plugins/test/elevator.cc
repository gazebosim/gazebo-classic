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
#include <gazebo/gazeboserver.hh>
#include <boost/bind.hpp>

namespace gazebo
{
  class Elevator : public Plugin
  {
    /// Constructor
    public: Elevator() : Plugin() 
    {
      this->open = 0;
      this->raise = 0;
    }

    /// Mandatory load function, neded by gazebo
    public: void Load()
    {
      // Get the right and left doors, and the lift
      this->rightDoor = (Body*)World::Instance()->GetEntityByName("doors::right");
      this->leftDoor = (Body*)World::Instance()->GetEntityByName("doors::left");
      this->lift = (Body*)World::Instance()->GetEntityByName("lift::body");

      // Get the first contact sensor
      this->contact = (Geom*)World::Instance()->GetEntityByName("contact::body::COM_Entity::geom");
      this->contact->SetContactsEnabled(true);
      this->contact->ConnectContactCallback(boost::bind(&Elevator::ContactCB, this));


      // Get the second contact sensor
      this->contact2 = (Geom*)World::Instance()->GetEntityByName("lift::body::COM_Entity::contact");
      this->contact2->SetContactsEnabled(true);
      this->contact2->ConnectContactCallback(boost::bind(&Elevator::Contact2CB, this));

      // Get an update callback
      World::Instance()->ConnectWorldUpdateStartSignal(
            boost::bind(&Elevator::UpdateCB, this));
    }

    /// Gazebo callback that contact occured on the plate in front of the door
    private: void ContactCB()
    {
      this->open = 1;
    }

    /// Gazebo callback that contact occured on the plate in the elevator
    private: void Contact2CB()
    {
      this->raise = 1;
    }

    /// Gazebo update callback
    public: void UpdateCB()
    {
      if (this->open == 1)
      {
        this->rightDoor->SetLinearVel(Vector3(0, -0.5, 0));
        this->leftDoor->SetLinearVel(Vector3(0, 0.5, 0));
      }

      if (this->raise == 1)
      {
        this->lift->SetLinearVel(Vector3(0,0,0.5));
      }
    }

    private: int open, raise;
    private: Body *rightDoor, *leftDoor, *lift;
    private: Geom *contact, *contact2;
  };

  /// Register this plugin with gazebo. This is mandatory
  GZ_REGISTER_PLUGIN("Elevator", Elevator)
}
