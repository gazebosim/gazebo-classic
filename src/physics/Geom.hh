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
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef GEOM_HH
#define GEOM_HH

#include "common/Event.hh"
#include "common/CommonTypes.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/Mass.hh"
#include "physics/Entity.hh"

namespace gazebo
{
	namespace physics
  {
    /// \brief Base class for all geoms
    class Geom : public Entity
    {
      /// \brief Constructor
      public: Geom(BodyPtr body);
    
      /// \brief Destructor
      public: virtual ~Geom();
  
      /// \brief Finalize the geom
      public: void Fini();
  
      /// \brief Load the geom
      public: virtual void Load(common::XMLConfigNode *node);

      public: virtual void Init();
  
      /// \brief Load the geom
      public: void Save(std::string &prefix, std::ostream &stream);
   
      /// \brief Set the encapsulated geometry object
      public: void SetGeom(bool placeable);
    
      /// \brief Update function for geoms
      public: void Update();
   
      /// \brief Return whether this geom is placeable
      public: bool IsPlaceable() const;
      
      /// \brief Set the category bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCategoryBits(unsigned int bits) = 0;
    
      /// \brief Set the collide bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCollideBits(unsigned int bits) = 0;
    
      /// \brief Get the mass of the geom
      public: virtual Mass GetBodyMassMatrix() = 0;
    
      /// \brief Set the laser fiducial integer id
      public: void SetLaserFiducialId(int id);
    
      /// \brief Get the laser fiducial integer id
      public: int GetLaserFiducialId() const;
    
      /// \brief Set the laser retro reflectiveness 
      public: void SetLaserRetro(float retro);
    
      /// \brief Get the laser retro reflectiveness 
      public: float GetLaserRetro() const;
  
      /// \brief Set the visibility of the bounding box
      public: void ShowBoundingBox(const bool &show);
  
      /// \brief Set the mass
      public: void SetMass(const double &mass);
  
      /// \brief Set the mass
      public: void SetMass(const Mass &mass);
  
      /// \brief Get the body this geom belongs to
      public: BodyPtr GetBody() const;
  
      /// \brief Get the model this geom belongs to
      public: ModelPtr GetModel() const;
  
      /// \brief Set the friction mode of the geom
      public: void SetFrictionMode( const bool &v );
  
      /// \brief Get the bounding box for this geom
      public: virtual math::Box GetBoundingBox() const = 0;
  
      /// \brief Get a pointer to the mass
      public: const Mass &GetMass() const;
  
      /// \brief Get the shape type
      public: Base::EntityType GetShapeType();
  
      /// \brief Set the shape for this geom
      public: void SetShape(ShapePtr shape);
              
      /// \brief Get the attached shape
      public: ShapePtr GetShape() const;
  
      /// \brief Turn contact recording on or off
      public: void SetContactsEnabled(const bool &enable);
  
      /// \brief Return true of contact recording is on
      public: bool GetContactsEnabled() const;
  
      /// \brief Add an occurance of a contact to this geom
      public: void AddContact(const Contact &contact);
  
      /// \brief Clear all contact info
      public: void ClearContacts();
  
      /// \brief Get the number of contacts
      public: unsigned int GetContactCount() const;
              
      /// \brief Get a specific contact
      public: Contact GetContact(unsigned int i) const;
  
      /// \brief Get the linear velocity of the geom
      public: virtual math::Vector3 GetRelativeLinearVel() const;
  
      /// \brief Get the linear velocity of the geom in the world frame
      public: virtual math::Vector3 GetWorldLinearVel() const;
  
      /// \brief Get the angular velocity of the geom
      public: virtual math::Vector3 GetRelativeAngularVel() const;
  
      /// \brief Get the angular velocity of the geom in the world frame
      public: virtual math::Vector3 GetWorldAngularVel() const;
  
      /// \brief Get the linear acceleration of the geom
      public: virtual math::Vector3 GetRelativeLinearAccel() const;
              
      /// \brief Get the linear acceleration of the geom in the world frame
      public: virtual math::Vector3 GetWorldLinearAccel() const;
  
      /// \brief Get the angular acceleration of the geom
      public: virtual math::Vector3 GetRelativeAngularAccel() const;
  
      /// \brief Get the angular acceleration of the geom in the world frame
      public: virtual math::Vector3 GetWorldAngularAccel() const;
  
      public: template< typename T>
              event::ConnectionPtr ConnectContactCallback( T subscriber )
              { return contactSignal.Connect(subscriber); }
      public: void DisconnectContactCallback( event::ConnectionPtr &c )
              { contactSignal.Disconnect(c); }
  
      /// \brief Enable callback: Called when the body changes
      private: void EnabledCB(bool enabled);
  
      /// \brief Create the bounding box for the geom
      private: void CreateBoundingBox();
  
      ///  Contact parameters
      public: SurfaceParamsPtr surface; 
  
      /// The body this geom belongs to
      protected: BodyPtr body;
    
      protected: bool placeable;
  
      protected: Mass mass;
  
      private: common::ParamT<int> *laserFiducialIdP;
      private: common::ParamT<float> *laserRetroP;
  
      ///  Mass as a double
      private: common::ParamT<double> *massP;
      protected: common::ParamT<math::Vector3> *xyzP;
      protected: common::ParamT<math::Quatern> *rpyP;
      protected: common::ParamT<bool> *enableContactsP;
  
      private: float transparency;
  
      /// All the visual apparence 
      private: std::string bbVisual;
  
      protected: ShapePtr shape;
  
      private: bool contactsEnabled;
  
      public: event::EventT< void (const Contact &)> contactSignal;
      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
