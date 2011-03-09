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
 * SVN: $Id$
 */

#ifndef GEOM_HH
#define GEOM_HH

#include <boost/bind.hpp>

#include "Event.hh"
#include "Contact.hh"
#include "Shape.hh"
#include "Param.hh"
#include "Entity.hh"
#include "Pose3d.hh"
#include "Vector3.hh"
#include "Mass.hh"

namespace gazebo
{
  class Model;
  class Body;
  class SurfaceParams;
  class XMLConfigNode;

  /// \addtogroup gazebo_physics
  /// \brief Base class for all geoms
  /// \{

  /// \brief Base class for all geoms
  class Geom : public Entity
  {
    /// \brief Constructor
    //public: Geom(Body *body, const std::string &name);
    public: Geom(Body *body);
  
    /// \brief Destructor
    public: virtual ~Geom();

    /// \brief Finalize the geom
    public: void Fini();

    /// \brief Load the geom
    public: virtual void Load(XMLConfigNode *node);

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
    public: Body *GetBody() const;

    /// \brief Get the model this geom belongs to
    public: Model *GetModel() const;

    /// \brief Set the friction mode of the geom
    public: void SetFrictionMode( const bool &v );

    /// \brief Get the bounding box for this geom
    public: virtual void GetBoundingBox(Vector3 &min, Vector3 &max) const = 0;

    /// \brief Get a pointer to the mass
    public: const Mass &GetMass() const;

    /// \brief Get the shape type
    public: EntityType GetShapeType();

    /// \brief Set the shape for this geom
    public: void SetShape(Shape *shape);
            
    /// \brief Get the attached shape
    public: Shape *GetShape() const;

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
    public: virtual Vector3 GetRelativeLinearVel() const;

    /// \brief Get the linear velocity of the geom in the world frame
    public: virtual Vector3 GetWorldLinearVel() const;

    /// \brief Get the angular velocity of the geom
    public: virtual Vector3 GetRelativeAngularVel() const;

    /// \brief Get the angular velocity of the geom in the world frame
    public: virtual Vector3 GetWorldAngularVel() const;

    /// \brief Get the linear acceleration of the geom
    public: virtual Vector3 GetRelativeLinearAccel() const;
            
    /// \brief Get the linear acceleration of the geom in the world frame
    public: virtual Vector3 GetWorldLinearAccel() const;

    /// \brief Get the angular acceleration of the geom
    public: virtual Vector3 GetRelativeAngularAccel() const;

    /// \brief Get the angular acceleration of the geom in the world frame
    public: virtual Vector3 GetWorldAngularAccel() const;

    public: template< typename T>
            ConnectionPtr ConnectContactCallback( T subscriber )
            { return contactSignal.Connect(subscriber); }
    public: void DisconnectContactCallback( ConnectionPtr &c )
            { contactSignal.Disconnect(c); }

    /// \brief Enable callback: Called when the body changes
    private: void EnabledCB(bool enabled);

    /// \brief Create the bounding box for the geom
    private: void CreateBoundingBox();

    ///  Contact parameters
    public: SurfaceParams *surface; 

    /// The body this geom belongs to
    protected: Body *body;
  
    protected: bool placeable;

    protected: Mass mass;

    private: ParamT<std::string> *typeP;
    private: ParamT<int> *laserFiducialIdP;
    private: ParamT<float> *laserRetroP;

    ///  Mass as a double
    private: ParamT<double> *massP;
    protected: ParamT<Vector3> *xyzP;
    protected: ParamT<Quatern> *rpyP;
    protected: ParamT<bool> *enableContactsP;

    private: float transparency;

    /// All the visual apparence 
    private: std::string bbVisual;

    protected: Shape *shape;

    private: bool contactsEnabled;

    public: EventT< void (const Contact &)> contactSignal;
    private: std::vector<ConnectionPtr> connections;
          
  };

  /// \}

}
#endif
