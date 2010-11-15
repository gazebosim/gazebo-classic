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
/* Desc: Base class for all models
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 * SVN: $Id$
 */

#ifndef MODEL_HH
#define MODEL_HH

//#include <python2.4/Python.h>
#include <boost/any.hpp>
#include <boost/signal.hpp>
#include <map>
#include <string>
#include <vector>

#include "Param.hh"
#include "Contact.hh"
#include "Pose3d.hh"
#include "Joint.hh"
#include "Entity.hh"
#include "gz.h"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{

  // Forward declarations
  class XMLConfigNode;
  class Body;
  class Controller;
  class GraphicsIfaceHandler;
  class Sensor;
  class Geom;
  class Light;

  /// \addtogroup gazebo_server
  /// \brief A model
  /// \{
  
  /// \brief A model
  class Model : public Entity
  {
    public: typedef std::vector<Joint*> JointContainer;

    /// \brief Constructor
    public: Model(Model *parent);
  
    /// \brief Destructor
    public: virtual ~Model();
  
    /// \brief Load the model
    /// \param removeDuplicate Remove existing model of same name
    public: void Load(XMLConfigNode *node, bool removeDuplicate);
  
    /// \brief Save the model
    public: void Save(std::string &prefix, std::ostream &stream);

    /// \brief Initialize the model
    public: virtual void Init();
  
    /// \brief Update the model
    /// \param params Update parameters
    public: void Update();

    /// \brief Remove a child
    public: virtual void RemoveChild(Entity *child);

    /// \brief Primarily used to update the graphics interfaces
    public: void GraphicsUpdate();

    /// \brief Finalize the model
    public: void Fini();

    /// \brief Reset the model
    public: void Reset();
  
    /// \brief Initialize the child model
    protected: virtual void InitChild() {}
  
    /// \brief Update the child model
    protected: virtual void UpdateChild() {}
  
    /// \brief Finilaize thie child model
    protected: virtual void FiniChild() {}
    
    /// \brief Get the type of the model
    public: const std::string &GetModelType() const;

    /// \brief Set the initial pose
    public: void SetInitPose(const Pose3d &pose);
  
    /// \brief Get the initial pose
    public: const Pose3d &GetInitPose() const;
  
    /// \brief Set the linear velocity of the model
    public: void SetLinearVel( const Vector3 &vel );

    /// \brief Set the angular velocity of the model
    public: void SetAngularVel( const Vector3 &vel );

    /// \brief Set the linear acceleration of the model
    public: void SetLinearAccel( const Vector3 &vel );

    /// \brief Set the angular acceleration of the model
    public: void SetAngularAccel( const Vector3 &vel );

    /// \brief Get the linear velocity of the entity
    public: virtual Vector3 GetRelativeLinearVel() const;

    /// \brief Get the linear velocity of the entity in the world frame
    public: virtual Vector3 GetWorldLinearVel() const;

    /// \brief Get the angular velocity of the entity
    public: virtual Vector3 GetRelativeAngularVel() const;

    /// \brief Get the angular velocity of the entity in the world frame
    public: virtual Vector3 GetWorldAngularVel() const;

    /// \brief Get the linear acceleration of the entity
    public: virtual Vector3 GetRelativeLinearAccel() const;

    /// \brief Get the linear acceleration of the entity in the world frame
    public: virtual Vector3 GetWorldLinearAccel() const;

    /// \brief Get the angular acceleration of the entity 
    public: virtual Vector3 GetRelativeAngularAccel() const;

    /// \brief Get the angular acceleration of the entity in the world frame
    public: virtual Vector3 GetWorldAngularAccel() const;


    /// \brief Get the size of the bounding box
    public: void GetBoundingBox(Vector3 &min, Vector3 &max) const;
  
    /// \brief Create and return a new body
    /// \return Pointer to a new body.
    public: Body *CreateBody();

    /// \brief Get the number of joints
    /// \return Get the number of joints
    public: unsigned int GetJointCount() const;

    /// \brief Get a joing by index
    /// \param index Index of the joint
    /// \return A pointer to the joint
    public: Joint *GetJoint( unsigned int index ) const;

    /// \brief Get a joint
    /// \param name The name of the joint, specified in the world file
    /// \return Pointer to the joint
    public: Joint *GetJoint(std::string name);
  
    /// \brief Get the default body
    /// \return The default body
    public: Body *GetBody();

    /// \brief Get a body by name
    /// \return Pointer to the body
    public: Body *GetBody(const std::string &name);

    /// \brief Get a map of all the bodies
    //public: const std::map<std::string, Body*> *GetBodies() const;

    /// \brief Get a sensor by name
    public: Sensor *GetSensor(const std::string &name) const;

    /// \brief Get a geom by name
    public: Geom *GetGeom(const std::string &name) const;

    /// \brief Detach from parent model
    public: void Detach();

    /// \brief Attach this model to its parent
    public: void Attach(XMLConfigNode *node);
  
    /// \brief Get the canonical body. Used for connected Model heirarchies
    /// \return Pointer to the body
    public: Body *GetCanonicalBody() const;

    /// \brief Called when the pose of the entity (or one of its parents) has
    /// changed
    public: virtual void OnPoseChange();

    /// \brief Set the gravity mode of the model
    public: void SetGravityMode( const bool &v );

    /// \brief Set the friction mode of the model
    public: void SetFrictionMode( const bool &v );

    /// \brief Set the collide mode of the model
    public: void SetCollideMode( const std::string &m );

    /// \brief Set the laser fiducial integer Id of the model
    public: void SetLaserFiducialId( const int &id );

    /// \brief Get the laser fiducial integer Id of the model
    public: int GetLaserFiducialId();

    /// \brief Set the laser retro reflectiveness of the model
    public: void SetLaserRetro( const float &retro );

    /// \brief Get the list of interfaces e.g "pioneer2dx_model1::laser::laser_iface0->laser"
    public: void GetModelInterfaceNames(std::vector<std::string>& list) const;

    /// \brief Add an occurance of a contact to this geom
    public: void StoreContact(const Geom *geom, const Contact &contact);

    /// \brief Get the number of contacts for a geom
    public: unsigned int GetContactCount(const Geom *geom) const;

    /// \brief Retreive a contact
    public: Contact RetrieveContact(const Geom *geom, unsigned int i) const;

    /// \brief Load a body helper function
    /// \param node XML Configuration node
    private: void LoadBody(XMLConfigNode *node);
  
    /// \brief Load a joint helper function
    /// \param node XML Configuration node
    private: void LoadJoint(XMLConfigNode *node);
  
    /// \brief Load a controller helper function
    /// \param node XML Configuration node
    private: void LoadController(XMLConfigNode *node);
  
    /// \brief Load a physical model
    private: void LoadPhysical(XMLConfigNode *node);
  
    /// \brief Load a renderable model (like a light source).
    private: void LoadRenderable(XMLConfigNode *node);
  
    /// \brief Type of the model (such as Pioneer2DX, or SimpleSolid)
    private: std::string modelType;
  
    /// \brief Initial pose of the model
    private: Pose3d initPose;
 
    /// \brief Map of the joints
    //protected: std::map<std::string, Joint* > joints;
    protected: std::vector<Joint* > joints;
  
    /// \brief Map of the controllers
    protected: std::map<std::string, Controller* > controllers;
  
    /// \brief Joint used to connected models (parent->child).
    private: Joint *joint;
  
    /// \brief Light numbering variable to give a unique name to all light entities
    private: static uint lightNumber;

    public: Pose3d GetWorldPose();

    private: ParamT<std::string> *canonicalBodyNameP;
    private: ParamT<Vector3> *xyzP;
    private: ParamT<Quatern> *rpyP;
    private: ParamT<std::string> *parentBodyNameP;
    private: ParamT<std::string> *myBodyNameP;
    private: ParamT<bool> *enableGravityP;
    private: ParamT<bool> *enableFrictionP;
    private: ParamT<int> *laserFiducialP;
    private: ParamT<float> *laserRetroP;
    private: ParamT<std::string> *collideP;

    private: Body *canonicalBody;
    private: std::vector<Body*> bodies;

    /// All the contacts for every geom
    public: std::map< std::string, std::vector<Contact> > contacts;

    // Name of a light (if the model is renderable:light)
    private: Light *light;

    private: GraphicsIfaceHandler *graphicsHandler;

  /*  private: PyObject *pName;
      private: PyObject *pModule;
      private: PyObject *pFuncUpdate;
    */

  };
  /// \}
}

#endif
