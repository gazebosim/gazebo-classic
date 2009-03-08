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
#include <map>
#include <string>
#include <vector>

#include "Param.hh"
#include "Pose3d.hh"
#include "Joint.hh"
#include "Entity.hh"
#include "gazebo.h"


namespace gazebo
{

  // Forward declarations
  class XMLConfigNode;
  class Body;
  class Controller;
  class HingeJoint;

  /// \addtogroup gazebo_server
  /// \brief A model
  /// \{
  
  /// \brief A model
  class Model : public Entity
  {
    /// \brief Constructor
    public: Model(Model *parent);
  
    /// \brief Destructor
    public: virtual ~Model();
  
    /// \brief Load the model
    public: int Load(XMLConfigNode *node);
  
    /// \brief Save the model
    public: void Save(std::string &prefix, std::ostream &stream);

    /// \brief Initialize the model
    public: int Init();
  
    /// \brief Update the model
    /// \param params Update parameters
    public: int Update();
  
    /// \brief Finalize the model
    public: int Fini();

    /// \brief Reset the model
    public: void Reset();
  
    /// \brief Initialize the child model
    protected: virtual int InitChild() {return 0;}
  
    /// \brief Update the child model
    protected: virtual int UpdateChild() {return 0;}
  
    /// \brief Finilaize thie child model
    protected: virtual int FiniChild() {return 0;}
    
    /// \brief Get the type of the model
    public: const std::string &GetType() const;

    /// \brief Set the initial pose
    public: void SetInitPose(const Pose3d &pose);
  
    /// \brief Get the initial pose
    public: const Pose3d &GetInitPose() const;
  
    /// \brief Set the current pose
    public: void SetPose(const Pose3d &pose);

    /// \brief Set the position of the model
    public: void SetPosition( const Vector3 &pos );

    /// \brief Set the rotation of the model
    public: void SetRotation( const Quatern &rot );

    /// \brief Set the linear velocity of the model
    public: void SetLinearVel( const Vector3 &vel );

    /// \brief Set the angular velocity of the model
    public: void SetAngularVel( const Vector3 &vel );

    /// \brief Set the linear acceleration of the model
    public: void SetLinearAccel( const Vector3 &vel );

    /// \brief Set the angular acceleration of the model
    public: void SetAngularAccel( const Vector3 &vel );
 
    /// \brief Get the current pose
    public: const Pose3d &GetPose() const;
  
    /// \brief Create and return a new body
    /// \return Pointer to a new body.
    public: Body *CreateBody();
  
    /// \brief Create and return a new joint
    /// \param type Type of the joint.
    /// \return Pointer to a new joint.
    public: Joint *CreateJoint(Joint::Type type);
  
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
    public: const std::map<std::string, Body*> *GetBodies() const;
  
    /// \brief Attach this model to its parent
    public: void Attach(XMLConfigNode *node);
  
    /// \brief Get the canonical body. Used for connected Model heirarchies
    /// \return Pointer to the body
    public: Body *GetCanonicalBody();

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
    private: std::string type;
  
    /// \brief The node this model was loaded from
    private: XMLConfigNode *xmlNode;
  
    /// \brief Initial pose of the model
    private: Pose3d initPose;
  
    /// \brief Current pose
    private: Pose3d pose;
  
    /// \brief Map of the bodies. std::string == body_name
    protected: std::map<std::string, Body* > bodies;
  
    /// \brief Map of the joints
    protected: std::map<std::string, Joint* > joints;
  
    /// \brief Map of the controllers
    protected: std::map<std::string, Controller* > controllers;
  
    /// \brief Joint used to connected models (parent->child).
    private: HingeJoint *joint;
  
    /// \brief Light numbering variable to give a unique name to all light entities
    private: static uint lightNumber;

    private: ParamT<std::string> *canonicalBodyNameP;
    private: ParamT<Vector3> *xyzP;
    private: ParamT<Quatern> *rpyP;
    private: ParamT<std::string> *parentBodyNameP;
    private: ParamT<std::string> *myBodyNameP;


    // Name of a light (if the model is renderable:light)
    private: std::string lightName;

  /*  private: PyObject *pName;
      private: PyObject *pModule;
      private: PyObject *pFuncUpdate;
    */
  };
  /// \}
}

#endif
