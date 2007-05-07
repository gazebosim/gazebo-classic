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
#include <map>
#include <string>
#include <vector>
#include <map>
#include "Pose3d.hh"
#include "Joint.hh"
#include "Entity.hh"
#include "gazebo.h"

// Forward declarations
namespace Ogre
{
  class SceneNode;
}

namespace gazebo
{

class XMLConfigNode;
class Body;

class Model : public Entity
{
  // Constructor
  public: Model();

  // Destructor
  public: virtual ~Model();

  // Load the model
  public: int Load(XMLConfigNode *node);

  // Initialize the model
  public: int Init();

  // Update the model
  public: int Update();

  // Finalize the model
  public: int Fini();

  // Load the child model
  protected: virtual int LoadChild(XMLConfigNode * /*node*/) {return 0;}

  // Initialize the child model
  protected: virtual int InitChild() {return 0;}

  // Update the child model
  protected: virtual int UpdateChild() {return 0;}

  // Finilaize thie child model
  protected: virtual int FiniChild() {return 0;}

  // Set the type of the model
  public: void SetType(const std::string &type);

  // Get the type of the model
  public: const std::string &GetType() const;

  // Set the XMLConfig node this model was loaded from
  public: void SetXMLConfigNode( XMLConfigNode *node );

  // Get the XML Conig node this model was loaded from
  public: XMLConfigNode *GetXMLConfigNode() const;

  // Set the name
  public: void SetName(const std::string &name);

  // Get the name
  public: const std::string &GetName() const;

  // Set the initial pose
  public: void SetInitPose(const Pose3d &pose);

  // Get the initial pose
  public: const Pose3d &GetInitPose() const;

  // Set the current pose
  public: void SetPose(const Pose3d &pose);

  // Get the current pose
  public: const Pose3d &GetPose() const;

  //! Create and return a new body
  /*!
   * \return Pointer to a new body.
   */
  public: Body *CreateBody();

  //! Create and return a new joint
  /*!
   * \param type Type of the joint.
   * \return Pointer to a new joint.
   */
  public: Joint *CreateJoint(Joint::Type type);

  //! Load a body helper function
  /*!
   * \param node XML Configuration node
   * \return Non-zero on error
   */
  private: int LoadBody(XMLConfigNode *node);

  //! Load a joint helper function
  /*!
   * \param node XML Configuration node
   * \return Non-zero on error
   */
  private: int LoadJoint(XMLConfigNode *node);

  //! Load a interface helper function
  /*!
   * \param node XML Configuration node
   * \return Non-zero on error
   */
  private: int LoadIface(XMLConfigNode *node);

  // Type of the model (such as Pioneer2DX, or SimpleSolid)
  private: std::string type;

  // Name for this model
  private: std::string name;

  // The node this model was loaded from
  private: XMLConfigNode *node;

  // Initial pose of the model
  private: Pose3d initPose;

  // Current pose
  private: Pose3d pose;

  //! Map of the bodies. std::string == body_name
  protected: std::map<std::string, Body*> bodies;
  protected: std::vector<Joint*> joints;

  /*private: PyObject *pName;
  private: PyObject *pModule;
  private: PyObject *pFuncUpdate;
  */
};

}

#endif
