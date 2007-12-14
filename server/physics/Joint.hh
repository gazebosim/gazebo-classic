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
/* Desc: The base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#ifndef JOINT_HH
#define JOINT_HH

#include <ode/ode.h>

#include "Vector3.hh"

namespace Ogre
{
  class SceneNode;
}

namespace gazebo
{
  /// \addtogroup gazebo_physics
  /// \brief Base class for all joints
  /// \{
  
  class Body;
  class XMLConfigNode;
  class Model;
  class OgreDynamicLines;

  /// \brief Base class for all joints
  class Joint
  {
    /// \brief Type of joint
    public: enum Type {SLIDER, HINGE, HINGE2, BALL, UNIVERSAL};

    /// \brief Constructor
    public: Joint();

    /// \brief Destructor
    public: virtual ~Joint();

    /// \brief Load a joint
    public: void Load(XMLConfigNode *node);

    /// \brief Load child joint
    protected: virtual void LoadChild(XMLConfigNode *node) {};

    /// \brief Update the joint
    public: void Update();

    /// \brief Set the model this joint belongs too
    public: void SetModel(Model *model);

    /// \brief Get the type of the joint
    public: Joint::Type GetType() const;

    /// \brief Get the body to which the joint is attached according the _index
    public: Body *GetJointBody( int index ) const;

    /// \brief Determines of the two bodies are connected by a joint
    public: bool AreConnected( Body *one, Body *two ) const;

    /// \brief Get the _parameter
    public: virtual double GetParam(int parameter) const;

    /// \brief Make this joint a fixed joint
    /// Use this only when absolutely necessary
    public: void SetFixed();

    /// \brief Attach the two bodies with this joint
    public: void Attach( Body *one, Body *two );

    /// \brief Detach this joint from all bodies
    public: void Detach();

    /// \brief Set the anchor point
    public: virtual void SetAnchor( const gazebo::Vector3 & /*anchor*/ ) {}

    /// \brief Set the anchor point
    public: virtual gazebo::Vector3 GetAnchor() const
            {return gazebo::Vector3(0,0,0);}


    /// \brief Set the _parameter to _value
    public: virtual void SetParam( int parameter, double value );

    /// \brief Set the name of this joint
    public: std::string GetName() const;

    /// \brief Get the name of this joint
    public: void SetName(const std::string &name);

    /// \brief This is our id
    protected: dJointID jointId;

    /// \brief Type of Joint
    protected: Type type;

    /// \brief The first body this joint connects to
    private: Body *body1;

    /// \brief The second body this joint connects to
    private: Body *body2;

    /// \brief Name of this joint
    private: std::string name;

    public: Ogre::SceneNode *sceneNode;

    private: Model *model;

    private: OgreDynamicLines *line1;
    private: OgreDynamicLines *line2;
  };

  /// \}
}
#endif

