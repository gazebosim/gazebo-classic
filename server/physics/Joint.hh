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
 * CVS: $Id: Joint.hh,v 1.1.2.1 2006/12/16 22:41:16 natepak Exp $
 */

#ifndef JOINT_HH
#define JOINT_HH

#include <ode/ode.h>
#include "Vector3.hh"


namespace gazebo
{
  class Body;

class Joint
{
  public: enum Type {SLIDER, HINGE, HINGE2, BALL, UNIVERSAL};

  // Constructor
  protected: Joint();

  // Destructor
  protected: virtual ~Joint();

  // Get the type of the joint
  public: Joint::Type GetType() const;

  // Get the body to which the joint is attached according the _index
  public: Body *GetJointBody( int index ) const;

  // Determines of the two bodies are connected by a joint
  public: bool AreConnected( Body *one, Body *two ) const;

  // Get the _parameter
  public: virtual double GetParam(int parameter) const;

  // Make this joint a fixed joint
  // Use this only when absolutely necessary
  public: void SetFixed();

  // Attach the two bodies with this joint
  public: void Attach( Body *one, Body *two );

  // Detach this joint from all bodies
  public: void Detach();

  // Set the anchor point
  public: virtual void SetAnchor( const gazebo::Vector3 & /*anchor*/ ) {}

  // Set the anchor point
  public: virtual gazebo::Vector3 GetAnchor() const
          {return gazebo::Vector3(0,0,0);}


  // Set the _parameter to _value
  public: virtual void SetParam( int parameter, double value );

  /// Set the name of this joint
  public: std::string GetName() const;

  /// Get the name of this joint
  public: void SetName(const std::string &name);

  // This is our id
  protected: dJointID jointId;

  // Type of Joint
  protected: Type type;

  // The first body this joint connects to
  private: Body* body1;

  // The second body this joint connects to
  private: Body* body2;

  /// Name of this joint
  private: std::string name;
};

}
#endif

