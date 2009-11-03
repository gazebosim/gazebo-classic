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
/* Desc: The base Bullet joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * SVN: $Id$
 */

#ifndef BULLETJOINT_HH
#define BULLETJOINT_HH

#include <btBulletDynamicsCommon.h>

#include "GazeboMessage.hh"
#include "Joint.hh"
#include "Param.hh"
#include "Vector3.hh"

namespace gazebo
{
  /// \addtogroup gazebo_physics
  /// \brief Base class for all joints
  /// \{
  
  /// \brief Base class for all joints
  class BulletJoint : public Joint
  {
    /// \brief Constructor
    public: BulletJoint();

    /// \brief Destructor
    public: virtual ~BulletJoint();

    /// \brief Load a joint
    public: void Load(XMLConfigNode *node);

    /// \brief Update the joint
    public: void Update();

    /// \brief Reset the joint
    public: void Reset() {}

    /// \brief Get the body to which the joint is attached according the _index
    public: Body *GetJointBody( int index ) const;

    /// \brief Determines of the two bodies are connected by a joint
    public: bool AreConnected( Body *one, Body *two ) const;

    /// \brief Detach this joint from all bodies
    public: virtual void Detach();

    /// \brief Set the anchor point
    public: virtual void SetAnchor( int /*index*/, 
                                    const gazebo::Vector3 & /*anchor*/ ) 
            {gzerr(0) << "Not implement in Bullet\n";}

    /// \brief Get the anchor point
    public: virtual gazebo::Vector3 GetAnchor(int index) const
            {gzerr(0) << "Not implement in Bullet\n"; return Vector3();}

    /// \brief Get the force the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyForce(unsigned int index) const
            {gzerr(0) << "Not implement in Bullet\n"; return Vector3();}

    /// \brief Get the torque the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyTorque(unsigned int index) const
            {gzerr(0) << "Not implement in Bullet\n"; return Vector3();}

    /// \brief Set a parameter for the joint
    public: virtual void SetAttribute( Attribute, int index, double value)
            {gzerr(0) << "Not implement in Bullet\n";}
 
    /// \brief Set the ERP of this joint
    public: void SetERP(double newERP);

    /// \brief Get the ERP of this joint
    public: double GetERP();

     /// \brief Set the CFM  of this joint
    public: void SetCFM(double newERP);

    /// \brief Get the CFM of this joint
    public: double GetCFM();

    protected: btTypedConstraint *constraint;

    protected: btDynamicsWorld *world;
  };

  /// \}
}
#endif

