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
/* Desc: A fixed joint. Does not get simulated in the physics engine
 * Author: Nate Koenig
 */

#ifndef FIXEDJOINT_HH
#define FIXEDJOINT_HH

#include "Angle.hh"
#include "Vector3.hh"
#include "Param.hh"
#include "Joint.hh"
#include "XMLConfig.hh"
#include "World.hh"
#include "Global.hh"

namespace gazebo
{

  /// \addtogroup gazebo_physics_joints
  /// \{
  /** \defgroup gazebo_hinge_joint Hinge Fixed
    
    \brief A fixed hinge joint.
  
    \par Attributes
    - body1 (string)
      - Name of the first body to attach to the joint
    - body2 (string)
      - Name of the second body to attach to the joint
    - anchor (string)
      - Name of the body which will act as the anchor to the joint
    - offset (tuple)
      - XYZ offset from the anchor body
 
    \par Example
    \verbatim
    <joint:hinge name="hinge_joint>
      <body1>body1_name</body1>
      <body2>body2_name</body2>
      <anchor>anchor_body</anchor>
    </joint:hinge>
    \endverbatim
  */
  /// \}
  
  /// \addtogroup gazebo_fixed_joint
  /// \{
  
  ///\brief A fixed joint
  class FixedJoint : public Joint
  {
    /// \brief Constructor
    public: FixedJoint();
 
    ///  \brief Destructor
    public: virtual ~FixedJoint();

    /// \brief Load joint
    protected: virtual void Load(XMLConfigNode *node);
               
    /// \brief Update the joint
    public: virtual void Update();

    /// \brief Save a joint to a stream in XML format
    protected: virtual void SaveJoint(std::string &prefix,std::ostream &stream);

    /// \brief Get the body to which the joint is attached according the _index
    public: virtual Body *GetJointBody( int index ) const;

    /// \brief Determines of the two bodies are connected by a joint
    public: virtual bool AreConnected( Body *one, Body *two ) const  
            {return false;}

    /// \brief Detach this joint from all bodies
    public: virtual void Detach() {}

    /// \brief Set the axis of rotation
    public: virtual void SetAxis(int , const Vector3 &) {}

    /// \brief Set the joint damping
    public: virtual void SetDamping(int , const double) {}

    /// \brief Set the anchor point
    public: virtual void SetAnchor( int index, const Vector3 &anchor ) {}

    /// \brief Get the anchor point
    public: virtual Vector3 GetAnchor(int index) const { return Vector3(); }

    /// \brief Set the high stop of an axis(index).
    public: virtual void SetHighStop(int index, Angle angle) {}

    /// \brief Set the low stop of an axis(index).
    public: virtual void SetLowStop(int index, Angle angle) {}
 
    /// \brief Get the high stop of an axis(index).
    public: virtual Angle GetHighStop(int index) {return Angle();}

    /// \brief Get the low stop of an axis(index).
    public: virtual Angle GetLowStop(int index) {return Angle();}

    /// \brief Set the velocity of an axis(index).
    public: virtual void SetVelocity(int index, double v) {}

    /// \brief Get the rotation rate of an axis(index)
    public: virtual double GetVelocity(int index) const {return 0;}

    /// \brief Set the max allowed force of an axis(index).
    public: virtual void SetMaxForce(int index, double t) {}

    /// \brief Get the max allowed force of an axis(index).
    public: virtual double GetMaxForce(int index) {return 0;}

    /// \brief Get the angle of rotation of an axis(index)
    public: virtual Angle GetAngle(int index) const {return Angle();}

    /// \brief Get the force the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyForce(unsigned int index) const {return Vector3();}

    /// \brief Get the torque the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyTorque(unsigned int index) const {return Vector3();}

    /// \brief Set a parameter for the joint
    public: virtual void SetAttribute( Attribute, int index, double value) {}

    /// \brief Get the axis of rotation
    public: virtual Vector3 GetAxis(int index) const {return Vector3();}

    private: Pose3d subsumedOffset;
  };
  /// \}
}
#endif

