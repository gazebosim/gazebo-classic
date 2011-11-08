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
/* Desc: A screw or primastic joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ScrewJoint.hh 7039 2008-09-24 18:06:29Z natepak $
 */

#ifndef ODESCREWJOINT_HH
#define ODESCREWJOINT_HH

#include "physics/ScrewJoint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{


/// \addtogroup gazebo_screw_joint Screw Joint
/// \{

  namespace physics
  {
    /// \brief A screw joint
    class ODEScrewJoint : public ScrewJoint<ODEJoint>
    {
      /// \brief Constructor
      public: ODEScrewJoint( dWorldID worldId );
    
      /// \brief Destructor
      public: virtual ~ODEScrewJoint();
    
      /// \brief Load the joint
      protected: virtual void Load( sdf::ElementPtr &_sdf );

      /// \brief Get the axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int index) const;

      /// \brief Set the axis of motion
      public: virtual void SetAxis( int index, const math::Vector3 &axis );
    
      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int index, const double damping);

      /// \brief Set screw joint thread pitch
      public: virtual void SetThreadPitch(int index, const double _thread_pitch);

      /// \brief callback to apply damping force to joint
      public: void ApplyDamping();

      /// \brief Get the position of the joint
      public: virtual math::Angle GetAngle(int index) const;

      /// \brief Get the rate of change
      public: virtual double GetVelocity(int index) const;

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle);

      /// \brief Set the screw force
      public: virtual void SetForce(int index, double force);

      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t);

      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index);

      /// \brief Get the _parameter
      public: virtual double GetParam( int parameter ) const;

      /// \brief Set the _parameter
      public: virtual void SetParam( int parameter, double value);
    };
  
/// \}
  }
}
#endif
