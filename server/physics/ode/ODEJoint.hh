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
/* Desc: The ODE base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 12 Oct 2009
 * SVN: $Id$
 */

#ifndef ODEJOINT_HH
#define ODEJOINT_HH

#include <ode/ode.h>
#include "Joint.hh"

namespace gazebo
{
  /// \brief ODE joint interface
  class ODEJoint : public Joint
  {
    /// \brief Constructor
    public: ODEJoint();
  
    /// \brief Destructor
    public: virtual ~ODEJoint();

    /// \brief Load a joint
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Get the body to which the joint is attached according the _index
    public: virtual Body *GetJointBody( int index ) const;

    /// \brief Determines of the two bodies are connected by a joint
    public: bool virtual AreConnected( Body *one, Body *two ) const;

    /// \brief The default function does nothing. This should be overriden in 
    ///        the child classes where appropriate
    public: virtual double GetParam( int /*parameter*/ ) const;

    /// \brief Attach the two bodies with this joint
    public: virtual void Attach( Body *one, Body *two );

    /// \brief Detach this joint from all bodies
    public: virtual void Detach();

    /// \brief By default this does nothing. It should be overridden in child 
    ///        classes where appropriate
    public: virtual void SetParam(int /*parameter*/, double /*value*/);

    /// \brief Set the ERP of this joint
    public: void SetERP(double newERP);

    /// \brief Get the ERP of this joint
    public: double GetERP();

    /// \brief Set the CFM of this joint
    public: void SetCFM(double newCFM);

    /// \brief Get the ERP of this joint
    public: double GetCFM();

    /// \brief Get the feedback data structure for this joint, if set
    public: dJointFeedback *GetFeedback();

    /// \brief Set the high stop of an axis(index).
    public: virtual void SetHighStop(int index, Angle angle);

    /// \brief Set the low stop of an axis(index).
    public: virtual void SetLowStop(int index, Angle angle);
 
    /// \brief Get the high stop of an axis(index).
    public: virtual Angle GetHighStop(int index);

    /// \brief Get the low stop of an axis(index).
    public: virtual Angle GetLowStop(int index);

    /// \brief Get the force the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyForce(unsigned int index) const;

    /// \brief Get the torque the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyTorque(unsigned int index) const;

    /// \brief Set a parameter for the joint
    public: virtual void SetAttribute( Attribute, int index, double value);
 
    /// This is our id
    protected: dJointID jointId;
  
    /// Feedback data for this joint
    private: dJointFeedback *feedback;
  
  };
}
#endif
