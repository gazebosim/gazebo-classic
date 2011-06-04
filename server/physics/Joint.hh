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

#include "Common.hh"
#include "Param.hh"
#include "Vector3.hh"
#include <boost/signal.hpp>
#include "gazebo_config.h"

namespace gazebo
{
  /// \addtogroup gazebo_physics
  /// \brief Base class for all joints
  /// \{

  class PhysicsEngine;  
  class Body;
  class XMLConfigNode;
  class Model;
  class OgreDynamicLines;
  class OgreVisual; 

  /// \brief Base class for all joints
  class Joint : public Common
  {
    /// \brief Type of joint
#ifdef ODE_SCREW_JOINT
    public: enum Type {SCREW, SLIDER, HINGE, HINGE2, BALL, UNIVERSAL, FIXED, TYPE_COUNT};
#else
    public: enum Type {SLIDER, HINGE, HINGE2, BALL, UNIVERSAL, FIXED, TYPE_COUNT};
#endif
    public: enum Attribute {FUDGE_FACTOR, SUSPENSION_ERP, SUSPENSION_CFM, STOP_ERP,STOP_CFM,ERP,CFM,FMAX,VEL,HI_STOP,LO_STOP};

    /// \brief Type names of joint
    public: static std::string TypeNames[TYPE_COUNT]; 

    /// \brief Constructor
    public: Joint();

    /// \brief Destructor
    public: virtual ~Joint();

    /// \brief Load a joint
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save a joint to a stream in XML format
    public: void Save(std::string &prefix, std::ostream &stream);
    protected: virtual void SaveJoint(std::string &prefix, std::ostream &stream) {}

    /// \brief Update the joint
    public: virtual void Update();

    /// \brief Set the joint to show visuals
    public: void ShowJoints(bool s);

    /// \brief Reset the joint
    public: virtual void Reset();

    /// \brief Set the model this joint belongs too
    public: void SetModel(Model *model);

    /// \brief Get the type of the joint
    public: Joint::Type GetType() const;

    /// \brief Get the body to which the joint is attached according the _index
    public: virtual Body *GetJointBody( int index ) const = 0;

    /// \brief Determines of the two bodies are connected by a joint
    public: virtual bool AreConnected( Body *one, Body *two ) const = 0;

    /// \brief Attach the two bodies with this joint
    public: virtual void Attach( Body *one, Body *two );

    /// \brief Detach this joint from all bodies
    public: virtual void Detach() = 0;

    /// \brief Set the axis of rotation
    public: virtual void SetAxis(int index, const Vector3 &axis) = 0;

    /// \brief Set the joint damping
    public: virtual void SetDamping(int index, const double damping) = 0;

    /// \brief Connect a boost::slot the the joint update signal
    public: template<typename T>
            boost::signals::connection ConnectJointUpdateSignal(T subscriber)
            { return jointUpdateSignal.connect(subscriber); }
    /// \brief Disconnect a boost::slot the the joint update signal
    public: template<typename T>
            void DisconnectJointUpdateSignal( T subscriber )
            { jointUpdateSignal.disconnect(subscriber); }

    /// \brief Get the axis of rotation
    public: virtual Vector3 GetAxis(int index) const = 0;

    /// \brief Set the anchor point
    public: virtual void SetAnchor( int index, const Vector3 &anchor ) = 0;

    /// \brief Get the anchor point
    public: virtual Vector3 GetAnchor(int index) const = 0;

    /// \brief Set the high stop of an axis(index).
    public: virtual void SetHighStop(int index, Angle angle) = 0;

    /// \brief Set the low stop of an axis(index).
    public: virtual void SetLowStop(int index, Angle angle) = 0;
 
    /// \brief Get the high stop of an axis(index).
    public: virtual Angle GetHighStop(int index) = 0;

    /// \brief Get the low stop of an axis(index).
    public: virtual Angle GetLowStop(int index) = 0;

    /// \brief Set the velocity of an axis(index).
    public: virtual void SetVelocity(int index, double v) = 0;

    /// \brief Get the rotation rate of an axis(index)
    public: virtual double GetVelocity(int index) const = 0;

    /// \brief Set the force applied to an axis
    public: virtual void SetForce(int index, double f) {}

    /// \brief Get the force applied to an axis
    public: virtual double GetForce(int index) {return 0;}
 
    /// \brief Set the max allowed force of an axis(index).
    public: virtual void SetMaxForce(int index, double t) = 0;

    /// \brief Get the max allowed force of an axis(index).
    public: virtual double GetMaxForce(int index) = 0;

    /// \brief Get the angle of rotation of an axis(index)
    public: virtual Angle GetAngle(int index) const = 0;

    /// \brief Get the force the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyForce(unsigned int index) const = 0;

    /// \brief Get the torque the joint applies to the first body
    /// \param index The index of the body( 0 or 1 )
    public: virtual Vector3 GetBodyTorque(unsigned int index) const = 0;

    /// \brief Set a parameter for the joint
    public: virtual void SetAttribute( Attribute, int index, double value) = 0;
  
    /// Type of Joint
    protected: Type type;

    /// The first body this joint connects to
    public: Body *body1;

    /// The second body this joint connects to
    public: Body *body2;

    /// Name of this joint
    protected: ParamT<double> *erpP;
    protected: ParamT<double> *cfmP;

    // joint limit Kp setting
    protected: ParamT<double> *stopKpP;

    // joint limit Kd setting
    protected: ParamT<double> *stopKdP;

    protected: ParamT<std::string> *body1NameP;
    protected: ParamT<std::string> *body2NameP;
    public: ParamT<std::string> *anchorBodyNameP;
    protected: ParamT<Vector3> *anchorOffsetP;
    protected: ParamT<bool> *provideFeedbackP;
    protected: ParamT<double> *fudgeFactorP;

    protected: OgreVisual *visual;

    protected: Model *model;

    protected: OgreDynamicLines *line1;
    protected: OgreDynamicLines *line2;
    protected: PhysicsEngine *physics;
    public: Vector3 anchorPos;
    public: Body *anchorBody;

    protected: boost::signal<void ()> jointUpdateSignal;

    // joint damping_coefficient
    protected: double damping_coefficient;
  };

  /// \}
}
#endif

