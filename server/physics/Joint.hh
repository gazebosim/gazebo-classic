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

#include "Common.hh"
#include "Param.hh"
#include "Vector3.hh"

namespace gazebo
{
  /// \addtogroup gazebo_physics
  /// \brief Base class for all joints
  /// \{
  
  class Body;
  class XMLConfigNode;
  class Model;
  class OgreDynamicLines;
  class OgreVisual; 

  /// \brief Base class for all joints
  class Joint : public Common
  {
    /// \brief Type of joint
    public: enum Type {SLIDER, HINGE, HINGE2, BALL, UNIVERSAL};

    /// \brief Constructor
    public: Joint();

    /// \brief Destructor
    public: virtual ~Joint();

    /// \brief Load a joint
    public: void Load(XMLConfigNode *node);

    /// \brief Save a joint to a stream in XML format
    public: void Save(std::string &prefix, std::ostream &stream);

    /// \brief Save a joint to a stream in XML format
    protected: virtual void SaveChild(std::string &prefix, std::ostream &stream) {}

    /// \brief Load child joint
    protected: virtual void LoadChild(XMLConfigNode *node) {}

    /// \brief Update the joint
    public: void Update();

    /// \brief Reset the joint
    public: void Reset();

    /// \brief Reset the child joint
    protected: virtual void ResetChild() {};

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

    /// \brief Get the anchor point
    public: virtual gazebo::Vector3 GetAnchor() const
            {return gazebo::Vector3(0,0,0);}


    /// \brief Set the _parameter to _value
    public: virtual void SetParam( int parameter, double value );

    /// \brief Get the name of this joint
    public: std::string GetName() const;

    /// \brief Set the name of this joint
    public: void SetName(const std::string &name);

     /// \brief Set the ERP of this joint
    public: void SetERP(double newERP);

    /// \brief Get the ERP of this joint
    public: double GetERP();

     /// \brief Set the CFM  of this joint
    public: void SetCFM(double newERP);

    /// \brief Get the CFM of this joint
    public: double GetCFM();

    /// \brief Get the feedback data structure for this joint, if set
    public: dJointFeedback *GetFeedback();

    /// \brief Get the high stop of an axis(index).
    public: double GetHighStop(int index=0);

    /// \brief Get the low stop of an axis(index).
    public: double GetLowStop(int index=0);
   
    /// This is our id
    protected: dJointID jointId;

    /// Type of Joint
    protected: Type type;

    /// The first body this joint connects to
    private: Body *body1;

    /// The second body this joint connects to
    private: Body *body2;

    /// Name of this joint
    private: ParamT<double> *erpP;
    private: ParamT<double> *cfmP;

    // joint limit Kp setting
    private: ParamT<double> *stopKpP;

    // joint limit Kd setting
    private: ParamT<double> *stopKdP;

    private: ParamT<std::string> *body1NameP;
    private: ParamT<std::string> *body2NameP;
    private: ParamT<std::string> *anchorBodyNameP;
    private: ParamT<Vector3> *anchorOffsetP;
    private: ParamT<bool> *provideFeedbackP;
    private: ParamT<double> *fudgeFactorP;

    /// Feedback data for this joint
    private: dJointFeedback *feedback;

    private: OgreVisual *visual;

    private: Model *model;

    private: OgreDynamicLines *line1;
    private: OgreDynamicLines *line2;
  };

  /// \}
}
#endif

