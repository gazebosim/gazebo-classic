/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: The base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef JOINT_HH
#define JOINT_HH

#include "common/Event.hh"
#include "common/Angle.hh"
#include "common/Vector3.hh"
#include "common/Global.hh"
#include "transport/TransportTypes.hh"

#include "physics/Base.hh"

namespace gazebo
{
	namespace physics
  {
    /// \brief Base class for all joints
    class Joint : public Base
    {
      /// \brief Type of joint
      public: enum Attribute {FUDGE_FACTOR, SUSPENSION_ERP, SUSPENSION_CFM, STOP_ERP,STOP_CFM,ERP,CFM,FMAX,VEL,HI_STOP,LO_STOP};
  
      /// \brief Constructor
      public: Joint();
  
      /// \brief Destructor
      public: virtual ~Joint();
  
      /// \brief Load a joint
      public: virtual void Load(common::XMLConfigNode *node);
  
      /// \brief Save a joint to a stream in XML format
      public: void Save(std::string &prefix, std::ostream &stream);

      protected: virtual void SaveJoint(std::string &prefix, std::ostream &stream) {}
  
      /// \brief Update the joint
      public: void Update();
  
      /// \brief Set the joint to show visuals
      public: void ShowJoints(const bool &s);
  
      /// \brief Reset the joint
      public: virtual void Reset();
  
      /// \brief Set the model this joint belongs too
      public: void SetModel(ModelPtr model);
  
      /// \brief Get the body to which the joint is attached according the _index
      public: virtual BodyPtr GetJointBody( int index ) const = 0;
  
      /// \brief Determines of the two bodies are connected by a joint
      public: virtual bool AreConnected( BodyPtr one, BodyPtr two ) const = 0;
  
      /// \brief Attach the two bodies with this joint
      public: virtual void Attach( BodyPtr one, BodyPtr two );
  
      /// \brief Detach this joint from all bodies
      public: virtual void Detach() = 0;
  
      /// \brief Set the axis of rotation
      public: virtual void SetAxis(int index, const common::Vector3 &axis) = 0;
  
      /// \brief Set the joint damping
      public: virtual void SetDamping(int index, const double damping) = 0;
  
      /// \brief Connect a boost::slot the the joint update signal
      public: template<typename T>
              event::ConnectionPtr ConnectJointUpdateSignal(T subscriber)
              { return jointUpdateSignal.Connect(subscriber); }
      /// \brief Disconnect a boost::slot the the joint update signal
      public: void DisconnectJointUpdateSignal( event::ConnectionPtr &c )
              { jointUpdateSignal.Disconnect(c); }
  
      /// \brief Get the axis of rotation
      public: virtual common::Vector3 GetAxis(int index) const = 0;
  
      /// \brief Set the anchor point
      public: virtual void SetAnchor( int index, const common::Vector3 &anchor ) = 0;
  
      /// \brief Get the anchor point
      public: virtual common::Vector3 GetAnchor(int index) const = 0;
  
      /// \brief Set the high stop of an axis(index).
      public: virtual void SetHighStop(int index, common::Angle angle) = 0;
  
      /// \brief Set the low stop of an axis(index).
      public: virtual void SetLowStop(int index, common::Angle angle) = 0;
   
      /// \brief Get the high stop of an axis(index).
      public: virtual common::Angle GetHighStop(int index) = 0;
  
      /// \brief Get the low stop of an axis(index).
      public: virtual common::Angle GetLowStop(int index) = 0;
  
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
      public: virtual common::Angle GetAngle(int index) const = 0;
  
      /// \brief Get the force the joint applies to the first body
      /// \param index The index of the body( 0 or 1 )
      public: virtual common::Vector3 GetBodyForce(unsigned int index) const = 0;
  
      /// \brief Get the torque the joint applies to the first body
      /// \param index The index of the body( 0 or 1 )
      public: virtual common::Vector3 GetBodyTorque(unsigned int index) const = 0;
  
      /// \brief Set a parameter for the joint
      public: virtual void SetAttribute( Attribute, int index, double value) = 0;
    
      /// The first body this joint connects to
      protected: BodyPtr body1;
  
      /// The second body this joint connects to
      protected: BodyPtr body2;
  
      /// Name of this joint
      protected: common::ParamT<double> *erpP;
      protected: common::ParamT<double> *cfmP;
  
      // joint limit Kp setting
      protected: common::ParamT<double> *stopKpP;
  
      // joint limit Kd setting
      protected: common::ParamT<double> *stopKdP;
  
      protected: common::ParamT<std::string> *body1NameP;
      protected: common::ParamT<std::string> *body2NameP;
      protected: common::ParamT<common::Vector3> *anchorOffsetP;
      protected: common::ParamT<bool> *provideFeedbackP;
      protected: common::ParamT<double> *fudgeFactorP;
  
      protected: std::string visual;
      protected: std::string line1;
      protected: std::string line2;
      protected: bool showJoints;
  
      protected: ModelPtr model;
  
      protected: common::Vector3 anchorPos;
      protected: BodyPtr anchorBody;
  
      private: event::EventT<void ()> jointUpdateSignal;
      private: event::ConnectionPtr showJointsConnection;
  
      // joint damping_coefficient
      protected: double damping_coefficient;
      protected: transport::PublisherPtr vis_pub;
    };
  }
}
#endif

