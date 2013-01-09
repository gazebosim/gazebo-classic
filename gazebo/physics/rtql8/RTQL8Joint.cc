/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: The RTQL8 base joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 12 Oct 2009
 */

#include "common/Exception.hh"
#include "common/Console.hh"

#include "physics/World.hh"
#include "physics/Link.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/rtql8/RTQL8Link.hh"
#include "physics/rtql8/RTQL8Joint.hh"
//#include "physics/ScrewJoint.hh"

#include "rtql8/kinematics/Joint.h"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
RTQL8Joint::RTQL8Joint(BasePtr _parent)
  : Joint(_parent), rtql8Joint(NULL)
{
}

//////////////////////////////////////////////////
RTQL8Joint::~RTQL8Joint()
{
  this->Detach();

  if (rtql8Joint)
    delete rtql8Joint;
}

//////////////////////////////////////////////////
void RTQL8Joint::Load(sdf::ElementPtr _sdf)
{
   Joint::Load(_sdf);

   // In case this joint is already loaded, we delete rtql8 joint if it is
   // created.
   if (rtql8Joint)
     delete rtql8Joint;

   // In Joint::Load(sdf::ElementPtr), this joint stored the parent joint and
   // child joint.
   kinematics::BodyNode* parentBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
         this->parentLink)->GetBodyNode();
   kinematics::BodyNode* childBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
         this->childLink)->GetBodyNode();

   // In order to create rtql8 joint, we need to know this joint's parent
   // and child link so we create rtql8 joint after the joint is loaded with sdf
   // .
   rtql8Joint = new kinematics::Joint(parentBodyNode, childBodyNode);
}

//////////////////////////////////////////////////
void RTQL8Joint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr RTQL8Joint::GetJointLink(int _index) const
{
  LinkPtr result;

  if (_index == 0)
  {
    RTQL8LinkPtr rtql8Link1
        = boost::shared_static_cast<RTQL8Link>(this->parentLink);

    if (rtql8Link1 != NULL)
      return this->parentLink;
  }

  if (_index == 1)
  {
    RTQL8LinkPtr rtql8Link2
        = boost::shared_static_cast<RTQL8Link>(this->childLink);

    if (rtql8Link2 != NULL)
      return this->childLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool RTQL8Joint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
//  RTQL8LinkPtr rtql8Link1 = boost::shared_dynamic_cast<RTQL8Link>(_one);
//  RTQL8LinkPtr rtql8Link2 = boost::shared_dynamic_cast<RTQL8Link>(_two);

//  if (rtql8Link1 == NULL || rtql8Link2 == NULL)
//    gzthrow("RTQL8Joint requires RTQL8 bodies\n");

//  return dAreConnected(odeLink1->GetODEId(), odeLink2->GetODEId());
  return (this->childLink.get() == _one.get() && this->parentLink.get() == _two.get())
      || (this->childLink.get() == _two.get() && this->parentLink.get() == _one.get());
}

//////////////////////////////////////////////////
void RTQL8Joint::Attach(LinkPtr _parent, LinkPtr _child)
{
   Joint::Attach(_parent, _child);

   RTQL8LinkPtr rtql8child = boost::shared_dynamic_cast<RTQL8Link>(this->childLink);
   RTQL8LinkPtr rtql8parent = boost::shared_dynamic_cast<RTQL8Link>(this->parentLink);

   if (rtql8child == NULL && rtql8parent == NULL)
     gzthrow("RTQL8Joint requires at least one RTQL8 link\n");

   // TODO: RTQL8's joint can't change their links connected.
   // For now, recreating the joint is the only way.
   if (this->rtql8Joint)
     delete this->rtql8Joint;

   kinematics::BodyNode* parentBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
         this->parentLink)->GetBodyNode();
   kinematics::BodyNode* childBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
         this->childLink)->GetBodyNode();

   this->rtql8Joint = new kinematics::Joint(parentBodyNode, childBodyNode);
}

//////////////////////////////////////////////////
void RTQL8Joint::Detach()
{
   this->childLink.reset();
   this->parentLink.reset();

  // TODO: RTQL8's joint can't change their links connected.
  // For now, recreating the joint is the only way.
  if (this->rtql8Joint)
    delete this->rtql8Joint;

//  kinematics::BodyNode* parentBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
//        this->parentLink)->GetBodyNode();
//  kinematics::BodyNode* childBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
//        this->childLink)->GetBodyNode();

  this->rtql8Joint = new kinematics::Joint(NULL, NULL);
}

//////////////////////////////////////////////////
void RTQL8Joint::SetHighStop(int _index, const math::Angle &_angle)
{
   switch (_index)
   {
     case 0:
//       this->SetParam(dParamHiStop, _angle.Radian());
       break;
     case 1:
//       this->SetParam(dParamHiStop2, _angle.Radian());
       break;
     case 2:
//       this->SetParam(dParamHiStop3, _angle.Radian());
       break;
     default:
       gzerr << "Invalid index[" << _index << "]\n";
       break;
   };
}

//////////////////////////////////////////////////
void RTQL8Joint::SetLowStop(int _index, const math::Angle &_angle)
{
   switch (_index)
   {
     case 0:
//       this->SetParam(dParamLoStop, _angle.Radian());
       break;
     case 1:
//       this->SetParam(dParamLoStop2, _angle.Radian());
       break;
     case 2:
//       this->SetParam(dParamLoStop3, _angle.Radian());
       break;
     default:
       gzerr << "Invalid index[" << _index << "]\n";
   };
}

//////////////////////////////////////////////////
math::Angle RTQL8Joint::GetHighStop(int _index)
{
//   switch (_index)
//   {
//     case 0:
//       return this->GetParam(dParamHiStop);
//     case 1:
//       return this->GetParam(dParamHiStop2);
//     case 2:
//       return this->GetParam(dParamHiStop3);
//     default:
//       gzerr << "Invalid index[" << _index << "]\n";
//   };

  return 0;
}

//////////////////////////////////////////////////
math::Angle RTQL8Joint::GetLowStop(int _index)
{
//   switch (_index)
//   {
//     case 0:
//       return this->GetParam(dParamLoStop);
//     case 1:
//       return this->GetParam(dParamLoStop2);
//     case 2:
//       return this->GetParam(dParamLoStop3);
//     default:
//       gzerr << "Invalid index[" << _index << "]\n";
//   };

  return 0;
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Joint::GetLinkForce(unsigned int _index) const
{
  math::Vector3 result;
//   dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);
// 
//   if (_index == 0)
//     result.Set(jointFeedback->f1[0], jointFeedback->f1[1],
//                jointFeedback->f1[2]);
//   else
//     result.Set(jointFeedback->f2[0], jointFeedback->f2[1],
//                jointFeedback->f2[2]);

  return result;
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Joint::GetLinkTorque(unsigned int _index) const
{
  math::Vector3 result;
//   dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);
// 
//   if (_index == 0)
//     result.Set(jointFeedback->t1[0], jointFeedback->t1[1],
//                jointFeedback->t1[2]);
//   else
//     result.Set(jointFeedback->t2[0], jointFeedback->t2[1],
//                jointFeedback->t2[2]);

  return result;
}

//////////////////////////////////////////////////
void RTQL8Joint::SetAttribute(Attribute _attr, int /*_index*/, double _value)
{
//   switch (_attr)
//   {
//     case FUDGE_FACTOR:
//       this->SetParam(dParamFudgeFactor, _value);
//       break;
//     case SUSPENSION_ERP:
//       this->SetParam(dParamSuspensionERP, _value);
//       break;
//     case SUSPENSION_CFM:
//       this->SetParam(dParamSuspensionCFM, _value);
//       break;
//     case STOP_ERP:
//       this->SetParam(dParamStopERP, _value);
//       break;
//     case STOP_CFM:
//       this->SetParam(dParamStopCFM, _value);
//       break;
//     case ERP:
//       this->SetParam(dParamERP, _value);
//       break;
//     case CFM:
//       this->SetParam(dParamCFM, _value);
//       break;
//     case FMAX:
//       this->SetParam(dParamFMax, _value);
//       break;
//     case VEL:
//       this->SetParam(dParamVel, _value);
//       break;
//     case HI_STOP:
//       this->SetParam(dParamHiStop, _value);
//       break;
//     case LO_STOP:
//       this->SetParam(dParamLoStop, _value);
//       break;
//     default:
//       gzerr << "Unable to handle joint attribute[" << _attr << "]\n";
//       break;
//   };
}

//////////////////////////////////////////////////
void RTQL8Joint::SetAttribute(const std::string &_key, int /*_index*/,
                            const boost::any &_value)
{
//   if (_key == "fudge_factor")
//   {
//     try
//     {
//       this->SetParam(dParamFudgeFactor, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "suspension_erp")
//   {
//     try
//     {
//       this->SetParam(dParamSuspensionERP, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "suspension_cfm")
//   {
//     try
//     {
//       this->SetParam(dParamSuspensionCFM, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "stop_erp")
//   {
//     try
//     {
//       this->SetParam(dParamStopERP, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "stop_cfm")
//   {
//     try
//     {
//       this->SetParam(dParamStopCFM, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "erp")
//   {
//     try
//     {
//       this->SetParam(dParamERP, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "cfm")
//   {
//     try
//     {
//       this->SetParam(dParamCFM, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "fmax")
//   {
//     try
//     {
//       this->SetParam(dParamFMax, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "vel")
//   {
//     try
//     {
//       this->SetParam(dParamVel, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "hi_stop")
//   {
//     try
//     {
//       this->SetParam(dParamHiStop, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "lo_stop")
//   {
//     try
//     {
//       this->SetParam(dParamLoStop, boost::any_cast<double>(_value));
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
//   else if (_key == "thread_pitch")
//   {
//     ScrewJoint<RTQL8Joint>* screwJoint =
//       dynamic_cast<ScrewJoint<RTQL8Joint>* >(this);
//     if (screwJoint != NULL)
//     {
//       try
//       {
//         screwJoint->SetThreadPitch(0, boost::any_cast<double>(_value));
//       }
//       catch(boost::bad_any_cast &e)
//       {
//         gzerr << "boost any_cast error:" << e.what() << "\n";
//       }
//     }
//   }
//   else
//   {
//     try
//     {
//       gzerr << "Unable to handle joint attribute["
//             << boost::any_cast<std::string>(_value) << "]\n";
//     }
//     catch(boost::bad_any_cast &e)
//     {
//       gzerr << "boost any_cast error:" << e.what() << "\n";
//     }
//   }
}
