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
/*
 * Desc: Position2d controller for a Pioneer2dx.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include <boost/bind.hpp>

#include "PhysicsEngine.hh"
#include "Geom.hh"
#include "Simulator.hh"
#include "RaySensor.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "Joint.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Pioneer2_Gripper.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("pioneer2_gripper", Pioneer2_Gripper);

enum {LEFT, RIGHT, LIFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Pioneer2_Gripper::Pioneer2_Gripper(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Pioneer2_Gripper controller requires a Model as its parent");

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pioneer2_Gripper::~Pioneer2_Gripper()
{
  this->paddles[LEFT]->DisconnectContactCallback(boost::bind(&Pioneer2_Gripper::LeftPaddleCB, this));
  this->paddles[RIGHT]->DisconnectContactCallback(boost::bind(&Pioneer2_Gripper::RightPaddleCB, this));

  if (this->holdJoint)
    delete this->holdJoint;
  this->holdJoint = NULL;

  for (int i=0; i<3; i++)
  {
    if (this->jointNamesP[i])
      delete this->jointNamesP[i];
    this->jointNamesP[i] = NULL;

    if (this->gainsP[i])
      delete this->gainsP[i];
    this->gainsP[i] = NULL;

    if (this->forcesP[i])
      delete this->forcesP[i];
    this->forcesP[i] = NULL;
  }

  for (int i =0; i<2; i++)
  {
    if (this->breakBeamNamesP[i])
      delete this->breakBeamNamesP[i];
    this->breakBeamNamesP[i] = NULL;

    if (this->paddleNamesP[i])
      delete this->paddleNamesP[i];
    this->paddleNamesP[i] = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Pioneer2_Gripper::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *jNode;
  this->gripIface = dynamic_cast<libgazebo::GripperIface*>(this->GetIface("gripper"));
  this->actIface = dynamic_cast<libgazebo::ActarrayIface*>(this->GetIface("actarray"));

  Param::Begin(&this->parameters);
  jNode = node->GetChild("leftJoint");
  if (jNode)
  {
    this->jointNamesP[LEFT] = new ParamT<std::string>("name", "", 1); 
    this->jointNamesP[LEFT]->Load(jNode);

    this->forcesP[LEFT] = new ParamT<float>("force",0.01,0);
    this->forcesP[LEFT]->Load(jNode);

    this->gainsP[LEFT] = new ParamT<float>("gain",0.01,0);
    this->gainsP[LEFT]->Load(jNode);

    this->joints[LEFT] = this->myParent->GetJoint(**this->jointNamesP[LEFT]);
  }

  jNode = node->GetChild("rightJoint");
  if (jNode)
  {
    this->jointNamesP[RIGHT] = new ParamT<std::string>("name", "", 1); 
    this->jointNamesP[RIGHT]->Load(jNode);

    this->forcesP[RIGHT] = new ParamT<float>("force",0.01,0);
    this->forcesP[RIGHT]->Load(jNode);

    this->gainsP[RIGHT] = new ParamT<float>("gain",0.01,0);
    this->gainsP[RIGHT]->Load(jNode);

    this->joints[RIGHT] = this->myParent->GetJoint(**this->jointNamesP[RIGHT]);
  }

  jNode = node->GetChild("liftJoint");
  if (jNode)
  {
    this->jointNamesP[LIFT] = new ParamT<std::string>("name", "", 1); 
    this->jointNamesP[LIFT]->Load(jNode);

    this->forcesP[LIFT] = new ParamT<float>("force",0.01,0);
    this->forcesP[LIFT]->Load(jNode);

    this->gainsP[LIFT] = new ParamT<float>("gain",0.01,0);
    this->gainsP[LIFT]->Load(jNode);

    this->joints[LIFT] = this->myParent->GetJoint(**this->jointNamesP[LIFT]);
  }

  this->breakBeamNamesP[0] = new ParamT<std::string>("name","",1);
  this->breakBeamNamesP[1] = new ParamT<std::string>("name","",1);

  this->breakBeamNamesP[0]->Load(node->GetChild("outerBreakBeam"));
  this->breakBeamNamesP[1]->Load(node->GetChild("innerBreakBeam"));

  this->paddleNamesP[LEFT] = new ParamT<std::string>("name","",1);
  this->paddleNamesP[RIGHT] = new ParamT<std::string>("name","",1);

  this->paddleNamesP[LEFT]->Load(node->GetChild("leftPaddle"));
  this->paddleNamesP[RIGHT]->Load(node->GetChild("rightPaddle"));

  Param::End();

  this->breakBeams[0] = dynamic_cast<RaySensor*>(this->myParent->GetSensor(**this->breakBeamNamesP[0]));
  this->breakBeams[1] = dynamic_cast<RaySensor*>(this->myParent->GetSensor(**this->breakBeamNamesP[1]));

  this->paddles[LEFT] = dynamic_cast<Geom*>(this->myParent->GetGeom(**this->paddleNamesP[LEFT]));
  this->paddles[RIGHT] = dynamic_cast<Geom*>(this->myParent->GetGeom(**this->paddleNamesP[RIGHT]));

  if (!this->joints[LEFT])
    gzthrow("couldn't get left slider joint");

  if (!this->joints[RIGHT])
    gzthrow("couldn't get right slider joint");

  if (!this->joints[LIFT])
    gzthrow("couldn't get lift slider joint");

  if (!this->breakBeams[0])
    gzthrow("Couldn't get outer breakbeam sensor");

  if (!this->breakBeams[1])
    gzthrow("Couldn't get inner breakbeam sensor");

  if (!this->paddles[LEFT])
    gzthrow("Couldn't get the left paddle geom");

  if (!this->paddles[RIGHT])
    gzthrow("Couldn't get the right paddle geom");

  this->holdJoint = World::Instance()->GetPhysicsEngine()->CreateJoint("slider");
  this->holdJoint->SetName(this->GetName() + "_Hold_Joint");

  this->paddles[LEFT]->ConnectContactCallback(
      boost::bind(&Pioneer2_Gripper::LeftPaddleCB, this, _1));
  this->paddles[RIGHT]->ConnectContactCallback(
      boost::bind(&Pioneer2_Gripper::RightPaddleCB, this, _1));
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller
void Pioneer2_Gripper::SaveChild(std::string &prefix, std::ostream &stream)
{
  for (int i=0; i < 3; i++)
  {
    stream << prefix << "<joint name=\"" << this->jointNamesP[i]->GetValue() << "\">\n";
    stream << prefix << "  " << *(this->forcesP[i]) << "\n";
    stream << prefix << "  " << *(this->gainsP[i]) << "\n";
    stream << prefix << "</joint>\n";
  }

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Pioneer2_Gripper::InitChild()
{
  // Initially keep the gripper closed
  this->joints[RIGHT]->SetVelocity(0, -0.1);
  this->joints[LEFT]->SetVelocity(0, 0.1);
  this->joints[LEFT]->SetMaxForce(0, **(this->forcesP[LEFT]));
  this->joints[RIGHT]->SetMaxForce(0, **(this->forcesP[RIGHT]));


  // Initially lower the lift
  this->joints[LIFT]->SetMaxForce(0, **(this->forcesP[LIFT]));
  this->joints[LIFT]->SetMaxForce(0, **(this->forcesP[LIFT]));

  this->contactGeoms[LEFT] = this->contactGeoms[RIGHT] = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Pioneer2_Gripper::UpdateChild()
{
  // Create a hold joint, if both paddles are touching the same geom
  if (this->contactGeoms[LEFT] && this->contactGeoms[RIGHT] &&
      this->contactGeoms[LEFT]->GetName() == 
      this->contactGeoms[RIGHT]->GetName())
  {
    if (!this->holdJoint->AreConnected(this->myParent->GetBody(),
          this->contactGeoms[LEFT]->GetBody()))
    {
      this->holdJoint->Attach(this->myParent->GetBody(), 
                              this->contactGeoms[LEFT]->GetBody()); 
      this->holdJoint->SetAxis(0, Vector3(0,0,1));

    }
  }
  // Otherwise disconnect the joint if it has been created
  else if (this->holdJoint->GetJointBody(0))
  {
    this->holdJoint->Detach();
    this->contactGeoms[LEFT] = this->contactGeoms[RIGHT] = NULL;
  }

  this->gripIface->Lock(1);

  // Move the paddles
  switch( this->gripIface->data->cmd)
  {
    case GAZEBO_GRIPPER_CMD_OPEN:
      this->joints[RIGHT]->SetVelocity(0, 0.1);
      this->joints[LEFT]->SetVelocity(0, -0.1);
      break;

    case GAZEBO_GRIPPER_CMD_CLOSE:
      this->joints[RIGHT]->SetVelocity(0, -0.1);
      this->joints[LEFT]->SetVelocity(0,0.1);
      break;

    case GAZEBO_GRIPPER_CMD_STOP:
      this->joints[RIGHT]->SetVelocity(0,0);
      this->joints[LEFT]->SetVelocity(0,0);
      break;
  }

  // Move the lift
  if (this->actIface->data->cmd_pos[0] > 0.5)
  {
    this->joints[LIFT]->SetVelocity(0, 0.2);
  }
  else if (this->actIface->data->cmd_pos[0] < 0.5)
  {
    this->joints[LIFT]->SetVelocity(0, -0.2);
  }

  this->joints[LEFT]->SetMaxForce(0, **(this->forcesP[LEFT]));
  this->joints[RIGHT]->SetMaxForce(0, **(this->forcesP[RIGHT]));
  this->joints[LIFT]->SetMaxForce(0, **(this->forcesP[LIFT]));


  // DEBUG Statements
  /*printf("Left Pos[%f] High[%f] Low[%f]\n",this->joints[LEFT]->GetPosition(), this->joints[LEFT]->GetHighStop(),this->joints[LEFT]->GetLowStop() );
  printf("Right Pos[%f] High[%f] Low[%f]\n",this->joints[RIGHT]->GetPosition(), this->joints[RIGHT]->GetHighStop(),this->joints[RIGHT]->GetLowStop() );
  printf("Lift Pos[%f] High[%f] Low[%f]\n", this->joints[LIFT]->GetPosition(),
  this->joints[LIFT]->GetHighStop(), this->joints[LIFT]->GetLowStop());
  */

  // Set the state of the left paddle pressure sensor
  if (this->contactGeoms[LEFT])
    this->gripIface->data->left_paddle_open = 0;
  else
    this->gripIface->data->left_paddle_open = 1;
    
  // Set the state of the right paddle pressure sensor
  if (this->contactGeoms[RIGHT])
    this->gripIface->data->right_paddle_open = 0;
  else
    this->gripIface->data->right_paddle_open = 1;


  Angle leftAngleDiff = this->joints[LEFT]->GetAngle(0) - 
                        this->joints[LEFT]->GetHighStop(0);

  Angle rightAngleDiff = this->joints[RIGHT]->GetAngle(0) - 
                         this->joints[RIGHT]->GetHighStop(0);


  // Set the OPEN/CLOSED/MOVING state of the gripper
  if (fabs(leftAngleDiff.GetAsRadian()) < 0.01 &&
      fabs(rightAngleDiff.GetAsRadian()) < 0.01)
    this->gripIface->data->state = GAZEBO_GRIPPER_STATE_CLOSED;
  else if (fabs(leftAngleDiff.GetAsRadian()) < 0.01 &&
           fabs(rightAngleDiff.GetAsRadian()) < 0.01)
    this->gripIface->data->state = GAZEBO_GRIPPER_STATE_OPEN;
  else
    this->gripIface->data->state = GAZEBO_GRIPPER_STATE_MOVING;

  // Set the UP/DOWN state of the lift
  if (fabs(leftAngleDiff.GetAsRadian()) < 0.01)
    this->actIface->data->actuators[0].position = 1;
  else if (fabs(rightAngleDiff.GetAsRadian()) < 0.01)
    this->actIface->data->actuators[0].position = 0;


  // Check the break beams
  if (this->gripIface->data->state == GAZEBO_GRIPPER_STATE_OPEN)
  {
    if (this->breakBeams[0]->GetRange(0) < 0.22)
      this->gripIface->data->outer_beam_obstruct = 1;
    else
      this->gripIface->data->outer_beam_obstruct = 0;

    if (this->breakBeams[1]->GetRange(0) < 0.22)
      this->gripIface->data->inner_beam_obstruct = 1;
    else
      this->gripIface->data->inner_beam_obstruct = 0;
  }

  this->actIface->data->actuators_count = 1;

  this->gripIface->data->head.time = Simulator::Instance()->GetSimTime().Double();
  this->gripIface->Post();
  this->gripIface->Unlock();

  // Reset these flags.
  this->contactGeoms[LEFT] = this->contactGeoms[RIGHT] = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Pioneer2_Gripper::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Left paddle contact callback
void Pioneer2_Gripper::LeftPaddleCB(const Contact &contact)
{
  if (contact.geom1->GetName() != this->paddles[LEFT]->GetName())
    this->contactGeoms[LEFT] = contact.geom1;
  else
    this->contactGeoms[LEFT] = contact.geom2;
}

////////////////////////////////////////////////////////////////////////////////
// Right paddle contact callback
void Pioneer2_Gripper::RightPaddleCB(const Contact &contact)
{
  if (contact.geom1->GetName() != this->paddles[RIGHT]->GetName())
    this->contactGeoms[RIGHT] = contact.geom1;
  else
    this->contactGeoms[RIGHT] = contact.geom2;
}
