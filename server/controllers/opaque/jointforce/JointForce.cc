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
 * Desc: Joint Force controller
 * Author: Benjamin Kloster
 * Date: 13 March 2008
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Simulator.hh"
#include "JointForce.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("jointforce", JointForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
  JointForce::JointForce(Entity *parent )
: Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("ControllerStub controller requires a Model as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
JointForce::~JointForce()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void JointForce::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *jNode;
  Joint *joint;
  std::string jointName;

  this->myIface = dynamic_cast<libgazebo::OpaqueIface*>(this->GetIface("opaque"));

  jNode = node->GetChild("joint");

  while(jNode)// && i < GAZEBO_JOINTFORCE_CONTROLLER_MAX_FEEDBACKS)
  {
    jointName = jNode->GetString("name","",1);
    joint = this->myParent->GetJoint(jointName);
    this->joints.push_back(joint);
    jNode = jNode->GetNext("joint");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void JointForce::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void JointForce::UpdateChild()
{
  this->myIface->Lock(1);
  this->myIface->data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

  // Let me explain this number: each joint reports 4 vectors: Force and torque
  // on each jointed object, respectively. These vectors have 3 elements: x,y,z.
  // So we transmit 3 doubles per vector.
  this->myIface->data->data_count = this->joints.size()*4*3*sizeof(double);

  uint8_t *ptr = this->myIface->data->data;

  for(unsigned int i=0; i< this->joints.size(); i++) 
  {
    Vector3 bf1, bf2, bt1, bt2;
    bf1 = this->joints[i]->GetBodyForce(0);
    bf2 = this->joints[i]->GetBodyForce(1);
    bt1 = this->joints[i]->GetBodyTorque(0);
    bt2 = this->joints[i]->GetBodyTorque(1);

    *ptr = bf1.x; 
    ptr += sizeof(double);

    *ptr = bf1.y; 
    ptr += sizeof(double);

    *ptr = bf1.z; 
    ptr += sizeof(double);

    *ptr++ = bt1.x; 
    ptr += sizeof(double);

    *ptr++ = bt1.y; 
    ptr += sizeof(double);

    *ptr++ = bt1.z; 
    ptr += sizeof(double);

    *ptr = bf2.x; 
    ptr += sizeof(double);

    *ptr = bf2.y; 
    ptr += sizeof(double);

    *ptr = bf2.z; 
    ptr += sizeof(double);

    *ptr = bt2.x; 
    ptr += sizeof(double);

    *ptr = bt2.y; 
    ptr += sizeof(double);

    *ptr = bt2.z; 
    ptr += sizeof(double);
  }

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void JointForce::FiniChild()
{
}
