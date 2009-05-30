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
#include "gazebo.h"
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
    dJointFeedback *jFeedback = new dJointFeedback;
    int i =0;
    this->myIface = dynamic_cast<OpaqueIface*>(this->GetIface("opaque"));
    jNode = node->GetChild("joint");
    while(jNode && i < GAZEBO_JOINTFORCE_CONTROLLER_MAX_FEEDBACKS)
    {
        jointName = jNode->GetString("name","",1);
        joint = this->myParent->GetJoint(jointName);
        jFeedback = joint->GetFeedback();
        if(jFeedback) {
            this->jointfeedbacks[i] = jFeedback;
            i++;
        }
        jNode = jNode->GetNext("joint");
    }
    this->n_joints = i;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void JointForce::InitChild()
{
    //this->myIface->data->data = new uint8_t[sizeof(dJointFeedback)*n_joints];
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void JointForce::UpdateChild()
{
  this->myIface->Lock(1);
  this->myIface->data->head.time = Simulator::Instance()->GetSimTime();
  // Let me explain this number: each joint reports 4 vectors: Force and torque
  // on each jointed object, respectively. These vectors have 4 elements: x,y,z
  // and a fourth one. So we transmit 4 dReals per vector.
  this->myIface->data->data_count = n_joints*4*4*sizeof(dReal);

  for(int i=0; i< n_joints; i++) {
    // Copy vector for force on first object
    memcpy(this->myIface->data->data + (i*4 + 0)*4*sizeof(dReal), this->jointfeedbacks[i]->f1, 4*sizeof(dReal));
    // Copy vector for torque on first object
    memcpy(this->myIface->data->data + (i*4 + 1)*4*sizeof(dReal), this->jointfeedbacks[i]->t1, 4*sizeof(dReal));
    // Copy vector for force on second object
    memcpy(this->myIface->data->data + (i*4 + 2)*4*sizeof(dReal), this->jointfeedbacks[i]->f2, 4*sizeof(dReal));
    // Copy vector for torque on second object
    memcpy(this->myIface->data->data + (i*4 + 3)*4*sizeof(dReal), this->jointfeedbacks[i]->t2, 4*sizeof(dReal));
  }
  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void JointForce::FiniChild()
{
}
