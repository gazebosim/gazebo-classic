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
/*
 * Desc: Joint Force controller
 * Author: Benjamin Kloster
 * Date: 13 March 2008
 */

#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "common/Exception.hh"
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
