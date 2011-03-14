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
 * Desc: Position2d controller for a Holonome3Sw drive.
 * Author: Christian Gagneraud (ch'Gans), based on Differential_Position2d.cc
 * Date: 21 Feb 2007
 * SVN info: $Id$
 */

#include "XMLConfig.hh"
#include "Model.hh"
#include "Global.hh"
#include "Joint.hh"
#include "World.hh"
#include "Simulator.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Holonome3Sw_Position2d.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("holonome3sw_position2d", Holonome3Sw_Position2d);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
  Holonome3Sw_Position2d::Holonome3Sw_Position2d(Entity *parent )
: Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Holonome3Sxw_Position2d controller requires a Model as its parent");

  ResetData();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Holonome3Sw_Position2d::~Holonome3Sw_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Holonome3Sw_Position2d::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<libgazebo::PositionIface*>(this->GetIface("position"));

  // Get wheels child
  node = node->GetChild("wheels");
  if (!node)
    gzthrow("Holonome3Sw_Position2d controller requires a <wheels> node");

  // Get default params
  float distdef = node->GetFloat("distance", 0.0, 0);
  float radiusdef = node->GetFloat("radius", 0.0, 0);
  float maxtorquedef = node->GetFloat("torque", 0.0, 0);
  float betadef = node->GetFloat("beta", 0.0, 0);
  float gammadef = node->GetFloat("gamma", 0.0, 0);

  // Get params for each wheels
  for (size_t i = 0; i < 3; ++i)
  {
    std::ostringstream sw; sw << "swedish" << i;
    XMLConfigNode *lnode = node->GetChild(sw.str());
    if (!lnode)
      gzthrow("The controller couldn't get swedish wheels " + sw.str());

    std::string joint =  lnode->GetString("joint", "", 1);
    this->joint[i] = this->myParent->GetJoint(joint);

    if (!this->joint[i])
      gzthrow("The controller couldn't get hinge joint for " + sw.str());

    this->ALPHA[i] = lnode->GetFloat("alpha", 1000000, 1);
    if (this->ALPHA[i] == 1000000)
      gzthrow("The controller could't get alpha param for " + sw.str());

    this->DIST[i] = lnode->GetFloat("distance", distdef, 0);
    if (!this->DIST[i])
      gzthrow("The controller could't get distance param for " + sw.str());

    this->BETA[i] = lnode->GetFloat("beta", betadef, 0);

    this->GAMMA[i] = lnode->GetFloat("gamma", gammadef, 0);

    this->RADIUS[i] = lnode->GetFloat("radius", radiusdef, 0);
    if (!this->RADIUS[i])
      gzthrow("The controller could't get radius param for " + sw.str());

    this->MAXTORQUE[i] = lnode->GetFloat("torque", maxtorquedef, 0);
    if (!this->MAXTORQUE[i])
      gzthrow("The controller could't get torque param for " + sw.str());

    // Pre-compute some constants
    A[i] = (2*3.14159265358979)*fmodf(ALPHA[i]+BETA[i]+GAMMA[i], 360)/360;
    L[i] = DIST[i]*cos((2*3.14159265358979)*fmodf(BETA[i]+GAMMA[i], 360)/360);
    R[i] = RADIUS[i]*cos((2*3.14159265358979)*fmodf(GAMMA[i], 360)/360);
  }

#if 0
  std::cout << "Holonomous robot, here are the params:" << std::endl;
  for (size_t i = 0; i < 3; ++i)
  {
    std::cout << " - swedish" << i << std::endl;
    std::cout << "   alpha =      " << ALPHA[i] << std::endl;
    std::cout << "   beta =       " << BETA[i] << std::endl;
    std::cout << "   gamma =      " << GAMMA[i] << std::endl;
    std::cout << "   dist =       " << DIST[i] << std::endl;
    std::cout << "   radius =     " << RADIUS[i] << std::endl;
    std::cout << "   max-torque = " << MAXTORQUE[i] << std::endl;
    std::cout << "   A =          " << A[i] << std::endl;
    std::cout << "   L =          " << L[i] << std::endl;
    std::cout << "   R =          " << R[i] << std::endl;
    std::cout << std::endl;      
  }
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
/*void Holonome3Sw_Position2d::SaveChild(XMLConfigNode *node)
  {
  node = node->GetChild("wheels");
  if (!node)
  gzthrow("No such child node: wheels");
  for (size_t i = 0; i < 3; ++i)
  {
  std::ostringstream sw; sw << "swedish" << i;
  XMLConfigNode *lnode = node->GetChild(sw.str());
  if (!lnode)
  gzthrow("No such child node wheels/" + sw.str());
  lnode->SetValue("alpha", this->ALPHA[i]);
  lnode->SetValue("distance", this->DIST[i]);
  lnode->SetValue("beta", this->BETA[i]);
  lnode->SetValue("gamma", this->GAMMA[i]);
  lnode->SetValue("radius", this->RADIUS[i]);
  lnode->SetValue("max-torque", this->MAXTORQUE[i]);
// "joint" ???
}
}*/

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Holonome3Sw_Position2d::InitChild()
{
  ResetData();
}

void Holonome3Sw_Position2d::ResetChild()
{
  ResetData();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Holonome3Sw_Position2d::UpdateChild()
{
  this->GetPositionCmd();

  //std::cout << "Anchors: {";
  for (size_t i = 0; i < 3; ++i)
  {
    if (this->enableMotors)
    {
      this->joint[i]->SetVelocity( 0, this->PhiP[i]);
      this->joint[i]->SetMaxForce( 0, this->MAXTORQUE[i] );
    }
    else
    {
      this->joint[i]->SetVelocity( 0, 0 ); 
      this->joint[i]->SetMaxForce( 0, 0 );
    }
  }
  //std::cout << "}" << std::endl;
  this->PutPositionData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Holonome3Sw_Position2d::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Reset internal data to 0
void Holonome3Sw_Position2d::ResetData()
{
  for (size_t i = 0; i < 3; ++i)
  {
    Xi[i] = 0;
    XiP[i] = 0;
    PhiP[i] = 0;
  }
  this->enableMotors = true;
}

//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Holonome3Sw_Position2d::GetPositionCmd()
{
  if (this->myIface->Lock(1))
  {
    double vx = this->myIface->data->cmdVelocity.pos.x;
    double vy = this->myIface->data->cmdVelocity.pos.y;
    double va = this->myIface->data->cmdVelocity.yaw;
    for (size_t i = 0; i < 3; ++i)
    {
      this->PhiP[i] = -(1/R[i])*(-sin(A[i])*vx +
          cos(A[i])*vy +
          L[i]*va);
    }
    this->enableMotors = this->myIface->data->cmdEnableMotors > 0;
    this->myIface->Unlock();
#if 0
    std::cout << "Xi=[" << Xi[0] << ", " << Xi[1] << ", " << Xi[2] << "]; ";
    std::cout << "vx=" << vx <<", vy=" << vy << ", va=" << va << "; ";
    std::cout << "PhiP=[" << PhiP[0] << ", " << PhiP[1] << ", " << PhiP[2] << "];\n";
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void Holonome3Sw_Position2d::PutPositionData()
{
  if (this->myIface->Lock(1))
  {
    // TODO: Data timestamp
    this->myIface->data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

    this->myIface->data->pose.pos.x = this->Xi[0];
    this->myIface->data->pose.pos.y = this->Xi[1];
    this->myIface->data->pose.yaw = NORMALIZE(this->Xi[2]);

    this->myIface->data->velocity.pos.x = 0;
    this->myIface->data->velocity.pos.y = 0;
    this->myIface->data->velocity.yaw = 0;

    // TODO
    this->myIface->data->stall = 0;

    this->myIface->Unlock();
  }
}
