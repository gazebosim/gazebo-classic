#include <stdio.h>
#include "Body.hh"
#include "PlaneGeom.hh"
#include "SphereGeom.hh"
#include "BoxGeom.hh"
#include "HingeJoint.hh"
#include "CylinderGeom.hh"
#include "ModelFactory.hh"
#include "XMLConfig.hh"
#include "Global.hh"
#include "gazebo.h"
#include "World.hh"
#include "Pioneer2DX.hh"

GZ_REGISTER_STATIC("pioneer2dx", Pioneer2DX);

Pioneer2DX::Pioneer2DX()
{
}

Pioneer2DX::~Pioneer2DX()
{
}

// Load the child model
int Pioneer2DX::LoadChild(XMLConfigNode * /*node*/)
{
}

// Initialize the child model
int Pioneer2DX::InitChild()
{
  // Set initial interface state

  this->position_iface->data->cmd_enable_motors = 1;

  // Reset odometric pose
  this->odomPose[0] = 0.0;
  this->odomPose[1] = 0.0;
  this->odomPose[2] = 0.0;

  return 0;
}

// Update the child model
int Pioneer2DX::UpdateChild()
{
  return 0;
}

// Finilaize thie child model
int Pioneer2DX::FiniChild()
{
  return 0;
}

void Pioneer2DX::UpdateOdometry( double step )
{
  double d1, d2;
  double dr, da;

  // Distance travelled by front wheels
  d1 = step * this->wheelDiam / 2.0 * wheelJoints[0]->GetAngleRate();
  d2 = step * this->wheelDiam / 2.0 * wheelJoints[1]->GetAngleRate();

  dr = (d1 + d2) / 2.0;
  da = (d2 - d1) / this->wheelSep;
  
  // Compute odometric pose
  this->odomPose[0] += dr * cos( this->odomPose[2] );
  this->odomPose[1] += dr * sin( this->odomPose[2] );
  this->odomPose[2] += da;

  // Compute odometric instantaneous velocity
  this->odomVel[0] = dr / step;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = da / step;

  return;
}

