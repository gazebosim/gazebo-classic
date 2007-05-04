#include <stdio.h>
#include "Body.hh"
#include "PlaneGeom.hh"
#include "ModelFactory.hh"
#include "XMLConfig.hh"
#include "GroundPlane.hh"

using namespace gazebo;

GZ_REGISTER_STATIC("GroundPlane", GroundPlane);

GroundPlane::GroundPlane()
{
}

GroundPlane::~GroundPlane()
{
}

// Load the child model
int GroundPlane::LoadChild(XMLConfigNode *node)
{
  this->body = this->CreateBody();

  Vector3 normal = node->GetVector3("normal",Vector3(0,1,0));
  double altitude = node->GetDouble("altitude",0,0);

  this->geom = new PlaneGeom(this->body,altitude,normal);

  Pose3d pose(node->GetVector3("xyz",Vector3(0,0,0)),
      node->GetRotation("rpt",Quatern(1,0,0,0)));

  this->body->SetPose(pose);
  this->body->SetEnabled(false);

  this->geom->SetMeshMaterial(node->GetString("material","Gazebo/GrassFloor",0));

  return 0;
}

// Initialize the child model
int GroundPlane::InitChild()
{
  return 0;
}

// Update the child model
int GroundPlane::UpdateChild()
{
  return 0;
}

// Finilaize thie child model
int GroundPlane::FiniChild()
{
  return 0;
}
