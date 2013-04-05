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

#include <sstream>

#include "gazebo/common/Console.hh"
#include "gazebo/math/Box.hh"

#include "gazebo/physics/rtql8/rtql8_inc.h"
//#include "gazebo/physics/SurfaceParams.hh"
//#include "gazebo/physics/rtql8/RTQL8Physics.hh"
#include "gazebo/physics/rtql8/RTQL8Link.hh"
#include "gazebo/physics/rtql8/RTQL8Collision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Collision::RTQL8Collision(LinkPtr _link)
  : Collision(_link),
    rtql8BodyNode(NULL),
    rtql8CollShape(NULL)
{
}

//////////////////////////////////////////////////
RTQL8Collision::~RTQL8Collision()
{
}

//////////////////////////////////////////////////
void RTQL8Collision::Load(sdf::ElementPtr /*_sdf*/)
{
  Collision::Load(_sdf);

}

//////////////////////////////////////////////////
void RTQL8Collision::Init()
{
  this->rtql8BodyNode
      = boost::shared_static_cast<RTQL8Link>(this->link)->GetBodyNode();

  sdf::ElementPtr geometryElem = this->sdf->GetElement("geometry");
  std::string geomType = geometryElem->GetFirstElement()->GetName();

  if (geomType == "sphere")
  {
    double radius = geometryElem->GetFirstElement()->GetValueDouble("radius");
    Eigen::Vector3d eigenSize(radius*2, radius*2, radius*2);
    rtql8::kinematics::ShapeEllipsoid* rtql8Shape
        = new rtql8::kinematics::ShapeEllipsoid(eigenSize, 1);
    rtql8BodyNode->setCollisionShape(rtql8Shape);
  }
  else if (geomType == "plane")
  {
    // TODO: rtql8 does not support plane!!!
    //      math::Vector3 normal
    //          = geometryElem->GetFirstElement()->GetValueVector3("normal");
    math::Vector2d size
        = geometryElem->GetFirstElement()->GetValueVector2d("size");
//    Eigen::Vector3d eigenSize(size.x, size.y, 0.001);
    Eigen::Vector3d eigenSize(100, 100, 0.001);
    rtql8::kinematics::ShapeCube* rtql8Shape
        = new rtql8::kinematics::ShapeCube(eigenSize, 1);
    rtql8BodyNode->setCollisionShape(rtql8Shape);
  }

  else if (geomType == "box")
  {
    math::Vector3 mathSize
        = geometryElem->GetFirstElement()->GetValueVector3("size");
    Eigen::Vector3d eigenSize(mathSize.x, mathSize.y, mathSize.z);
    rtql8::kinematics::ShapeCube* rtql8Shape
        = new rtql8::kinematics::ShapeCube(eigenSize, 1);
    rtql8BodyNode->setCollisionShape(rtql8Shape);
  }
  else if (geomType == "cylinder")
  {
    double radius = geometryElem->GetFirstElement()->GetValueDouble("radius");
    double length = geometryElem->GetFirstElement()->GetValueDouble("length");
    Eigen::Vector3d eigenSize(radius, length, 0.0);
    //      rtql8::kinematics::ShapeCylinder* shape
    //              = new rtql8::kinematics::ShapeCylinder(radius, length);
    rtql8::kinematics::ShapeCylinder* rtql8Shape
        = new rtql8::kinematics::ShapeCylinder(eigenSize, 1);
    rtql8BodyNode->setCollisionShape(rtql8Shape);
  }
  else if (geomType == "multiray")
    gzerr << "Not implemented yet...";
  else if (geomType == "mesh" || geomType == "trimesh")
    gzerr << "Not implemented yet...";
  else if (geomType == "heightmap")
    gzerr << "Not implemented yet...";
  else if (geomType == "map" || geomType == "image")
    gzerr << "Not implemented yet...";
  else if (geomType == "ray")
    gzerr << "Not implemented yet...";
  else
    gzerr << "Unknown visual type[" << geomType << "]\n";
}

//////////////////////////////////////////////////
void RTQL8Collision::Fini()
{

  Collision::Fini();
}

//////////////////////////////////////////////////
void RTQL8Collision::SetCollision(bool /*_placeable*/)
{
  //   // Must go first in this function
  //   this->collisionId = _collisionId;
  //
  Collision::SetCollision(_placeable);
  //
  //   if (dGeomGetSpace(this->collisionId) == 0)
  //   {
  //     dSpaceAdd(this->spaceId, this->collisionId);
  //     assert(dGeomGetSpace(this->collisionId) != 0);
  //   }
  //
  //   if (this->collisionId && this->placeable)
  //   {
  //     if (this->IsStatic())
  //       this->onPoseChangeFunc = &RTQL8Collision::OnPoseChangeGlobal;
  //     else
  //       this->onPoseChangeFunc = &RTQL8Collision::OnPoseChangeRelative;
  //   }
  //   else
  //   {
  //     this->onPoseChangeFunc = &RTQL8Collision::OnPoseChangeNull;
  //   }
  //
  //   dGeomSetData(this->collisionId, this);
  gzerr << "Not yet implemented." << std::endl;
}

//////////////////////////////////////////////////
void RTQL8Collision::OnPoseChange()
{
}

//////////////////////////////////////////////////
void RTQL8Collision::SetCategoryBits(unsigned int /*_bits*/)
{
  //   if (this->collisionId)
  //     dGeomSetCategoryBits(this->collisionId, _bits);
  //   if (this->spaceId)
  //     dGeomSetCategoryBits((dGeomID)this->spaceId, _bits);
  gzerr << "Not yet implemented." << std::endl;
}

//////////////////////////////////////////////////
void RTQL8Collision::SetCollideBits(unsigned int /*_bits*/)
{
  //   if (this->collisionId)
  //     dGeomSetCollideBits(this->collisionId, _bits);
  //   if (this->spaceId)
  //     dGeomSetCollideBits((dGeomID)this->spaceId, _bits);

  gzerr << "Not yet implemented." << std::endl;
}

//////////////////////////////////////////////////
math::Box RTQL8Collision::GetBoundingBox() const
{
  math::Box box;
  //   dReal aabb[6];
  //
  //   memset(aabb, 0, 6 * sizeof(dReal));
  //
  //   if (this->collisionId == NULL)
  //     printf("HOW IS THIS NULL\n");
  //
  //   // if (this->collisionId && this->type != Shape::PLANE)
  //   dGeomGetAABB(this->collisionId, aabb);
  //
  //   box.min.Set(aabb[0], aabb[2], aabb[4]);
  //   box.max.Set(aabb[1], aabb[3], aabb[5]);
  gzerr << "Not yet implemented." << std::endl;

  return box;
}
