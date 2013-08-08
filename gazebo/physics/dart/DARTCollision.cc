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

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTCollision::DARTCollision(LinkPtr _link)
  : Collision(_link),
    dartBodyNode(NULL),
    dartCollShape(NULL)
{
}

//////////////////////////////////////////////////
DARTCollision::~DARTCollision()
{
}

//////////////////////////////////////////////////
void DARTCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

}

//////////////////////////////////////////////////
void DARTCollision::Init()
{
  Collision::Init();

  this->dartBodyNode
      = boost::shared_static_cast<DARTLink>(this->link)->GetBodyNode();

  sdf::ElementPtr geometryElem = this->sdf->GetElement("geometry");
  std::string geomType = geometryElem->GetFirstElement()->GetName();

  if (geomType == "sphere")
  {
    double radius = geometryElem->GetFirstElement()->Get<double>("radius");
    Eigen::Vector3d eigenSize(radius*2, radius*2, radius*2);
    dartCollShape = new dart::dynamics::ShapeEllipsoid(eigenSize);
    dartBodyNode->addCollisionShape(dartCollShape);
  }
  else if (geomType == "plane")
  {
    // TODO: dart does not support plane!!!
    //      math::Vector3 normal
    //          = geometryElem->GetFirstElement()->GetValueVector3("normal");
    math::Vector2d size
        = geometryElem->GetFirstElement()->Get<math::Vector2d>("size");
    Eigen::Vector3d eigenSize(2100, 2100, 0.001);
    dartCollShape = new dart::dynamics::ShapeBox(eigenSize);
    dartBodyNode->addCollisionShape(dartCollShape);
  }
  else if (geomType == "box")
  {
    math::Vector3 mathSize
        = geometryElem->GetFirstElement()->Get<math::Vector3>("size");
    Eigen::Vector3d eigenSize(mathSize.x, mathSize.y, mathSize.z);
    dartCollShape = new dart::dynamics::ShapeBox(eigenSize);
    dartBodyNode->addCollisionShape(dartCollShape);
  }
  else if (geomType == "cylinder")
  {
    double radius = geometryElem->GetFirstElement()->Get<double>("radius");
    double length = geometryElem->GetFirstElement()->Get<double>("length");
    dartCollShape = new dart::dynamics::ShapeCylinder(radius, length);
    dartBodyNode->addCollisionShape(dartCollShape);
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

  //
  math::Pose relativePose = this->GetRelativePose();
  this->dartCollShape->setOffset(DARTUtils::ConvertVector3(relativePose.pos));
}

//////////////////////////////////////////////////
void DARTCollision::Fini()
{
  Collision::Fini();
}

//////////////////////////////////////////////////
void DARTCollision::SetCollision(bool _placeable)
{
  Collision::SetCollision(_placeable);

  // Nothing to do in dart.
}

//////////////////////////////////////////////////
void DARTCollision::OnPoseChange()
{
  // Nothing to do in dart.
}

//////////////////////////////////////////////////
void DARTCollision::SetCategoryBits(unsigned int /*_bits*/)
{
  // Nothing to do in dart.
}

//////////////////////////////////////////////////
void DARTCollision::SetCollideBits(unsigned int /*_bits*/)
{
  // Nothing to do in dart.
}

//////////////////////////////////////////////////
gazebo::math::Box DARTCollision::GetBoundingBox() const
{
  gazebo::math::Box box;

  gzwarn << "Not implemented!\n";

  return box;
}
