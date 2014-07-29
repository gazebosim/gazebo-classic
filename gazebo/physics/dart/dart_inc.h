/*
 * Copyright 2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DART_INC_H_
#define _GAZEBO_DART_INC_H_

// This disables warning messages for ODE
#pragma GCC system_header

#include <dart/math/Helpers.h>
#include <dart/math/Geometry.h>

#include <dart/collision/CollisionDetector.h>
#include <dart/collision/CollisionNode.h>
#include <dart/collision/dart/DARTCollisionDetector.h>
#include <dart/collision/fcl_mesh/FCLMeshCollisionDetector.h>

#include <dart/integration/Integrator.h>
#include <dart/integration/EulerIntegrator.h>
#include <dart/integration/RK4Integrator.h>

#include <dart/dynamics/BallJoint.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/BoxShape.h>
#include <dart/dynamics/CylinderShape.h>
#include <dart/dynamics/EllipsoidShape.h>
#include <dart/dynamics/FreeJoint.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/MeshShape.h>
#include <dart/dynamics/PointMass.h>
#include <dart/dynamics/PrismaticJoint.h>
#include <dart/dynamics/RevoluteJoint.h>
#include <dart/dynamics/Shape.h>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/ScrewJoint.h>
#include <dart/dynamics/UniversalJoint.h>
#include <dart/dynamics/WeldJoint.h>
#include <dart/dynamics/SoftBodyNode.h>
#include <dart/dynamics/SoftMeshShape.h>

#include <dart/constraint/Constraint.h>
#include <dart/constraint/ConstraintSolver.h>
#include <dart/constraint/ContactConstraint.h>
#include <dart/constraint/JointLimitConstraint.h>
#include <dart/constraint/WeldJointConstraint.h>

#include <dart/simulation/World.h>

#endif
