/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _PHYSICSTYPES_HH_
#define _PHYSICSTYPES_HH_

#include <vector>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include "gazebo/util/system.hh"

/// \file
/// \ingroup gazebo_physics
/// \brief default namespace for gazebo
namespace gazebo
{
  /// \brief Physics forward declarations and type defines
  namespace physics
  {
    class Base;
    class Entity;
    class World;
    class Model;
    class Actor;
    class Link;
    class Collision;
    class FrictionPyramid;
    class Gripper;
    class Joint;
    class JointController;
    class Contact;
    class PhysicsEngine;
    class Mass;
    class Road;
    class Shape;
    class RayShape;
    class MultiRayShape;
    class Inertial;
    class SurfaceParams;
    class BoxShape;
    class CylinderShape;
    class MeshShape;
    class SphereShape;
    class PlaneShape;
    class HeightmapShape;
    class PolylineShape;
    class ModelState;
    class LinkState;
    class JointState;

    /// \def BasePtr
    /// \brief Boost shared pointer to a Base object
    typedef boost::shared_ptr<Base> BasePtr;

    /// \def ContactPtr
    /// \brief Boost shared pointer to a Contact object
    typedef boost::shared_ptr<Contact> ContactPtr;

    /// \def EntityPtr
    /// \brief Boost shared pointer to an Entity object
    typedef boost::shared_ptr<Entity> EntityPtr;

    /// \def WorldPtr
    /// \brief Boost shared pointer to a World object
    typedef boost::shared_ptr<World> WorldPtr;

    /// \def ModelPtr
    /// \brief Boost shared pointer to a Model object
    typedef boost::shared_ptr<Model> ModelPtr;

    /// \def Actor
    /// \brief Boost shared pointer to an Actor object
    typedef boost::shared_ptr<Actor> ActorPtr;

    /// \def LinkPtr
    /// \brief Boost shared pointer to a Link object
    typedef boost::shared_ptr<Link> LinkPtr;

    /// \def CollisionPtr
    /// \brief Boost shared pointer to a Collision object
    typedef boost::shared_ptr<Collision> CollisionPtr;

    /// \def JointPtr
    /// \brief Boost shared pointer to a Joint object
    typedef boost::shared_ptr<Joint> JointPtr;

    /// \def JointControllerPtr
    /// \brief Boost shared pointer to a JointController object
    typedef boost::shared_ptr<JointController> JointControllerPtr;

    /// \def  PhysicsEnginePtr
    /// \brief Boost shared pointer to a PhysicsEngine object
    typedef boost::shared_ptr<PhysicsEngine> PhysicsEnginePtr;

    /// \def ShapePtr
    /// \brief Boost shared pointer to a Shape object
    typedef boost::shared_ptr<Shape> ShapePtr;

    /// \def RayShapePtr
    /// \brief Boost shared pointer to a RayShape object
    typedef boost::shared_ptr<RayShape> RayShapePtr;

    /// \def HeightmapShapePtr
    /// \brief Boost shared pointer to a HeightmapShape object
    typedef boost::shared_ptr<HeightmapShape> HeightmapShapePtr;

    /// \def MultiRayShapePtr
    /// \brief Boost shared pointer to a MultiRayShape object
    typedef boost::shared_ptr<MultiRayShape> MultiRayShapePtr;

    /// \def InertialPtr
    /// \brief Boost shared pointer to a Inertial object
    typedef boost::shared_ptr<Inertial> InertialPtr;

    /// \def RoadPtr
    /// \brief Boost shared pointer to a Road object
    typedef boost::shared_ptr<Road> RoadPtr;

    /// \def FrictionPyramidPtr
    /// \brief Boost shared pointer to a FrictionPyramid object
    typedef boost::shared_ptr<FrictionPyramid> FrictionPyramidPtr;

    /// \def SurfaceParamsPtr
    /// \brief Boost shared pointer to a SurfaceParams object
    typedef boost::shared_ptr<SurfaceParams> SurfaceParamsPtr;

    /// \def BoxShapePtr
    /// \brief Boost shared pointer to a BoxShape object
    typedef boost::shared_ptr<BoxShape> BoxShapePtr;

    /// \def CylinderShapePtr
    /// \brief Boost shared pointer to a CylinderShape object
    typedef boost::shared_ptr<CylinderShape> CylinderShapePtr;

    /// \def PlaneShapePtr
    /// \brief Boost shared pointer to a PlaneShape object
    typedef boost::shared_ptr<PlaneShape> PlaneShapePtr;

    /// \def MeshShapePtr
    /// \brief Boost shared pointer to a MeshShape object
    typedef boost::shared_ptr<MeshShape> MeshShapePtr;

    /// \def PolylineShapePtr
    /// \brief Boost shared pointer to a Polyline shape object
    typedef boost::shared_ptr<PolylineShape> PolylineShapePtr;

    /// \def SphereShapePtr
    /// \brief Boost shared pointer to a SphereShape object
    typedef boost::shared_ptr<SphereShape> SphereShapePtr;

    /// \def GripperPtr
    /// \brief Boost shared pointer to a Gripper object
    typedef boost::shared_ptr<Gripper> GripperPtr;

    /// \def Base_V
    /// \brief Vector of BasePtr
    typedef std::vector<BasePtr> Base_V;

    /// \def Model_V
    /// \brief Vector of ModelPtr
    typedef std::vector<ModelPtr> Model_V;

    /// \def Actor_V
    /// \brief Vector of ActorPtr
    typedef std::vector<ActorPtr> Actor_V;

    /// \def Joint_V
    /// \brief Vector of JointPtr
    typedef std::vector<JointPtr> Joint_V;

    /// \def JointController_V
    /// \brief Vector of JointControllerPtr
    typedef std::vector<JointControllerPtr> JointController_V;

    /// \def Link_V
    /// \brief Vector of LinkPtr
    typedef std::vector<LinkPtr>  Link_V;

    /// \def Collision_V
    /// \brief Vector of CollisionPtr
    typedef std::vector<CollisionPtr>  Collision_V;

    /// \def ModelState_M
    /// \brief Map of model state
    typedef std::map<std::string, ModelState> ModelState_M;

    /// \def LinkState_M
    /// \brief Map of link state
    typedef std::map<std::string, LinkState> LinkState_M;

    /// \def JointState_M
    /// \brief Map of joint state
    typedef std::map<std::string, JointState> JointState_M;

    #ifndef GZ_COLLIDE_BITS

    /// \def GZ_ALL_COLLIDE
    /// \brief Default collision bitmask. Collision objects will collide
    /// with everything.
    #define GZ_ALL_COLLIDE 0x0FFFFFFF

    /// \def GZ_NONE_COLLIDE
    /// \brief Collision object will collide with nothing.
    #define GZ_NONE_COLLIDE 0x00000000

    /// \def GZ_FIXED_COLLIDE
    /// \brief Collision object will collide only with fixed objects.
    #define GZ_FIXED_COLLIDE 0x00000001

    /// \def GZ_SENSOR_COLLIDE
    /// \brief Collision object will collide only with sensors
    #define GZ_SENSOR_COLLIDE 0x00000002

    /// \def GZ_GHOST_COLLIDE
    /// \brief Collides with everything else but other ghost.
    #define GZ_GHOST_COLLIDE 0x10000000

    #endif
  }
}
#endif
