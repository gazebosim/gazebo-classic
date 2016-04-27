/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_PHYSICSTYPES_HH_
#define GAZEBO_PHYSICS_PHYSICSTYPES_HH_

#include <map>
#include <string>
#include <vector>
#include <memory>

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
    class Light;
    class Link;
    class Collision;
    class FrictionPyramid;
    class Gripper;
    class Joint;
    class JointController;
    class Contact;
    class PresetManager;
    class UserCmd;
    class UserCmdManager;
    class PhysicsEngine;
    class Wind;
    class Atmosphere;
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
    class LightState;
    class LinkState;
    class JointState;
    class TrajectoryInfo;

    /// \def BasePtr
    /// \brief Shared pointer to a Base object
    typedef std::shared_ptr<Base> BasePtr;

    /// \def ContactPtr
    /// \brief Shared pointer to a Contact object
    typedef std::shared_ptr<Contact> ContactPtr;

    /// \def EntityPtr
    /// \brief Shared pointer to an Entity object
    typedef std::shared_ptr<Entity> EntityPtr;

    /// \def WorldPtr
    /// \brief Shared pointer to a World object
    typedef std::shared_ptr<World> WorldPtr;

    /// \def ModelPtr
    /// \brief Shared pointer to a Model object
    typedef std::shared_ptr<Model> ModelPtr;

    /// \def Actor
    /// \brief Shared pointer to an Actor object
    typedef std::shared_ptr<Actor> ActorPtr;

    /// \def TrajectoryInfoPtr
    /// \brief Shared pointer to a TrajectoryInfo object
    typedef std::shared_ptr<TrajectoryInfo> TrajectoryInfoPtr;

    /// \def LightPtr
    /// \brief Shared pointer to a Light object
    typedef std::shared_ptr<Light> LightPtr;

    /// \def LinkPtr
    /// \brief Shared pointer to a Link object
    typedef std::shared_ptr<Link> LinkPtr;

    /// \def CollisionPtr
    /// \brief Shared pointer to a Collision object
    typedef std::shared_ptr<Collision> CollisionPtr;

    /// \def JointPtr
    /// \brief Shared pointer to a Joint object
    typedef std::shared_ptr<Joint> JointPtr;

    /// \def JointControllerPtr
    /// \brief Shared pointer to a JointController object
    typedef std::shared_ptr<JointController> JointControllerPtr;

    /// \def  PhysicsEnginePtr
    /// \brief Shared pointer to a PhysicsEngine object
    typedef std::shared_ptr<PhysicsEngine> PhysicsEnginePtr;

    /// \def AtmospherePtr
    /// \brief Standard shared pointer to an Atmosphere object
    typedef std::shared_ptr<Atmosphere> AtmospherePtr;

    /// \def  PresetManagerPtr
    /// \brief Shared pointer to a PresetManager object
    typedef std::shared_ptr<PresetManager> PresetManagerPtr;

    /// \def  UserCmdPtr
    /// \brief Shared pointer to a UserCmd object
    typedef std::shared_ptr<UserCmd> UserCmdPtr;

    /// \def  UserCmdManagerPtr
    /// \brief Shared pointer to a UserCmdManager object
    typedef std::shared_ptr<UserCmdManager> UserCmdManagerPtr;

    /// \def ShapePtr
    /// \brief Shared pointer to a Shape object
    typedef std::shared_ptr<Shape> ShapePtr;

    /// \def RayShapePtr
    /// \brief Shared pointer to a RayShape object
    typedef std::shared_ptr<RayShape> RayShapePtr;

    /// \def HeightmapShapePtr
    /// \brief Shared pointer to a HeightmapShape object
    typedef std::shared_ptr<HeightmapShape> HeightmapShapePtr;

    /// \def MultiRayShapePtr
    /// \brief Shared pointer to a MultiRayShape object
    typedef std::shared_ptr<MultiRayShape> MultiRayShapePtr;

    /// \def InertialPtr
    /// \brief Shared pointer to a Inertial object
    typedef std::shared_ptr<Inertial> InertialPtr;

    /// \def RoadPtr
    /// \brief Shared pointer to a Road object
    typedef std::shared_ptr<Road> RoadPtr;

    /// \def FrictionPyramidPtr
    /// \brief Shared pointer to a FrictionPyramid object
    typedef std::shared_ptr<FrictionPyramid> FrictionPyramidPtr;

    /// \def SurfaceParamsPtr
    /// \brief Shared pointer to a SurfaceParams object
    typedef std::shared_ptr<SurfaceParams> SurfaceParamsPtr;

    /// \def BoxShapePtr
    /// \brief Shared pointer to a BoxShape object
    typedef std::shared_ptr<BoxShape> BoxShapePtr;

    /// \def CylinderShapePtr
    /// \brief Shared pointer to a CylinderShape object
    typedef std::shared_ptr<CylinderShape> CylinderShapePtr;

    /// \def PlaneShapePtr
    /// \brief Shared pointer to a PlaneShape object
    typedef std::shared_ptr<PlaneShape> PlaneShapePtr;

    /// \def MeshShapePtr
    /// \brief Shared pointer to a MeshShape object
    typedef std::shared_ptr<MeshShape> MeshShapePtr;

    /// \def PolylineShapePtr
    /// \brief Shared pointer to a Polyline shape object
    typedef std::shared_ptr<PolylineShape> PolylineShapePtr;

    /// \def SphereShapePtr
    /// \brief Shared pointer to a SphereShape object
    typedef std::shared_ptr<SphereShape> SphereShapePtr;

    /// \def GripperPtr
    /// \brief Shared pointer to a Gripper object
    typedef std::shared_ptr<Gripper> GripperPtr;

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

    /// \def Light_V
    /// \brief Vector of LightPtr
    typedef std::vector<LightPtr>  Light_V;

    /// \def Link_V
    /// \brief Vector of LinkPtr
    typedef std::vector<LinkPtr>  Link_V;

    /// \def Collision_V
    /// \brief Vector of CollisionPtr
    typedef std::vector<CollisionPtr>  Collision_V;

    /// \def ModelState_M
    /// \brief Map of model state
    typedef std::map<std::string, ModelState> ModelState_M;

    /// \def LightState_M
    /// \brief Map of light state
    typedef std::map<std::string, LightState> LightState_M;

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
