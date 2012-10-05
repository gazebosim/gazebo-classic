/*
 * Copyright 2011 Nate Koenig
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

#include <vector>
#include <boost/shared_ptr.hpp>

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
    class Joint;
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
    class SphereShape;
    class MeshShape;
    class HeightmapShape;

    typedef boost::shared_ptr<Base> BasePtr;
    typedef boost::shared_ptr<Contact> ContactPtr;
    typedef boost::shared_ptr<Entity> EntityPtr;
    typedef boost::shared_ptr<World> WorldPtr;
    typedef boost::shared_ptr<Model> ModelPtr;
    typedef boost::shared_ptr<Actor> ActorPtr;
    typedef boost::shared_ptr<Link> LinkPtr;
    typedef boost::shared_ptr<Collision> CollisionPtr;
    typedef boost::shared_ptr<Joint> JointPtr;
    typedef boost::shared_ptr<PhysicsEngine> PhysicsEnginePtr;
    typedef boost::shared_ptr<Shape> ShapePtr;
    typedef boost::shared_ptr<RayShape> RayShapePtr;
    typedef boost::shared_ptr<HeightmapShape> HeightmapShapePtr;
    typedef boost::shared_ptr<MultiRayShape> MultiRayShapePtr;
    typedef boost::shared_ptr<Inertial> InertialPtr;
    typedef boost::shared_ptr<Road> RoadPtr;
    typedef boost::shared_ptr<SurfaceParams> SurfaceParamsPtr;

    typedef boost::shared_ptr<BoxShape> BoxShapePtr;
    typedef boost::shared_ptr<CylinderShape> CylinderShapePtr;
    typedef boost::shared_ptr<SphereShape> SphereShapePtr;
    typedef boost::shared_ptr<MeshShape> MeshShapePtr;

    typedef std::vector<BasePtr> Base_V;
    typedef std::vector<ModelPtr> Model_V;
    typedef std::vector<ActorPtr> Actor_V;
    typedef std::vector<JointPtr> Joint_V;
    typedef std::vector<LinkPtr>  Link_V;
    typedef std::vector<CollisionPtr>  Collision_V;

    #ifndef GZ_COLLIDE_BITS

    #define GZ_ALL_COLLIDE 0x0FFFFFFF
    #define GZ_NONE_COLLIDE 0x00000000
    #define GZ_FIXED_COLLIDE 0x00000001
    #define GZ_SENSOR_COLLIDE 0x00000003
    #define GZ_GHOST_COLLIDE 0x10000000

    #endif
  }
}


