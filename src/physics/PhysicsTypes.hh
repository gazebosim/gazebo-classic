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

#include <vector>
#include <boost/shared_ptr.hpp>

namespace gazebo
{
  namespace physics
  {
    class Base;
    class Entity;
    class World;
    class Model;
    class Body;
    class Geom;
    class Joint;
    class Contact;
    class PhysicsEngine;
    class Mass;
    class SurfaceParams;
    class Shape;
    class RayShape;
    class Inertial;

    typedef boost::shared_ptr<Base> BasePtr;
    typedef boost::shared_ptr<Entity> EntityPtr;
    typedef boost::shared_ptr<World> WorldPtr;
    typedef boost::shared_ptr<Model> ModelPtr;
    typedef boost::shared_ptr<Body> BodyPtr;
    typedef boost::shared_ptr<Geom> GeomPtr;
    typedef boost::shared_ptr<Joint> JointPtr;
    typedef boost::shared_ptr<PhysicsEngine> PhysicsEnginePtr;
    typedef boost::shared_ptr<SurfaceParams> SurfaceParamsPtr;
    typedef boost::shared_ptr<Shape> ShapePtr;
    typedef boost::shared_ptr<RayShape> RayShapePtr;
    typedef boost::shared_ptr<Inertial> InertialPtr;

    typedef std::vector<BasePtr> Base_V;
    typedef std::vector<ModelPtr> Model_V;
    typedef std::vector<JointPtr> Joint_V;
    typedef std::vector<BodyPtr>  Body_V;
    typedef std::vector<GeomPtr>  Geom_V;

    #ifndef GZ_COLLIDE_BITS
    
    #define GZ_ALL_COLLIDE 0x0FFFFFFF
    #define GZ_NONE_COLLIDE 0x00000000
    #define GZ_FIXED_COLLIDE 0x00000001
    #define GZ_SENSOR_COLLIDE 0x00000002
    #define GZ_GHOST_COLLIDE 0x10000000
    
    #endif
  }
}
