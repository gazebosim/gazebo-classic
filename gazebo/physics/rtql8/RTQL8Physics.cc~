// #include "physics/bullet/BulletTypes.hh"
// #include "physics/bullet/BulletLink.hh"
// #include "physics/bullet/BulletCollision.hh"
// 
// #include "physics/bullet/BulletPlaneShape.hh"
// #include "physics/bullet/BulletSphereShape.hh"
// #include "physics/bullet/BulletHeightmapShape.hh"
// #include "physics/bullet/BulletMultiRayShape.hh"
// #include "physics/bullet/BulletBoxShape.hh"
// #include "physics/bullet/BulletCylinderShape.hh"
// #include "physics/bullet/BulletTrimeshShape.hh"
// #include "physics/bullet/BulletRayShape.hh"
// 
// #include "physics/bullet/BulletHingeJoint.hh"
// #include "physics/bullet/BulletUniversalJoint.hh"
// #include "physics/bullet/BulletBallJoint.hh"
// #include "physics/bullet/BulletSliderJoint.hh"
// #include "physics/bullet/BulletHinge2Joint.hh"
// #include "physics/bullet/BulletScrewJoint.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/World.hh"
#include "physics/Entity.hh"
#include "physics/Model.hh"
#include "physics/SurfaceParams.hh"
#include "physics/Collision.hh"
#include "physics/MapShape.hh"

#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Vector3.hh"

#include "RTQL8Physics.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("rtql8", RTQL8Physics)

// extern ContactAddedCallback gContactAddedCallback;
// extern ContactProcessedCallback gContactProcessedCallback;
// 
// //////////////////////////////////////////////////
// bool ContactCallback(btManifoldPoint &/*_cp*/,
//     const btCollisionObjectWrapper * /*_obj0*/, int /*_partId0*/,
//     int /*_index0*/, const btCollisionObjectWrapper * /*_obj1*/,
//     int /*_partId1*/, int /*_index1*/)
// {
//   return true;
// }
// 
// //////////////////////////////////////////////////
// bool ContactProcessed(btManifoldPoint &/*_cp*/, void * /*_body0*/,
//                       void * /*_body1*/)
// {
//   return true;
// }
// 
//////////////////////////////////////////////////
RTQL8Physics::RTQL8Physics(WorldPtr _world)
    : PhysicsEngine(_world)
{
//   // Create the dynamics solver
//   this->solver = new btSequentialImpulseConstraintSolver;
// 
//   // Instantiate the world
//   this->dynamicsWorld = new btDiscreteDynamicsWorld(this->dispatcher,
//       this->broadPhase, this->solver, this->collisionConfig);

  this->world = new simulation::World;
}

//////////////////////////////////////////////////
RTQL8Physics::~RTQL8Physics()
{
//   delete this->broadPhase;
//   delete this->collisionConfig;
//   delete this->dispatcher;
//   delete this->solver;
// 
//   // TODO: Fix this line
//   // delete this->dynamicsWorld;
// 
//   this->broadPhase = NULL;
//   this->collisionConfig = NULL;
//   this->dispatcher = NULL;
//   this->solver = NULL;
//   this->dynamicsWorld = NULL;

  delete this->world;
  
  this->world = NULL;
}

//////////////////////////////////////////////////
void RTQL8Physics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  // Gravity
  math::Vector3 g = this->sdf->GetValueVector3("gravity");
  this->world->setGravity(Eigen::Vector3d(g.x, g.y, g.z));
  
  // Time step
  this->world->setTimeStep(this->sdf->GetValueDouble("time_step"));
  
  // TODO: Elements for rtql8 settings
  sdf::ElementPtr rtql8Elem = this->sdf->GetElement("rtql8");
  //this->stepTimeDouble = rtql8Elem->GetElement("dt")->GetValueDouble();
  
}
 
//////////////////////////////////////////////////
void RTQL8Physics::Init()
{
  this->world->init();
}

//////////////////////////////////////////////////
void RTQL8Physics::Fini()
{
}

//////////////////////////////////////////////////
void RTQL8Physics::Reset()
{
}

//////////////////////////////////////////////////
void RTQL8Physics::InitForThread()
{
}
 
//////////////////////////////////////////////////
void RTQL8Physics::UpdateCollision()
{
}

//////////////////////////////////////////////////
void RTQL8Physics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  this->physicsUpdateMutex->lock();

  //common::Time currTime =  this->world->GetRealTime();
  this->world->updatePhysics();
  //this->lastUpdateTime = currTime;

  this->physicsUpdateMutex->unlock();
}

//////////////////////////////////////////////////
void RTQL8Physics::SetStepTime(double _value)
{
   // TODO: element of dt
//   this->sdf->GetElement("ode")->GetElement(
//       "solver")->GetAttribute("dt")->Set(_value);
   this->stepTimeDouble = _value;
   this->world->setTimeStep(_value);
}

//////////////////////////////////////////////////
double RTQL8Physics::GetStepTime()
{
  return this->stepTimeDouble;
  //return this->world->getTimeStep();
}

//////////////////////////////////////////////////
LinkPtr RTQL8Physics::CreateLink(ModelPtr _parent)
{
  LinkPtr link;
//   if (_parent == NULL)
//     gzthrow("Link must have a parent\n");
// 
//   BulletLinkPtr link(new BulletLink(_parent));
//   link->SetWorld(_parent->GetWorld());
// 
//   return link;
  return link;
}

//////////////////////////////////////////////////
CollisionPtr RTQL8Physics::CreateCollision(const std::string &_type,
                                            LinkPtr _parent)
{
  CollisionPtr collision;
//   BulletCollisionPtr collision(new BulletCollision(_parent));
//   ShapePtr shape = this->CreateShape(_type, collision);
//   collision->SetShape(shape);
//   shape->SetWorld(_parent->GetWorld());
//   return collision;
  return collision;
}

//////////////////////////////////////////////////
ShapePtr RTQL8Physics::CreateShape(const std::string &_type,
                                    CollisionPtr _collision)
{
  ShapePtr shape;
//   BulletCollisionPtr collision =
//     boost::shared_dynamic_cast<BulletCollision>(_collision);
// 
//   if (_type == "plane")
//     shape.reset(new BulletPlaneShape(collision));
//   else if (_type == "sphere")
//     shape.reset(new BulletSphereShape(collision));
//   else if (_type == "box")
//     shape.reset(new BulletBoxShape(collision));
//   else if (_type == "cylinder")
//     shape.reset(new BulletCylinderShape(collision));
//   else if (_type == "mesh" || _type == "trimesh")
//     shape.reset(new BulletTrimeshShape(collision));
//   else if (_type == "heightmap")
//     shape.reset(new BulletHeightmapShape(collision));
//   else if (_type == "multiray")
//     shape.reset(new BulletMultiRayShape(collision));
//   else if (_type == "ray")
//     if (_collision)
//       shape.reset(new BulletRayShape(_collision));
//     else
//       shape.reset(new BulletRayShape(this->world->GetPhysicsEngine()));
//   else
//     gzerr << "Unable to create collision of type[" << _type << "]\n";
// 
//   /*
//   else if (_type == "map" || _type == "image")
//     shape.reset(new MapShape(collision));
//     */
  return shape;
}

//////////////////////////////////////////////////
JointPtr RTQL8Physics::CreateJoint(const std::string &_type, ModelPtr _parent)
{
  JointPtr joint;

//   if (_type == "revolute")
//     joint.reset(new BulletHingeJoint(this->dynamicsWorld, _parent));
//   else if (_type == "universal")
//     joint.reset(new BulletUniversalJoint(this->dynamicsWorld, _parent));
//   else if (_type == "ball")
//     joint.reset(new BulletBallJoint(this->dynamicsWorld, _parent));
//   else if (_type == "prismatic")
//     joint.reset(new BulletSliderJoint(this->dynamicsWorld, _parent));
//   else if (_type == "revolute2")
//     joint.reset(new BulletHinge2Joint(this->dynamicsWorld, _parent));
//   else if (_type == "screw")
//     joint.reset(new BulletScrewJoint(this->dynamicsWorld, _parent));
//   else
//     gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

// //////////////////////////////////////////////////
// void RTQL8Physics::ConvertMass(InertialPtr /*_inertial*/,
//                                 void * /*_engineMass*/)
// {
// }
// 
// //////////////////////////////////////////////////
// void RTQL8Physics::ConvertMass(void * /*_engineMass*/,
//                                 const InertialPtr /*_inertial*/)
// {
// }
// 
// //////////////////////////////////////////////////
// math::Pose RTQL8Physics::ConvertPose(const btTransform &_bt)
// {
//   math::Pose pose;
//   pose.pos.x = _bt.getOrigin().getX();
//   pose.pos.y = _bt.getOrigin().getY();
//   pose.pos.z = _bt.getOrigin().getZ();
// 
//   pose.rot.w = _bt.getRotation().getW();
//   pose.rot.x = _bt.getRotation().getX();
//   pose.rot.y = _bt.getRotation().getY();
//   pose.rot.z = _bt.getRotation().getZ();
// 
//   return pose;
// }
// 
// //////////////////////////////////////////////////
// btTransform RTQL8Physics::ConvertPose(const math::Pose &_pose)
// {
//   btTransform trans;
// 
//   trans.setOrigin(btVector3(_pose.pos.x, _pose.pos.y, _pose.pos.z));
//   trans.setRotation(btQuaternion(_pose.rot.x, _pose.rot.y,
//                                  _pose.rot.z, _pose.rot.w));
//   return trans;
// }

//////////////////////////////////////////////////
void RTQL8Physics::SetGravity(const gazebo::math::Vector3& _gravity)
{
  this->sdf->GetElement("gravity")->GetAttribute("xyz")->Set(_gravity);
  this->world->setGravity(Eigen::Vector3d(_gravity.x, _gravity.y, _gravity.z));
}

//////////////////////////////////////////////////
void RTQL8Physics::DebugPrint() const
{
//   dBodyID b;
//   std::cout << "Debug Print[" << dWorldGetBodyCount(this->worldId) << "]\n";
//   for (int i = 0; i < dWorldGetBodyCount(this->worldId); ++i)
//   {
//     b = dWorldGetBody(this->worldId, i);
//     ODELink *link = static_cast<ODELink*>(dBodyGetData(b));
//     math::Pose pose = link->GetWorldPose();
//     const dReal *pos = dBodyGetPosition(b);
//     const dReal *rot = dBodyGetRotation(b);
//     math::Vector3 dpos(pos[0], pos[1], pos[2]);
//     math::Quaternion drot(rot[0], rot[1], rot[2], rot[3]);
// 
//     std::cout << "Body[" << link->GetScopedName() << "]\n";
//     std::cout << "  World: Pos[" << dpos << "] Rot[" << drot << "]\n";
//     if (pose.pos != dpos)
//       std::cout << "    Incorrect world pos[" << pose.pos << "]\n";
//     if (pose.rot != drot)
//       std::cout << "    Incorrect world rot[" << pose.rot << "]\n";
// 
//     dMass mass;
//     dBodyGetMass(b, &mass);
//     std::cout << "  Mass[" << mass.mass << "] COG[" << mass.c[0]
//               << " " << mass.c[1] << " " << mass.c[2] << "]\n";
// 
//     dGeomID g = dBodyGetFirstGeom(b);
//     while (g)
//     {
//       ODECollision *coll = static_cast<ODECollision*>(dGeomGetData(g));
// 
//       pose = coll->GetWorldPose();
//       const dReal *gpos = dGeomGetPosition(g);
//       const dReal *grot = dGeomGetRotation(g);
//       dpos.Set(gpos[0], gpos[1], gpos[2]);
//       drot.Set(grot[0], grot[1], grot[2], grot[3]);
// 
//       std::cout << "    Geom[" << coll->GetScopedName() << "]\n";
//       std::cout << "      World: Pos[" << dpos << "] Rot[" << drot << "]\n";
// 
//       if (pose.pos != dpos)
//         std::cout << "      Incorrect world pos[" << pose.pos << "]\n";
//       if (pose.rot != drot)
//         std::cout << "      Incorrect world rot[" << pose.rot << "]\n";
// 
//       g = dBodyGetNextGeom(g);
//     }
//   }
}


