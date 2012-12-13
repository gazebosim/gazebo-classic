
#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/World.hh"

#include "physics/rtql8/rtql8_inc.h"
//#include "physics/bullet/BulletCollision.hh"
//#include "physics/bullet/BulletMotionState.hh"
#include "physics/rtql8/RTQL8Physics.hh"
#include "physics/rtql8/RTQL8Link.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Link::RTQL8Link(EntityPtr _parent)
    : Link(_parent)
{
  //TODO:
  //this->rigidLink = NULL;
  //this->compoundShape = new btCompoundShape();
  //this->motionState = new BulletMotionState(this);
}

//////////////////////////////////////////////////
RTQL8Link::~RTQL8Link()
{
  //TODO:
}

void RTQL8Link::Load(sdf::ElementPtr _sdf)
{
  this->rtql8Physics = boost::shared_dynamic_cast<RTQL8Physics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->rtql8Physics == NULL)
    gzthrow("Not using the rtql8 physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void RTQL8Link::Init()
{
  Link::Init();
  
  this->rtql8BodyNode = new kinematics::BodyNode();
}

//////////////////////////////////////////////////
void RTQL8Link::Fini()
{
  delete rtql8BodyNode;
  
  Link::Fini();
}

//////////////////////////////////////////////////
void RTQL8Link::Update()
{
  Link::Update();
}

// //////////////////////////////////////////////////
// void BulletLink::SetGravityMode(bool _mode)
// {
//   if (!this->rigidLink)
//     return;
// 
//   if (_mode == false)
//     this->rigidLink->setGravity(btVector3(0, 0, 0));
//     // this->rigidLink->setMassProps(btScalar(0), btmath::Vector3(0, 0, 0));
//   else
//   {
//     math::Vector3 g = this->bulletPhysics->GetGravity();
//     this->rigidLink->setGravity(btVector3(g.x, g.y, g.z));
//     /*btScalar btMass = this->mass.GetAsDouble();
//     btmath::Vector3 fallInertia(0, 0, 0);
// 
//     this->compoundShape->calculateLocalInertia(btMass, fallInertia);
//     this->rigidLink->setMassProps(btMass, fallInertia);
//     */
//   }
// }
// 
// //////////////////////////////////////////////////
// bool BulletLink::GetGravityMode()
// {
//   bool result = false;
//   if (this->rigidLink)
//   {
//     btVector3 g = this->rigidLink->getGravity();
//     result = !math::equal(static_cast<double>(g.length()), 0.0);
//   }
// 
//   return result;
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetSelfCollide(bool /*_collide*/)
// {
// }
// 
// //////////////////////////////////////////////////
// /*void BulletLink::AttachCollision(Collision *_collision)
// {
//   Link::AttachCollision(_collision);
// 
//   BulletCollision *bcollision = dynamic_cast<BulletCollision*>(_collision);
// 
//   if (_collision == NULL)
//     gzthrow("requires BulletCollision");
// 
//   btTransform trans;
//   math::Pose relativePose = _collision->GetRelativePose();
//   trans = BulletPhysics::ConvertPose(relativePose);
// 
//   bcollision->SetCompoundShapeIndex(this->compoundShape->getNumChildShapes());
//   this->compoundShape->addChildShape(trans, bcollision->GetCollisionShape());
// }
//   */
// 
// //////////////////////////////////////////////////
// /// changed
// void BulletLink::OnPoseChange()
// {
//   /*
//   math::Pose pose = this->GetWorldPose();
// 
//   this->motionState->SetWorldPose(pose);
//   if (this->rigidLink)
//     this->rigidLink->setMotionState(this->motionState);
//     */
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetEnabled(bool /*_enable*/) const
// {
//   /*
//   if (!this->rigidLink)
//     return;
// 
//   if (_enable)
//     this->rigidLink->activate(true);
//   else
//     this->rigidLink->setActivationState(WANTS_DEACTIVATION);
//     */
// }
// 
// /////////////////////////////////////////////////////////////////////
// /*
//   What's going on here?  In ODE the CoM of a body corresponds to the
//   origin of the body-fixed coordinate system.  In Gazebo, however, we
//   want to have arbitrary body coordinate systems (i.e., CoM may be
//   displaced from the body-fixed cs).  To get around this limitation in
//   Bullet, we have an extra fudge-factor (comPose), describing the pose of
//   the CoM relative to Gazebo's body-fixed cs.  When using low-level
//   Bullet functions, one must use apply this factor appropriately.
// 
//   The UpdateCoM() function is used to compute this offset, based on
//   the mass distribution of attached collisions.  This function also shifts
//   the Bullet-pose of the collisions, to keep everything in the same place in
//   the Gazebo cs.  Simple, neh?
// */
// void BulletLink::UpdateCoM()
// {
//   // Link::UpdateCoM();
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetLinearVel(const math::Vector3 & /*_vel*/)
// {
//   /*
//   if (!this->rigidLink)
//     return;
// 
//   this->rigidLink->setLinearVelocity(btmath::Vector3(_vel.x, _vel.y, _vel.z));
//   */
// }
// 
// //////////////////////////////////////////////////
// math::Vector3 BulletLink::GetWorldLinearVel() const
// {
//   /*
//   if (!this->rigidLink)
//     return math::Vector3(0, 0, 0);
// 
//   btmath::Vector3 btVec = this->rigidLink->getLinearVelocity();
// 
//   return math::Vector3(btVec.x(), btVec.y(), btVec.z());
//   */
//   return math::Vector3();
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetAngularVel(const math::Vector3 &_vel)
// {
//   if (!this->rigidLink)
//     return;
// 
//   this->rigidLink->setAngularVelocity(btVector3(_vel.x, _vel.y, _vel.z));
// }
// 
// //////////////////////////////////////////////////
// math::Vector3 BulletLink::GetWorldAngularVel() const
// {
//   if (!this->rigidLink)
//     return math::Vector3(0, 0, 0);
// 
//   btVector3 btVec = this->rigidLink->getAngularVelocity();
// 
//   return math::Vector3(btVec.x(), btVec.y(), btVec.z());
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetForce(const math::Vector3 &/*_force*/)
// {
//   /*
//   if (!this->rigidLink)
//     return;
// 
//   this->rigidLink->applyCentralForce(
//       btmath::Vector3(_force.x, _force.y, _force.z));
//       */
// }
// 
// //////////////////////////////////////////////////
// math::Vector3 BulletLink::GetWorldForce() const
// {
//   /*
//   if (!this->rigidLink)
//     return math::Vector3(0, 0, 0);
// 
//   btmath::Vector3 btVec;
// 
//   btVec = this->rigidLink->getTotalForce();
// 
//   return math::Vector3(btVec.x(), btVec.y(), btVec.z());
//   */
//   return math::Vector3();
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetTorque(const math::Vector3 &_torque)
// {
//   if (!this->rigidLink)
//     return;
// 
//   this->rigidLink->applyTorque(btVector3(_torque.x, _torque.y, _torque.z));
// }
// 
// //////////////////////////////////////////////////
// math::Vector3 BulletLink::GetWorldTorque() const
// {
//   /*
//   if (!this->rigidLink)
//     return math::Vector3(0, 0, 0);
// 
//   btmath::Vector3 btVec;
// 
//   btVec = this->rigidLink->getTotalTorque();
// 
//   return math::Vector3(btVec.x(), btVec.y(), btVec.z());
//   */
//   return math::Vector3();
// }
// 
// //////////////////////////////////////////////////
// btRigidBody *BulletLink::GetBulletLink() const
// {
//   return this->rigidLink;
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetLinearDamping(double _damping)
// {
//   if (this->rigidLink)
//     this->rigidLink->setDamping((btScalar)_damping,
//         (btScalar)this->rigidLink->getAngularDamping());
// }
// 
// //////////////////////////////////////////////////
// void BulletLink::SetAngularDamping(double _damping)
// {
//   if (this->rigidLink)
//     this->rigidLink->setDamping(
//         (btScalar)this->rigidLink->getLinearDamping(), (btScalar)_damping);
// }
// 
// //////////////////////////////////////////////////
// /*void BulletLink::SetCollisionRelativePose(BulletCollision *_collision,
//     const math::Pose &_newPose)
// {
//   std::map<std::string, Collision*>::iterator iter;
//   unsigned int i;
// 
//   for (iter = this->collisions.begin(), i = 0; iter != this->collisions.end();
//        ++iter, ++i)
//   {
//     if (iter->second == _collision)
//       break;
//   }
// 
//   if (i < this->collisions.size())
//   {
//     // Set the pose of the _collision in Bullet
//     this->compoundShape->updateChildTransform(i,
//         BulletPhysics::ConvertPose(_newPose));
//   }
// }*/
// 
// /////////////////////////////////////////////////
// void BulletLink::AddForce(const math::Vector3 &/*_force*/)
// {
// }
// 
// /////////////////////////////////////////////////
// void BulletLink::AddRelativeForce(const math::Vector3 &/*_force*/)
// {
// }
// 
// /////////////////////////////////////////////////
// void BulletLink::AddForceAtWorldPosition(const math::Vector3 &/*_force*/,
//                                          const math::Vector3 &/*_pos*/)
// {
// }
// 
// /////////////////////////////////////////////////
// void BulletLink::AddForceAtRelativePosition(const math::Vector3 &/*_force*/,
//                   const math::Vector3 &/*_relpos*/)
// {
// }
// 
// /////////////////////////////////////////////////
// void BulletLink::AddTorque(const math::Vector3 &/*_torque*/)
// {
// }
// 
// /////////////////////////////////////////////////
// void BulletLink::AddRelativeTorque(const math::Vector3 &/*_torque*/)
// {
// }
// 
// /////////////////////////////////////////////////
// void BulletLink::SetAutoDisable(bool /*_disable*/)
// {
// }
