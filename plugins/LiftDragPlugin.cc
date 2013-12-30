/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/LiftDragPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)

/////////////////////////////////////////////////
LiftDragPlugin::LiftDragPlugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041)
{
  this->cp = math::Vector3(0, 0, 0);
  this->forward = math::Vector3(1, 0, 0);
  this->upward = math::Vector3(0, 0, 1);
  this->area = 1.0;
  this->a0 = 0.0;
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("a0"))
    this->a0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<math::Vector3>("cp");

  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<math::Vector3>("forward");

  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<math::Vector3>("upward");

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    this->linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(this->linkName);
  }
}

/////////////////////////////////////////////////
void LiftDragPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{
  // get linear velocity at cp in inertial frame
  math::Vector3 vel = this->link->GetWorldLinearVel(this->cp);

  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*vel + (1.0 - e)*velSmooth;
  // vel = this->velSmooth;

  if (vel.GetLength() <= 0.01)
    return;

  // pose of body
  math::Pose pose = this->link->GetWorldPose();

  // rotate forward and upward vectors into inertial frame
  math::Vector3 forwardI = pose.rot.RotateVector(this->forward);
  math::Vector3 upwardI = pose.rot.RotateVector(this->upward);

  // normal vector to lift-drag-plane described in inertial frame
  math::Vector3 normal = forwardI.Cross(upwardI).Normalize();

  // check sweep (angle between vel and lift-drag-plane)
  double sinSweepAngle = normal.Dot(vel) / vel.GetLength();

  // get cos from trig identity
  double cosSweepAngle2 = (1.0 - sinSweepAngle * sinSweepAngle);
  this->sweep = asin(sinSweepAngle);

  // angle of attack is the angle between
  // vel projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = normal Xcross ( vector Xcross normal)
  //
  // so,
  // velocity in lift-drag plane (expressed in inertial frame) is:
  math::Vector3 velInLDPlane = normal.Cross(vel.Cross(normal));

  // get direction of drag
  math::Vector3 dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  math::Vector3 liftDirection = normal.Cross(velInLDPlane);
  liftDirection.Normalize();

  // get direction of moment
  math::Vector3 momentDirection = normal;

  double cosAlpha = math::clamp(
    forwardI.Dot(velInLDPlane) /
    (forwardI.GetLength() * velInLDPlane.GetLength()), -1.0, 1.0);
  // gzerr << "ca " << forwardI.Dot(velInLDPlane) /
  //   (forwardI.GetLength() * velInLDPlane.GetLength()) << "\n";

  // get sign of alpha
  // take upwards component of velocity in lift-drag plane.
  // if sign == upward, then alpha is negative
  double alphaSign = -upwardI.Dot(velInLDPlane)/
    (upwardI.GetLength() + velInLDPlane.GetLength());

  // double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
  this->alpha = (alphaSign > 0.0) ? acos(cosAlpha) : -acos(cosAlpha);

  // normalize to within +/-90 deg
  while (abs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // HACK: get back magnitude for now
  // velInLDPlane *= vel.GetLength();

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.GetLength();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // lift at cp
  double cl = this->cla * (this->a0 + this->alpha) / cosSweepAngle2;
  math::Vector3 lift = cl * q * liftDirection;

  // drag at cp
  double cd = this->cda * (this->a0 + this->alpha) / cosSweepAngle2;
  math::Vector3 drag = cd * q * dragDirection;

  // moment about cp
  double cm = this->cma * (this->a0 + this->alpha) / cosSweepAngle2;
  math::Vector3 moment = cm * q * momentDirection;

  // moment arm from cg to cp in inertial plane
  math::Vector3 momentArm = pose.rot.RotateVector(
    this->link->GetInertial()->GetCoG() - this->cp);
  // gzerr << this->cp << " : " << this->link->GetInertial()->GetCoG() << "\n";

  // force an torque about cg in inertial frame
  math::Vector3 force = lift + drag + moment.Cross(momentArm);
  math::Vector3 torque = moment - lift.Cross(momentArm) - drag.Cross(momentArm);

  /* debug
  gzerr << "=============================\n";
  gzerr << "Link: " << this->link->GetName() << "\n";
  gzerr << "pose: [" << pose << "]\n";
  gzerr << "q: [" << q << "]\n";
  gzerr << "vel: [" << vel << "] |vel|: " << vel.GetLength() << "\n";
  gzerr << "velInPlane: [" << velInLDPlane
        << "] |vel|: " << velInLDPlane.GetLength() << "\n";
  gzerr << "forward: " << forwardI << "\n";
  gzerr << "upward: " << upwardI << "\n";
  gzerr << "normal: " << normal << "\n";
  gzerr << "sweep: " << this->sweep << "\n";
  // gzerr << "sin(sweep): " << sinSweepAngle << "\n";
  // gzerr << "cos(sweep): " << sqrt(cosSweepAngle2) << "\n";
  // gzerr << "|forward|: " << forwardI.GetLength() << "\n";
  // gzerr << "cos(alpha): " << cosAlpha << "\n";
  gzerr << "alpha: " << this->alpha << "\n";
  gzerr << "lift: " << lift << "\n";
  gzerr << "drag: " << drag << "\n";
  gzerr << "moment: " << moment << "\n";
  gzerr << "momentArm: " << momentArm << "\n";
  gzerr << "force: " << force << "\n";
  gzerr << "torque: " << torque << "\n";
  */

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  this->link->AddTorque(torque);
}
