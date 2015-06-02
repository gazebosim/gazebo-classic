/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <algorithm>
#include <string>

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
  this->alpha0 = 0.0;
  this->alpha = 0.0;
  this->sweep = 0.0;
  this->velocityStall = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  this->radialSymmetry = false;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;

  /// how much to change CL per every radian of the control joint value
  this->controlJointRadToCL = 4.0;
}

/////////////////////////////////////////////////
LiftDragPlugin::~LiftDragPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<math::Vector3>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<math::Vector3>("forward");

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<math::Vector3>("upward");

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
    this->controlJoint = this->model->GetJoint(controlJointName);
    if (!this->controlJoint)
    {
      gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("control_joint_rad_to_cl"))
    this->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl");
}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
  math::Vector3 velI = this->link->GetWorldLinearVel(this->cp);

  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*velI + (1.0 - e)*velSmooth;
  // velI = this->velSmooth;

  if (velI.GetLength() <= 0.01)
    return;

  // pose of body
  math::Pose pose = this->link->GetWorldPose();

  // rotate forward and upward vectors into inertial frame
  math::Vector3 forwardI = pose.rot.RotateVector(this->forward);

  math::Vector3 upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    math::Vector3 tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.rot.RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  math::Vector3 spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = math::clamp(
      spanwiseI.Dot(velI) / velI.GetLength(), minRatio, maxRatio);

  // get cos from trig identity
  double cosSweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // velocity in lift-drag plane (expressed in inertial frame) is:
  math::Vector3 velInLDPlane = spanwiseI.Cross(velI.Cross(spanwiseI));

  // get direction of drag
  math::Vector3 dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  math::Vector3 liftDirection = spanwiseI.Cross(velInLDPlane);
  liftDirection.Normalize();

  // get direction of moment
  math::Vector3 momentDirection = spanwiseI;

  double forwardVelocity = forwardI.GetLength() * velInLDPlane.GetLength();
  double cosAlpha = math::clamp(
    forwardI.Dot(velInLDPlane) / forwardVelocity, minRatio, maxRatio);

  // gzerr << "ca " << forwardI.Dot(velInLDPlane) /
  //   (forwardI.GetLength() * velInLDPlane.GetLength()) << "\n";

  // get sign of alpha
  // take upwards component of velocity in lift-drag plane.
  // if sign == upward, then alpha is negative
  double upwardVelocity = upwardI.GetLength() + velInLDPlane.GetLength();
  double alphaSign = -upwardI.Dot(velInLDPlane)/upwardVelocity;

  if (alphaSign > 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.GetLength();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // compute cl at cp, check for stall, correct for sweep
  double cl;
  if (this->alpha > this->alphaStall)
  {
    cl = (this->cla * this->alphaStall +
          this->claStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
    // make sure cl is still great than 0
    cl = std::max(0.0, cl);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cl = (-this->cla * this->alphaStall +
          this->claStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
    // make sure cl is still less than 0
    cl = std::min(0.0, cl);
  }
  else
    cl = this->cla * this->alpha * cosSweepAngle;

  // modify cl per control joint value
  if (this->controlJoint)
  {
    double controlAngle = this->controlJoint->GetAngle(0).Radian();
    cl = cl + this->controlJointRadToCL * controlAngle;
    /// \TODO: also change cm and cd
  }

  // compute lift force at cp
  math::Vector3 lift = cl * q * this->area * liftDirection;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  if (this->alpha > this->alphaStall)
  {
    cd = (this->cda * this->alphaStall +
          this->cdaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
  }
  else if (this->alpha < -this->alphaStall)
  {
    cd = (-this->cda * this->alphaStall +
          this->cdaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
  }
  else
    cd = (this->cda * this->alpha) * cosSweepAngle;

  // make sure drag is positive
  cd = fabs(cd);

  // drag at cp
  math::Vector3 drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (this->alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall +
          this->cmaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall +
          this->cmaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = this->cma * this->alpha * cosSweepAngle;

  /// \TODO: implement cm
  /// for now, reset cm to zero, as cm needs testing
  cm = 0.0;

  // compute moment (torque) at cp
  math::Vector3 moment = cm * q * this->area * momentDirection;

  // moment arm from cg to cp in inertial plane
  math::Vector3 momentArm = pose.rot.RotateVector(
    this->cp - this->link->GetInertial()->GetCoG());
  // gzerr << this->cp << " : " << this->link->GetInertial()->GetCoG() << "\n";

  // force and torque about cg in inertial frame
  math::Vector3 force = lift + drag;
  // + moment.Cross(momentArm);

  math::Vector3 torque = moment;
  // - lift.Cross(momentArm) - drag.Cross(momentArm);

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (velI.GetLength() > 50.0 &&
  //      velI.GetLength() < 50.0))
  if (0)
  {
    gzerr << "=============================\n";
    gzerr << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    gzerr << "spd: [" << velI.GetLength()
          << "] velI: [" << velI << "]\n";
    gzerr << "spd sweep: [" << velInLDPlane.GetLength()
          << "] velI in LD plane: [" << velInLDPlane << "]\n";
    gzerr << "forward (inertial): " << forwardI << "\n";
    gzerr << "upward (inertial): " << upwardI << "\n";
    gzerr << "lift dir (inertial): " << liftDirection << "\n";
    gzerr << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzerr << "sweep: " << this->sweep << "\n";
    gzerr << "alpha: " << this->alpha << "\n";
    gzerr << "lift: " << lift << "\n";
    gzerr << "drag: " << drag << " cd: "
    << cd << " cda: " << this->cda << "\n";
    gzerr << "moment: " << moment << "\n";
    gzerr << "cp momentArm: " << momentArm << "\n";
    gzerr << "force: " << force << "\n";
    gzerr << "torque: " << torque << "\n";
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  this->link->AddTorque(torque);
}
