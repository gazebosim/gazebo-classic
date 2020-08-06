/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <map>
#include <mutex>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/physics/ode/ODETypes.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>

#include "plugins/WheelSlipPlugin.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Joint> JointWeakPtr;
    typedef boost::weak_ptr<physics::Link> LinkWeakPtr;
    typedef boost::weak_ptr<physics::Model> ModelWeakPtr;
    typedef boost::weak_ptr<physics::ODESurfaceParams> ODESurfaceParamsWeakPtr;
  }

  class WheelSlipPluginPrivate
  {
    public: class LinkSurfaceParams
    {
      /// \brief Pointer to wheel spin joint.
      public: physics::JointWeakPtr joint;

      /// \brief Pointer to ODESurfaceParams object.
      public: physics::ODESurfaceParamsWeakPtr surface;

      /// \brief Unitless wheel slip compliance in lateral direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLateral = 0;

      /// \brief Unitless wheel slip compliance in longitudinal direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLongitudinal = 0;

      /// \brief Wheel normal force estimate used to compute slip
      /// compliance for ODE, which takes units of 1/N.
      public: double wheelNormalForce = 0;

      /// \brief Wheel radius extracted from collision shape if not
      /// specified as xml parameter.
      public: double wheelRadius = 0;

      /// \brief Publish slip for each wheel.
      public: transport::PublisherPtr slipPub;
    };

    /// \brief Initial gravity direction in parent model frame.
    public: ignition::math::Vector3d initialGravityDirection;

    /// \brief Model pointer.
    public: physics::ModelWeakPtr model;

    /// \brief Protect data access during transport callbacks
    public: std::mutex mutex;

    /// \brief Gazebo communication node
    /// \todo: Transition to ignition-transport in gazebo8
    public: transport::NodePtr gzNode;

    /// \brief Link and surface pointers to update.
    public: std::map<physics::LinkWeakPtr,
                        LinkSurfaceParams> mapLinkSurfaceParams;

    /// \brief Lateral slip compliance subscriber.
    /// \todo: Transition to ignition-transport in gazebo8.
    public: transport::SubscriberPtr lateralComplianceSub;

    /// \brief Longitudinal slip compliance subscriber.
    /// \todo: Transition to ignition-transport in gazebo8.
    public: transport::SubscriberPtr longitudinalComplianceSub;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;
  };
}

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(WheelSlipPlugin)

/////////////////////////////////////////////////
WheelSlipPlugin::WheelSlipPlugin()
  : dataPtr(new WheelSlipPluginPrivate)
{
}

/////////////////////////////////////////////////
WheelSlipPlugin::~WheelSlipPlugin()
{
}

/////////////////////////////////////////////////
void WheelSlipPlugin::Fini()
{
  this->dataPtr->updateConnection.reset();

  this->dataPtr->lateralComplianceSub.reset();
  this->dataPtr->longitudinalComplianceSub.reset();
  for (auto linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipPub.reset();
  }
  if (this->dataPtr->gzNode)
    this->dataPtr->gzNode->Fini();
}

/////////////////////////////////////////////////
void WheelSlipPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "WheelSlipPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "WheelSlipPlugin sdf pointer is NULL");

  this->dataPtr->model = _model;
  auto world = _model->GetWorld();
  GZ_ASSERT(world, "world pointer is NULL");
  {
    ignition::math::Vector3d gravity = world->Gravity();
    ignition::math::Quaterniond initialModelRot =
        _model->WorldPose().Rot();
    this->dataPtr->initialGravityDirection =
        initialModelRot.RotateVectorReverse(gravity.Normalized());
  }

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return;
  }

  // Read each wheel element
  for (auto wheelElem = _sdf->GetElement("wheel"); wheelElem;
      wheelElem = wheelElem->GetNextElement("wheel"))
  {
    if (!wheelElem->HasAttribute("link_name"))
    {
      gzerr << "wheel element missing link_name attribute" << std::endl;
      continue;
    }

    // Get link name
    auto linkName = wheelElem->Get<std::string>("link_name");

    WheelSlipPluginPrivate::LinkSurfaceParams params;
    if (wheelElem->HasElement("slip_compliance_lateral"))
    {
      params.slipComplianceLateral =
        wheelElem->Get<double>("slip_compliance_lateral");
    }
    if (wheelElem->HasElement("slip_compliance_longitudinal"))
    {
      params.slipComplianceLongitudinal =
        wheelElem->Get<double>("slip_compliance_longitudinal");
    }
    if (wheelElem->HasElement("wheel_normal_force"))
    {
      params.wheelNormalForce = wheelElem->Get<double>("wheel_normal_force");
    }

    if (wheelElem->HasElement("wheel_radius"))
    {
      params.wheelRadius = wheelElem->Get<double>("wheel_radius");
    }

    auto link = _model->GetLink(linkName);
    if (link == nullptr)
    {
      gzerr << "Could not find link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto collisions = link->GetCollisions();
    if (collisions.empty() || collisions.size() != 1)
    {
      gzerr << "There should be 1 collision in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << collisions.size() << " were found"
            << std::endl;
      continue;
    }
    auto collision = collisions.front();
    if (collision == nullptr)
    {
      gzerr << "Could not find collision in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    auto surface = collision->GetSurface();
    auto odeSurface =
      boost::dynamic_pointer_cast<physics::ODESurfaceParams>(surface);
    if (odeSurface == nullptr)
    {
      gzerr << "Could not find ODE Surface "
            << "in collision named [" << collision->GetName()
            << "] in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.surface = odeSurface;

    auto joints = link->GetParentJoints();
    if (joints.empty() || joints.size() != 1)
    {
      gzerr << "There should be 1 parent joint for link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << ", but " << joints.size() << " were found"
            << std::endl;
      continue;
    }
    auto joint = joints.front();
    if (joint == nullptr)
    {
      gzerr << "Could not find parent joint for link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }
    params.joint = joint;

    if (params.wheelRadius <= 0)
    {
      // get collision shape and extract radius if it is a cylinder or sphere
      auto shape = collision->GetShape();
      if (shape->HasType(physics::Base::CYLINDER_SHAPE))
      {
        auto cyl = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
        if (cyl != nullptr)
        {
          params.wheelRadius = cyl->GetRadius();
        }
      }
      else if (shape->HasType(physics::Base::SPHERE_SHAPE))
      {
        auto sphere = boost::dynamic_pointer_cast<physics::SphereShape>(shape);
        if (sphere != nullptr)
        {
          params.wheelRadius = sphere->GetRadius();
        }
      }
      else
      {
        gzerr << "A positive wheel radius was not specified in the"
              << " [wheel_radius] parameter, and the the wheel radius"
              << " could not be identified automatically because a"
              << " sphere or cylinder collision shape could not be found."
              << " Skipping link [" << linkName << "]."
              << std::endl;
        continue;
      }

      // if that still didn't work, skip this link
      if (params.wheelRadius <= 0)
      {
        gzerr << "Found wheel radius [" << params.wheelRadius
              << "], which is not positive"
              << " in link named [" << linkName
              << "] in model [" << _model->GetScopedName() << "]"
              << std::endl;
        continue;
      }
    }

    if (params.wheelNormalForce <= 0)
    {
      gzerr << "Found wheel normal force [" << params.wheelNormalForce
            << "], which is not positive"
            << " in link named [" << linkName
            << "] in model [" << _model->GetScopedName() << "]"
            << std::endl;
      continue;
    }

    this->dataPtr->mapLinkSurfaceParams[link] = params;
  }

  if (this->dataPtr->mapLinkSurfaceParams.empty())
  {
    gzerr << "No ODE links and surfaces found, plugin is disabled" << std::endl;
    return;
  }

  // Subscribe to slip compliance updates
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init(world->Name());

  // add publishers
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto link = linkSurface.first.lock();
    GZ_ASSERT(link, "link should still exist inside Load");
    auto &params = linkSurface.second;
    params.slipPub = this->dataPtr->gzNode->Advertise<msgs::Vector3d>(
        "~/" + _model->GetName() + "/wheel_slip/" + link->GetName());
  }

  this->dataPtr->lateralComplianceSub = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/lateral_compliance",
      &WheelSlipPlugin::OnLateralCompliance, this);

  this->dataPtr->longitudinalComplianceSub = this->dataPtr->gzNode->Subscribe(
      "~/" + _model->GetName() + "/wheel_slip/longitudinal_compliance",
      &WheelSlipPlugin::OnLongitudinalCompliance, this);

  // Connect to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WheelSlipPlugin::Update, this));
}

/////////////////////////////////////////////////
physics::ModelPtr WheelSlipPlugin::GetParentModel() const
{
  return this->dataPtr->model.lock();
}

/////////////////////////////////////////////////
void WheelSlipPlugin::GetSlips(
        std::map<std::string, ignition::math::Vector3d> &_out) const
{
  auto model = this->GetParentModel();
  if (!model)
  {
    gzerr << "Parent model does not exist" << std::endl;
    return;
  }
  auto modelWorldPose = model->WorldPose();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto link = linkSurface.first.lock();
    if (!link)
      continue;
    auto params = linkSurface.second;

    // Compute wheel velocity in parent model frame
    auto wheelWorldLinearVel = link->WorldLinearVel();
    auto wheelModelLinearVel =
        modelWorldPose.Rot().RotateVectorReverse(wheelWorldLinearVel);
    // Compute wheel spin axis in parent model frame
    auto joint = params.joint.lock();
    if (!joint)
      continue;
    auto wheelWorldAxis = joint->GlobalAxis(0).Normalized();
    auto wheelModelAxis =
        modelWorldPose.Rot().RotateVectorReverse(wheelWorldAxis);
    // Estimate longitudinal direction as cross product of initial gravity
    // direction with wheel spin axis.
    auto longitudinalModelAxis =
        this->dataPtr->initialGravityDirection.Cross(wheelModelAxis);

    double spinSpeed = params.wheelRadius * joint->GetVelocity(0);
    double lateralSpeed = wheelModelAxis.Dot(wheelModelLinearVel);
    double longitudinalSpeed = longitudinalModelAxis.Dot(wheelModelLinearVel);

    ignition::math::Vector3d slip;
    slip.X(longitudinalSpeed - spinSpeed);
    slip.Y(lateralSpeed);
    slip.Z(spinSpeed);

    auto name = link->GetName();
    _out[name] = slip;
  }
}

/////////////////////////////////////////////////
void WheelSlipPlugin::OnLateralCompliance(ConstGzStringPtr &_msg)
{
  try
  {
    this->SetSlipComplianceLateral(std::stod(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Invalid slip compliance data[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void WheelSlipPlugin::OnLongitudinalCompliance(ConstGzStringPtr &_msg)
{
  try
  {
    this->SetSlipComplianceLongitudinal(std::stod(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Invalid slip compliance data[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void WheelSlipPlugin::SetSlipComplianceLateral(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipComplianceLateral = _compliance;
  }
}

/////////////////////////////////////////////////
void WheelSlipPlugin::SetSlipComplianceLongitudinal(const double _compliance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    linkSurface.second.slipComplianceLongitudinal = _compliance;
  }
}

/////////////////////////////////////////////////
void WheelSlipPlugin::Update()
{
  // Get slip data so it can be published later
  std::map<std::string, ignition::math::Vector3d> slips;
  this->GetSlips(slips);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    const auto &params = linkSurface.second;

    // get user-defined normal force constant
    double force = params.wheelNormalForce;

    // get link angular velocity parallel to joint axis
    ignition::math::Vector3d wheelAngularVelocity;
    auto link = linkSurface.first.lock();
    if (link)
      wheelAngularVelocity = link->WorldAngularVel();

    ignition::math::Vector3d jointAxis;
    auto joint = params.joint.lock();
    if (joint)
      jointAxis = joint->GlobalAxis(0);

    double spinAngularVelocity = wheelAngularVelocity.Dot(jointAxis);

    auto surface = params.surface.lock();
    if (surface)
    {
      // As discussed in WheelSlipPlugin.hh, the ODE slip1 and slip2
      // parameters have units of inverse viscous damping:
      // [linear velocity / force] or [m / s / N].
      // Since the slip compliance parameters supplied to the plugin
      // are unitless, they must be scaled by a linear speed and force
      // magnitude before being passed to ODE.
      // The force is taken from a user-defined constant that should roughly
      // match the steady-state normal force at the wheel.
      // The linear speed is computed dynamically at each time step as
      // radius * spin angular velocity.
      // This choice of linear speed corresponds to the denominator of
      // the slip ratio during acceleration (see equation (1) in
      // Yoshida, Hamano 2002 DOI 10.1109/ROBOT.2002.1013712
      // "Motion dynamics of a rover with slip-based traction model").
      // The acceleration form is more well-behaved numerically at low-speed
      // and when the vehicle is at rest than the braking form,
      // so it is used for both slip directions.
      double speed = params.wheelRadius * std::abs(spinAngularVelocity);
      surface->slip1 = speed / force * params.slipComplianceLateral;
      surface->slip2 = speed / force * params.slipComplianceLongitudinal;
    }

    // Try to publish slip data for this wheel
    if (link)
    {
      msgs::Vector3d msg;
      auto name = link->GetName();
      msg = msgs::Convert(slips[name]);
      if (params.slipPub)
        params.slipPub->Publish(msg);
    }
  }
}
