/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <boost/bind.hpp>
#include <functional>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include "VariableGearboxPlugin.hh"

namespace gazebo
{
class VariableGearboxPluginPrivate
{
  /// \brief Vector of knot points in piecewise cubic Hermite splines.
  public: std::vector<ignition::math::Vector2d> splinePoints;

  /// \brief Vector of slopes at each knot in piecewise cubic Hermite splines.
  public: std::vector<double> splineSlopes;

  /// \brief Parent model pointer.
  public: physics::ModelPtr model;

  /// \brief Gearbox joint.
  public: physics::JointPtr gearbox;

  /// \brief Input joint.
  public: physics::JointPtr inputJoint;

  /// \brief Output joint.
  public: physics::JointPtr outputJoint;

  /// \brief World update connection.
  public: event::ConnectionPtr updateConnection;
};

/////////////////////////////////////////////////
/// \brief Interpolate point and slope from piecewise cubic Hermite splines.
/// \param[in] _input X value to interpolate.
/// \param[in] _points Knot points for splines.
/// \param[in] _slopes Slopes at each knot point.
/// \return Interpolated point and slope in Vector3 form:
/// Vector3[point.x, point.y, slope].
ignition::math::Vector3d interpolatePointSlope(
    double input,
    const std::vector<ignition::math::Vector2d> &_points,
    const std::vector<double> &_slopes)
{
  // Check if input is outside of domain of spline points
  // and do a linear extrapolation if so
  if (input < _points.front().X())
  {
    return ignition::math::Vector3d(
        _points.front().X(), _points.front().Y(), _slopes.front());
  }
  else if (input > _points.back().X())
  {
    return ignition::math::Vector3d(
        _points.back().X(), _points.back().Y(), _slopes.back());
  }
  else
  {
    // Interpolate over the spline segments
    std::size_t i;
    for (i = 0; i < _points.size() - 1; ++i)
    {
      if (input >= _points[i].X() && input <= _points[i+1].X())
      {
        break;
      }
    }

    if (i == _points.size() - 1)
    {
      GZ_ASSERT(i < _points.size() - 1, "failed to find spline index");
      return ignition::math::Vector3d();
    }

    const double dx = _points[i+1].X() - _points[i].X();
    const double t = (input - _points[i].X()) / dx;
    const double dy = _points[i+1].Y() - _points[i].Y();

    const double g1 = dy / dx - _slopes[i];
    const double g2 = _slopes[i+1] - _slopes[i];
    const double a = -2*g1 + g2;
    const double b =  3*g1 - g2;

    const double y = _points[i].Y() + dx*t*(_slopes[i] + t*(b + a*t));
    const double slope = _slopes[i] + t*(2*b + 3*a*t);

    return ignition::math::Vector3d(input, y, slope);
  }
}

/////////////////////////////////////////////////
VariableGearboxPlugin::VariableGearboxPlugin()
  : dataPtr(new VariableGearboxPluginPrivate)
{
}

/////////////////////////////////////////////////
VariableGearboxPlugin::~VariableGearboxPlugin()
{
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _parent;

  if (!_sdf->HasElement("gearbox_joint_name"))
  {
    gzerr << "No <gearbox_joint_name> tag found."
          << std::endl;
    return;
  }
  const std::string jointName = _sdf->Get<std::string>("gearbox_joint_name");
  auto joint = this->dataPtr->model->GetJoint(jointName);
  if (joint == nullptr || !joint->HasType(physics::Base::GEARBOX_JOINT))
  {
    gzerr << "Could not find a joint named " << jointName << std::endl;
    return;
  }
  this->dataPtr->gearbox = joint;

  // Identify input joint from parent link
  auto parentLink = joint->GetParent();
  if (parentLink == nullptr)
  {
    gzerr << "Could not find parent link." << std::endl;
    return;
  }
  gzdbg << "Checking " << parentLink->GetScopedName()
        << " for its joints."
        << std::endl;

  {
    auto joints = parentLink->GetParentJoints();
    if (joints.size() != 1)
    {
      gzerr << "link [" << parentLink->GetScopedName()
            << "] is child of more than 1 joint, not sure which one to pick."
            << std::endl;
      return;
    }
    this->dataPtr->inputJoint = joints.front();
  }
  gzdbg << "Using " << this->dataPtr->inputJoint->GetScopedName()
        << " as input joint."
        << std::endl;

  // Identify output joint from child link
  auto childLink = joint->GetChild();
  if (childLink == nullptr)
  {
    gzerr << "Could not find child link." << std::endl;
    return;
  }
  gzdbg << "Checking " << childLink->GetScopedName()
        << " for its joints."
        << std::endl;

  {
    auto joints = childLink->GetParentJoints();
    if (joints.size() < 1 || joints.size() > 2)
    {
      gzerr << "link [" << childLink->GetScopedName()
            << "] is child of not 1 or 2 joints, not sure which one to pick."
            << std::endl;
      return;
    }
    this->dataPtr->outputJoint = joints.front();
  }
  gzdbg << "Using " << this->dataPtr->outputJoint->GetScopedName()
        << " as output joint."
        << std::endl;

  if (_sdf->HasElement("x_y_dydx"))
  {
    sdf::ElementPtr x_y_dydxElem = _sdf->GetElement("x_y_dydx");
    while (x_y_dydxElem)
    {
      ignition::math::Vector3d x_y_dydx =
          x_y_dydxElem->Get<ignition::math::Vector3d>();

      if (!this->dataPtr->splinePoints.empty() &&
           this->dataPtr->splinePoints.back().X() >= x_y_dydx.X())
      {
        gzerr << "Out of order x_y_dydx element detected: "
              << this->dataPtr->splinePoints.back().X()
              << " should be less than "
              << x_y_dydx.X()
              << std::endl;
        return;
      }

      this->dataPtr->splinePoints.push_back(
          ignition::math::Vector2d(x_y_dydx.X(),
                                   x_y_dydx.Y()));
      this->dataPtr->splineSlopes.push_back(x_y_dydx.Z());

      x_y_dydxElem = x_y_dydxElem->GetNextElement("x_y_dydx");
    }
  }
  else
  {
    gzerr << "No x_y_dydx elements detected"
          << std::endl;
    return;
  }

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VariableGearboxPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  double ratio;
  double refInput;
  double refOutput;
  double inputAngle = this->dataPtr->inputJoint->GetAngle(0).Radian();
  // double outputAngle = this->dataPtr->outputJoint->GetAngle(0).Radian();

  // the Load function should print an error message and return
  // before setting up this callback if no parameters are found.
  GZ_ASSERT(!this->dataPtr->splinePoints.empty(), "no spline points found");

  ignition::math::Vector3d pointSlope = interpolatePointSlope(
      inputAngle, this->dataPtr->splinePoints, this->dataPtr->splineSlopes);

  refInput = pointSlope.X();
  refOutput = pointSlope.Y();
  ratio = pointSlope.Z();
  ratio = -ratio;

  this->dataPtr->gearbox->SetParam("reference_angle1", 0, refOutput);
  this->dataPtr->gearbox->SetParam("reference_angle2", 0, refInput);
  this->dataPtr->gearbox->SetParam("ratio", 0, ratio);
  // std::cerr << "input " << inputAngle
  //           << " , " << refInput << " ref"
  //           << " : " << ratio
  //           << " : ref " << refOutput
  //           << " , " << outputAngle << " output"
  //           << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(VariableGearboxPlugin)
}
