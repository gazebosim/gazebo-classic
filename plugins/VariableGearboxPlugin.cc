/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <functional>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>
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
/// \brief Interpolate function value y = f(x) and its slope dy/dx
/// at a specified value of x from piecewise cubic Hermite splines
/// specified by knot points [(x_1,y_1), (x_2,y_2), ... (x_N,y_N)]
/// and the slope at each knot point dy/dx|(x_i,y_i) = m_i
/// [m_1, m_2, ... m_N].
/// \param[in] _inputX X value at interpolation point.
/// \param[in] _points Knot points for splines.
/// \param[in] _slopes Slopes at each knot point.
/// \return Interpolated point and slope encoded in Vector3:
/// Vector3(_inputX, interpolatedY, interpolatedSlope).
ignition::math::Vector3d interpolatePointSlope(
    double _inputX,
    const std::vector<ignition::math::Vector2d> &_points,
    const std::vector<double> &_slopes)
{
  GZ_ASSERT(!_points.empty(), "_points should not be empty");
  GZ_ASSERT(!_slopes.empty(), "_slopes should not be empty");

  // Check if _inputX is outside of domain of spline points
  // and do a linear extrapolation if so
  if (_inputX <= _points.front().X())
  {
    const double y = _points.front().Y() +
        _slopes.front() * (_inputX - _points.front().X());
    return ignition::math::Vector3d(_inputX, y, _slopes.front());
  }
  else if (_inputX >= _points.back().X())
  {
    const double y = _points.back().Y() +
        _slopes.back() * (_inputX - _points.back().X());
    return ignition::math::Vector3d(_inputX, y, _slopes.back());
  }
  else
  {
    // Interpolate using the piecewise cubic Hermite splines.

    // Find which spline interval contains _inputX and store in i
    std::size_t i;
    for (i = 0; i < _points.size() - 1; ++i)
    {
      if (_inputX >= _points[i].X() && _inputX <= _points[i+1].X())
      {
        break;
      }
    }

    if (i == _points.size() - 1)
    {
      GZ_ASSERT(i < _points.size() - 1, "failed to find spline index");
      return ignition::math::Vector3d();
    }

    // Rescale the domain of the ith piecewise cubic Hermite spline
    // dy = y_{i+1} - y_i
    // dx = x_{i+1} - x_i
    // t = (x - x_i) / dx
    const double dy = _points[i+1].Y() - _points[i].Y();
    const double dx = _points[i+1].X() - _points[i].X();
    const double t = (_inputX - _points[i].X()) / dx;

    // The ith piecewise cubic Hermite spline can be written as:
    // y(t) = y_i + dx*(m_i*t + p1*t^2 + p2*t^3)
    // or
    // (y(t) - y_i)/dx = m_i*t + p1*t^2 + p2*t^3
    //
    // By differentiating the first equation, the slope m(t) is given by
    // m(t) = m_i + 2*p1*t + 3*p2*t^2
    //
    // Evaluate these equations at the endpoint: t=1, y=y_{i+1}, m=m_{i+1}:
    //   dy/dx = m_i +   p1 +   p2
    // m_{i+1} = m_i + 2*p1 + 3*p2
    //
    // Use a linear system to solve for unknown coefficients p1,p2
    //     p1 +     p2 = dy/dx - m_i
    // 2 * p1 + 3 * p2 = m_{i+1} - m_i
    //
    // Formulate as A*p = b
    // A = [ 1 1 ]    p = [ p1 ]    b = [ dy / dx - m_i ]
    //     [ 2 3 ]        [ p2 ]        [ m_{i+1} - m_i ]
    const double b1 = dy / dx - _slopes[i];
    const double b2 = _slopes[i+1] - _slopes[i];

    // Determinant of A matrix: det(A) = 1
    // Inverse of A matrix: inv(A) = [  3  -1 ]
    //                               [ -2   1 ]
    // p = inv(A) * b
    const double p1 =  3*b1 - b2;
    const double p2 = -2*b1 + b2;

    // Rewrite y(t), m(t) to reduce multiplication operations:
    // y(t) = y_i + dx*t*(m_i + t*(p1 + p2*t))
    const double interpolatedY =
        _points[i].Y() + dx*t*(_slopes[i] + t*(p1 + p2*t));

    // m(t) = m_i + t*(2*p1 + 3*p2*t)
    const double interpolatedSlope = _slopes[i] + t*(2*p1 + 3*p2*t);

    return ignition::math::Vector3d(_inputX, interpolatedY, interpolatedSlope);
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
    gzthrow("No <gearbox_joint_name> tag found.");
  }
  const std::string jointName = _sdf->Get<std::string>("gearbox_joint_name");
  auto joint = this->dataPtr->model->GetJoint(jointName);
  if (joint == nullptr || !joint->HasType(physics::Base::GEARBOX_JOINT))
  {
    gzerr << "Could not find a joint named " << jointName
          << ", but found joints named:"
          << std::endl;
    for (auto j : this->dataPtr->model->GetJoints())
    {
      gzerr << "  " << j->GetName() << std::endl;
    }
    gzthrow("Could not find specified joint.");
  }
  this->dataPtr->gearbox = joint;

  // Identify input joint from parent link
  auto parentLink = joint->GetParent();
  if (parentLink == nullptr)
  {
    gzthrow("Could not find parent link.");
  }
  {
    auto joints = parentLink->GetParentJoints();
    if (joints.size() != 1)
    {
      gzerr << "link [" << parentLink->GetScopedName()
            << "] is child of more than 1 joint, not sure which one to pick."
            << std::endl;
      for (auto j : parentLink->GetParentJoints())
      {
        gzerr << "  " << j->GetName() << std::endl;
      }
      gzthrow("Input joint is ambiguous.");
    }
    this->dataPtr->inputJoint = joints.front();
  }

  // Identify output joint from child link
  auto childLink = joint->GetChild();
  if (childLink == nullptr)
  {
    gzthrow("Could not find child link.");
  }
  {
    auto joints = childLink->GetParentJoints();
    if (joints.size() < 1 || joints.size() > 2)
    {
      gzerr << "link [" << childLink->GetScopedName()
            << "] is child of not 1 or 2 joints, not sure which one to pick."
            << std::endl;
      for (auto j : childLink->GetParentJoints())
      {
        gzerr << "  " << j->GetName() << std::endl;
      }
      gzthrow("Output joint is ambiguous.");
    }
    this->dataPtr->outputJoint = joints.front();
  }

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
        gzthrow("Out of order x_y_dydx element detected.");
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
    gzthrow("No x_y_dydx elements detected");
  }

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&VariableGearboxPlugin::OnUpdate, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  double ratio;
  double refInput;
  double refOutput;
  double inputAngle = this->dataPtr->inputJoint->Position(0);
  // double outputAngle = this->dataPtr->outputJoint->Position(0);

  // the Load function should print an error message and return
  // before setting up this callback if no parameters are found.
  GZ_ASSERT(!this->dataPtr->splinePoints.empty(), "no spline points found");

  ignition::math::Vector3d pointSlope = interpolatePointSlope(
      inputAngle, this->dataPtr->splinePoints, this->dataPtr->splineSlopes);

  refInput = pointSlope.X();
  refOutput = pointSlope.Y();
  // ode gearbox defines gear ratio with an extra sign change
  ratio = -pointSlope.Z();

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
