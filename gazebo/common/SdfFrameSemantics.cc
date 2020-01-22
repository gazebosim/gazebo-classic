/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "gazebo/common/SdfFrameSemantics.hh"
#include "gazebo/common/Console.hh"

#include "sdf/Model.hh"
#include "sdf/Link.hh"
#include "sdf/Visual.hh"
#include "sdf/Collision.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/SemanticPose.hh"

namespace gazebo
{
namespace common
{
/////////////////////////////////////////////////
ignition::math::Pose3d resolveSdfPose(const sdf::SemanticPose &_semPose,
                                      const std::string &_resolveTo)
{
  ignition::math::Pose3d pose;
  sdf::Errors errors = _semPose.Resolve(pose, _resolveTo);
  if (!errors.empty())
  {
    if (!_semPose.RelativeTo().empty())
    {
      gzerr << "There was an error in SemanticPose::Resolve\n";
      for (const auto &err : errors)
      {
        gzerr << err.Message() << std::endl;
      }
      gzerr << "There is no optimal fallback since the relative_to attribute["
            << _semPose.RelativeTo() << "] does not match the _resolveTo "
            << "argument[" << _resolveTo << "]. "
            << "Falling back to using the raw Pose.\n";
    }
    pose = _semPose.RawPose();
  }
  return pose;
}
}
}
