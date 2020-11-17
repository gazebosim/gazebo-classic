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

#include <ignition/math/SemanticVersion.hh>

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

bool isSdfFrameSemanticsError(const sdf::Error &_err)
{
  switch (_err.Code())
  {
    case sdf::ErrorCode::FRAME_ATTACHED_TO_INVALID:
    case sdf::ErrorCode::FRAME_ATTACHED_TO_CYCLE:
    case sdf::ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR:
    case sdf::ErrorCode::POSE_RELATIVE_TO_INVALID:
    case sdf::ErrorCode::POSE_RELATIVE_TO_CYCLE:
    case sdf::ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR:
      return true;
    default:
      return false;
  }
}

/////////////////////////////////////////////////
void convertPosesToSdf16(const sdf::ElementPtr &_modelElem)
{
  // Only <model> is supported right now, <actor> is not supported.
  if (_modelElem->GetName() != "model")
  {
    return;
  }

  // SDF Model DOM object to be used for resolving poses with frame semantics
  sdf::Model modelSDFDom;
  sdf::Errors errors = modelSDFDom.Load(_modelElem);

  auto sdfVersion =
      ignition::math::SemanticVersion(_modelElem->OriginalVersion());
  // Only print out errors if the original SDFormat version does not support
  // frame semantics
  if (sdfVersion >= ignition::math::SemanticVersion(1, 7))
  {
    for (const auto &error : errors)
    {
      if (isSdfFrameSemanticsError(error))
      {
        gzerr << error << "\n";
      }
    }
  }

  // Continue to try to resolve poses even if there were errors when loading the
  // model

  // Convenience lambda to set the resolved pose on the elementptr if a
  // relative_to attribute is set.
  auto updateIfRelativeTo =
      [](const sdf::ElementPtr &_elem, const sdf::SemanticPose &_semPose)
  {
    if (!_semPose.RelativeTo().empty())
    {
      _elem->GetElement("pose")->Set(common::resolveSdfPose(_semPose));
      _elem->GetElement("pose")->GetAttribute("relative_to")->Reset();
    }
  };

  // Links
  for (std::size_t linkInd = 0; linkInd < modelSDFDom.LinkCount(); ++linkInd)
  {
    auto *linkSDFDom = modelSDFDom.LinkByIndex(linkInd);
    updateIfRelativeTo(linkSDFDom->Element(), linkSDFDom->SemanticPose());

    // Visuals
    for (std::size_t visInd = 0; visInd < linkSDFDom->VisualCount(); ++visInd)
    {
      auto *visSDFDom = linkSDFDom->VisualByIndex(visInd);
      updateIfRelativeTo(visSDFDom->Element(), visSDFDom->SemanticPose());
    }

    // Collisions
    for (std::size_t colInd = 0; colInd < linkSDFDom->CollisionCount();
         ++colInd)
    {
      auto *colSDFDom = linkSDFDom->CollisionByIndex(colInd);
      updateIfRelativeTo(colSDFDom->Element(), colSDFDom->SemanticPose());
    }
  }

  // Joints
  for (std::size_t jointInd = 0; jointInd < modelSDFDom.JointCount();
       ++jointInd)
  {
    auto *jointSDFDom = modelSDFDom.JointByIndex(jointInd);
    updateIfRelativeTo(jointSDFDom->Element(), jointSDFDom->SemanticPose());

    // Axis
    for (std::size_t axisInd = 0; axisInd < 2; ++axisInd)
    {
      auto *axisSDFDom = jointSDFDom->Axis(axisInd);
      if (nullptr != axisSDFDom)
      {
        ignition::math::Vector3d xyz = axisSDFDom->Xyz();
        if (!axisSDFDom->XyzExpressedIn().empty() &&
            axisSDFDom->XyzExpressedIn() != "__model__")
        {
          sdf::Errors xyzErrors = axisSDFDom->ResolveXyz(xyz);
          if (!xyzErrors.empty())
          {
            gzerr << "There was an error in JointAxis::ResolveXyz. There is no "
                  << "optimall fallback since the expressed_in["
                  << axisSDFDom->XyzExpressedIn() << "] value is not empty or "
                  << "__model__. Falling back to the joint frame." << std::endl;
          }
          axisSDFDom->Element()->GetElement("xyz")->Set(xyz);
          axisSDFDom->Element()
              ->GetElement("xyz")
              ->GetAttribute("expressed_in")
              ->Reset();
        }
      }
    }
  }

  for (std::size_t modelInd = 0; modelInd < modelSDFDom.ModelCount();
       ++modelInd)
  {
    auto *nestedModelDom = modelSDFDom.ModelByIndex(modelInd);
    updateIfRelativeTo(nestedModelDom->Element(),
                       nestedModelDom->SemanticPose());
    convertPosesToSdf16(nestedModelDom->Element());
  }
}
}
}
