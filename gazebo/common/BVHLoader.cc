/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <fstream>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix4.hh>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/BVHLoader.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/Console.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
BVHLoader::BVHLoader()
{
}

/////////////////////////////////////////////////
BVHLoader::~BVHLoader()
{
}

/////////////////////////////////////////////////
Skeleton *BVHLoader::Load(const std::string &_filename, double _scale)
{
  std::string fullname = common::find_file(_filename);

  Skeleton *skeleton = NULL;
  std::ifstream file;
  file.open(fullname.c_str());
  std::vector<SkeletonNode*> nodes;
  std::vector<std::vector<std::string> > nodeChannels;
  unsigned int totalChannels = 0;
  std::string line;
  if (file.is_open())
  {
    getline(file, line);
    if (line.find("HIERARCHY") == std::string::npos)
    {
      file.close();
      return NULL;
    }

    SkeletonNode *parent = NULL;
    SkeletonNode *node = NULL;
    while (!file.eof())
    {
      getline(file, line);
      std::vector<std::string> words;
      boost::trim(line);
      boost::split(words, line, boost::is_any_of("   "));
      if (words[0] == "ROOT" || words[0] == "JOINT")
      {
        if (words.size() < 2)
        {
          file.close();
          return NULL;
        }
        SkeletonNode::SkeletonNodeType type = SkeletonNode::JOINT;
        std::string name = words[1];
        node = new SkeletonNode(parent, name, name, type);
        if (words[0] != "End")
          nodes.push_back(node);
      }
      else
        if (words[0] == "OFFSET")
        {
          if (words.size() < 4)
          {
            file.close();
            return NULL;
          }
          ignition::math::Vector3d offset = ignition::math::Vector3d(
              ignition::math::parseFloat(words[1]) * _scale,
              ignition::math::parseFloat(words[2]) * _scale,
              ignition::math::parseFloat(words[3]) * _scale);
          ignition::math::Matrix4d transform(
              ignition::math::Matrix4d::Identity);
          transform.Translate(offset);
          node->SetTransform(transform);
        }
        else
          if (words[0] == "CHANNELS")
          {
            if (words.size() < 3 ||
                static_cast<size_t>(ignition::math::parseInt(words[1]) + 2) >
                 words.size())
            {
              file.close();
              return NULL;
            }
            nodeChannels.push_back(words);
            totalChannels += ignition::math::parseInt(words[1]);
          }
          else
            if (words[0] == "{")
              parent = node;
            else
              if (words[0] == "}")
                parent = parent->GetParent();
              else
                if (words.size() == 2 && words[0] == "End"
                        && words[1] == "Site")
                {
                  /// ignore End Sites
                  getline(file, line);  /// read {
                  getline(file, line);  /// read OFFSET
                  getline(file, line);  /// read }
                }
                else
                {
                  if (nodes.empty())
                  {
                    file.close();
                    return NULL;
                  }
                  skeleton = new Skeleton(nodes[0]);
                  break;
                }
    }
  }
  getline(file, line);
  std::vector<std::string> words;
  boost::trim(line);
  boost::split(words, line, boost::is_any_of("   "));
  unsigned int frameCount = 0;
  double frameTime = 0.0;
  if (words[0] != "Frames:" || words.size() < 2)
  {
    file.close();
    return NULL;
  }
  else
    frameCount = ignition::math::parseInt(words[1]);

  getline(file, line);
  words.clear();
  boost::trim(line);
  boost::split(words, line, boost::is_any_of("   "));

  if (words.size() < 3 || words[0] != "Frame" || words[1] != "Time:")
  {
    file.close();
    return NULL;
  }
  else
    frameTime = ignition::math::parseFloat(words[2]);

  double time = 0.0;
  unsigned int frameNo = 0;

  SkeletonAnimation *animation = new SkeletonAnimation(_filename);

  while (!file.eof())
  {
    getline(file, line);
    words.clear();
    boost::trim(line);
    boost::split(words, line, boost::is_any_of("   "));
    if (words.size() < totalChannels)
    {
      gzwarn << "Frame " << frameNo << " invalid.\n";
      frameNo++;
      time += frameTime;
      continue;
    }

    unsigned int cursor = 0;
    for (unsigned int i = 0; i < nodes.size(); ++i)
    {
      SkeletonNode *node = nodes[i];
      std::vector<std::string> channels = nodeChannels[i];
      ignition::math::Vector3d translation = node->Transform().Translation();
      ignition::math::Vector3d xAxis(1, 0, 0);
      ignition::math::Vector3d yAxis(0, 1, 0);
      ignition::math::Vector3d zAxis(0, 0, 1);
      double xAngle = 0.0;
      double yAngle = 0.0;
      double zAngle = 0.0;
      ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);
      std::vector<ignition::math::Matrix4d> mats;
      unsigned int chanCount = ignition::math::parseInt(channels[1]);
      for (unsigned int j = 2; j < (2 + chanCount); ++j)
      {
        double value = ignition::math::parseFloat(words[cursor]);
        cursor++;
        std::string channel = channels[j];
        if (channel == "Xposition")
          translation.X(value * _scale);
        else
          if (channel == "Yposition")
            translation.Y(value * _scale);
          else
          {
            if (channel == "Zposition")
            {
              translation.Z(value * _scale);
            }
            else
            {
              if (channel == "Zrotation")
              {
                zAngle = GZ_DTOR(value);
                mats.push_back(ignition::math::Matrix4d(
                      ignition::math::Quaterniond(zAxis, zAngle)));
              }
              else
              {
                if (channel == "Xrotation")
                {
                  xAngle = GZ_DTOR(value);
                  mats.push_back(ignition::math::Matrix4d(
                    ignition::math::Quaterniond(xAxis, xAngle)));
                }
                else
                {
                  if (channel == "Yrotation")
                  {
                    yAngle = GZ_DTOR(value);
                    mats.push_back(ignition::math::Matrix4d(
                      ignition::math::Quaterniond(yAxis, yAngle)));
                  }
                }
              }
            }
          }
      }
      while (!mats.empty())
      {
        transform = mats.back() * transform;
        mats.pop_back();
      }
      ignition::math::Matrix4d pos(ignition::math::Matrix4d::Identity);
      pos.Translate(translation);
      transform = pos * transform;
      animation->AddKeyFrame(node->GetName(), time, transform);
    }

    frameNo++;
    time += frameTime;
    if (frameNo == frameCount)
      break;
  }
  if (frameNo < frameCount - 1)
    gzwarn << "BVH file ended unexpectedly.\n";

  skeleton->AddAnimation(animation);

  file.close();
  return skeleton;
}
