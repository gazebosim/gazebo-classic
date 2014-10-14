/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <tinyxml.h>
#include <math.h>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Matrix4.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/ColladaLoader.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Exception.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
  ColladaLoader::ColladaLoader()
: MeshLoader(), meter(1.0)
{
}

//////////////////////////////////////////////////
ColladaLoader::~ColladaLoader()
{
}

//////////////////////////////////////////////////
Mesh *ColladaLoader::Load(const std::string &_filename)
{
  // reset scale
  this->meter = 1.0;

  TiXmlDocument xmlDoc;

  this->path.clear();
  if (_filename.rfind('/') != std::string::npos)
  {
    this->path = _filename.substr(0, _filename.rfind('/'));
  }

  this->filename = _filename;
  if (!xmlDoc.LoadFile(_filename))
    gzerr << "Unable to load collada file[" << _filename << "]\n";

  this->colladaXml = xmlDoc.FirstChildElement("COLLADA");
  if (!this->colladaXml)
    gzerr << "Missing COLLADA tag\n";

  if (std::string(this->colladaXml->Attribute("version")) != "1.4.0" &&
      std::string(this->colladaXml->Attribute("version")) != "1.4.1")
    gzerr << "Invalid collada file. Must be version 1.4.0 or 1.4.1\n";

  TiXmlElement *assetXml = this->colladaXml->FirstChildElement("asset");
  if (assetXml)
  {
    TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
    if (unitXml && unitXml->Attribute("meter"))
      this->meter = math::parseFloat(unitXml->Attribute("meter"));
  }

  Mesh *mesh = new Mesh();
  mesh->SetPath(this->path);

  this->LoadScene(mesh);

  // This will make the model the correct size.
  mesh->Scale(this->meter);

  return mesh;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadScene(Mesh *_mesh)
{
  TiXmlElement *sceneXml = this->colladaXml->FirstChildElement("scene");
  std::string sceneURL =
    sceneXml->FirstChildElement("instance_visual_scene")->Attribute("url");

  TiXmlElement *visSceneXml = this->GetElementId("visual_scene", sceneURL);

  if (!visSceneXml)
  {
    gzerr << "Unable to find visual_scene id ='" << sceneURL << "'\n";
    return;
  }

  TiXmlElement *nodeXml = visSceneXml->FirstChildElement("node");
  while (nodeXml)
  {
    this->LoadNode(nodeXml, _mesh, math::Matrix4::IDENTITY);
    nodeXml = nodeXml->NextSiblingElement("node");
  }
}

/////////////////////////////////////////////////
void ColladaLoader::LoadNode(TiXmlElement *_elem, Mesh *_mesh,
    const math::Matrix4 &_transform)
{
  TiXmlElement *nodeXml;
  TiXmlElement *instGeomXml;

  math::Matrix4 transform = this->LoadNodeTransform(_elem);
  transform = _transform * transform;

  if (_elem->Attribute("name"))
  {
    this->currentNodeName = _elem->Attribute("name");
  }

  nodeXml = _elem->FirstChildElement("node");
  while (nodeXml)
  {
    this->LoadNode(nodeXml, _mesh, transform);
    nodeXml = nodeXml->NextSiblingElement("node");
  }

  if (_elem->FirstChildElement("instance_node"))
  {
    std::string nodeURLStr =
      _elem->FirstChildElement("instance_node")->Attribute("url");

    nodeXml = this->GetElementId("node", nodeURLStr);
    if (!nodeXml)
    {
      gzerr << "Unable to find node[" << nodeURLStr << "]\n";
      return;
    }
    this->LoadNode(nodeXml, _mesh, transform);
    return;
  }
  else
    nodeXml = _elem;

  instGeomXml = nodeXml->FirstChildElement("instance_geometry");
  while (instGeomXml)
  {
    std::string geomURL = instGeomXml->Attribute("url");
    TiXmlElement *geomXml = this->GetElementId("geometry", geomURL);

    this->materialMap.clear();
    TiXmlElement *bindMatXml, *techniqueXml, *matXml;
    bindMatXml = instGeomXml->FirstChildElement("bind_material");
    while (bindMatXml)
    {
      if ((techniqueXml = bindMatXml->FirstChildElement("technique_common")))
      {
        matXml = techniqueXml->FirstChildElement("instance_material");
        while (matXml)
        {
          std::string symbol = matXml->Attribute("symbol");
          std::string target = matXml->Attribute("target");
          this->materialMap[symbol] = target;
          matXml = matXml->NextSiblingElement("instance_material");
        }
      }
      bindMatXml = bindMatXml->NextSiblingElement("bind_material");
    }

    this->LoadGeometry(geomXml, transform, _mesh);
    instGeomXml = instGeomXml->NextSiblingElement("instance_geometry");
  }

  TiXmlElement *instContrXml =
    nodeXml->FirstChildElement("instance_controller");
  while (instContrXml)
  {
    std::string contrURL = instContrXml->Attribute("url");
    TiXmlElement *contrXml = this->GetElementId("controller", contrURL);

    TiXmlElement *instSkelXml = instContrXml->FirstChildElement("skeleton");
    std::string rootURL = instSkelXml->GetText();
    TiXmlElement *rootNodeXml = this->GetElementId("node", rootURL);

    this->materialMap.clear();
    TiXmlElement *bindMatXml, *techniqueXml, *matXml;
    bindMatXml = instContrXml->FirstChildElement("bind_material");
    while (bindMatXml)
    {
      if ((techniqueXml = bindMatXml->FirstChildElement("technique_common")))
      {
        matXml = techniqueXml->FirstChildElement("instance_material");
        while (matXml)
        {
          std::string symbol = matXml->Attribute("symbol");
          std::string target = matXml->Attribute("target");
          this->materialMap[symbol] = target;
          matXml = matXml->NextSiblingElement("instance_material");
        }
      }
      bindMatXml = bindMatXml->NextSiblingElement("bind_material");
    }

    this->LoadController(contrXml, rootNodeXml, transform, _mesh);
    instContrXml = instContrXml->NextSiblingElement("instance_controller");
  }
}

/////////////////////////////////////////////////
math::Matrix4 ColladaLoader::LoadNodeTransform(TiXmlElement *_elem)
{
  math::Matrix4 transform(math::Matrix4::IDENTITY);

  if (_elem->FirstChildElement("matrix"))
  {
    std::string matrixStr = _elem->FirstChildElement("matrix")->GetText();
    std::istringstream iss(matrixStr);
    std::vector<double> values(16);
    for (unsigned int i = 0; i < 16; i++)
      iss >> values[i];
    transform.Set(values[0], values[1], values[2], values[3],
        values[4], values[5], values[6], values[7],
        values[8], values[9], values[10], values[11],
        values[12], values[13], values[14], values[15]);
  }
  else
  {
    if (_elem->FirstChildElement("translate"))
    {
      std::string transStr = _elem->FirstChildElement("translate")->GetText();
      math::Vector3 translate;
      translate = boost::lexical_cast<math::Vector3>(transStr);
      // translate *= this->meter;
      transform.SetTranslate(translate);
    }

    TiXmlElement *rotateXml = _elem->FirstChildElement("rotate");
    while (rotateXml)
    {
      math::Matrix3 mat;
      math::Vector3 axis;
      double angle;

      std::string rotateStr = rotateXml->GetText();
      std::istringstream iss(rotateStr);

      iss >> axis.x >> axis.y >> axis.z;
      iss >> angle;
      mat.SetFromAxis(axis, GZ_DTOR(angle));

      transform = transform * mat;

      rotateXml = rotateXml->NextSiblingElement("rotate");
    }

    if (_elem->FirstChildElement("scale"))
    {
      std::string scaleStr = _elem->FirstChildElement("scale")->GetText();
      math::Vector3 scale;
      scale = boost::lexical_cast<math::Vector3>(scaleStr);
      math::Matrix4 scaleMat;
      scaleMat.SetScale(scale);
      transform = transform * scaleMat;
    }
  }

  return transform;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadController(TiXmlElement *_contrXml,
      TiXmlElement *_skelXml, const math::Matrix4 &_transform, Mesh *_mesh)
{
  Skeleton *skeleton = new Skeleton(this->LoadSkeletonNodes(_skelXml, NULL));
  _mesh->SetSkeleton(skeleton);

  TiXmlElement *rootXml = _contrXml->GetDocument()->RootElement();

  if (rootXml->FirstChildElement("library_animations"))
    this->LoadAnimations(rootXml->FirstChildElement("library_animations"),
        skeleton);

  TiXmlElement *skinXml = _contrXml->FirstChildElement("skin");
  std::string geomURL = skinXml->Attribute("source");

  math::Matrix4 bindTrans;
  std::string matrixStr =
        skinXml->FirstChildElement("bind_shape_matrix")->GetText();
  std::istringstream iss(matrixStr);
  std::vector<double> values(16);
  for (unsigned int i = 0; i < 16; i++)
    iss >> values[i];
  bindTrans.Set(values[0], values[1], values[2], values[3],
                values[4], values[5], values[6], values[7],
                values[8], values[9], values[10], values[11],
                values[12], values[13], values[14], values[15]);

  skeleton->SetBindShapeTransform(bindTrans);

  TiXmlElement *jointsXml = skinXml->FirstChildElement("joints");
  std::string jointsURL, invBindMatURL;
  TiXmlElement *inputXml = jointsXml->FirstChildElement("input");
  while (inputXml)
  {
    std::string semantic = inputXml->Attribute("semantic");
    std::string source = inputXml->Attribute("source");
    if (semantic == "JOINT")
      jointsURL = source;
    else
    {
      if (semantic == "INV_BIND_MATRIX")
        invBindMatURL = source;
    }
    inputXml = inputXml->NextSiblingElement("input");
  }

  jointsXml = this->GetElementId("source", jointsURL);

  if (!jointsXml)
  {
    gzerr << "Could not find node[" << jointsURL << "]\n";
    gzthrow("Faild to parse skinning information in Collada file.");
  }

  std::string jointsStr = jointsXml->FirstChildElement("Name_array")->GetText();

  std::vector<std::string> joints;
  boost::split(joints, jointsStr, boost::is_any_of("   "));

  TiXmlElement *invBMXml = this->GetElementId("source", invBindMatURL);

  if (!invBMXml)
  {
    gzerr << "Could not find node[" << invBindMatURL << "]\n";
    gzthrow("Faild to parse skinning information in Collada file.");
  }

  std::string posesStr = invBMXml->FirstChildElement("float_array")->GetText();

  std::vector<std::string> strs;
  boost::split(strs, posesStr, boost::is_any_of("   "));

  for (unsigned int i = 0; i < joints.size(); i++)
  {
    unsigned int id = i * 16;
    math::Matrix4 mat;
    mat.Set(math::parseFloat(strs[id +  0]), math::parseFloat(strs[id +  1]),
            math::parseFloat(strs[id +  2]), math::parseFloat(strs[id +  3]),
            math::parseFloat(strs[id +  4]), math::parseFloat(strs[id +  5]),
            math::parseFloat(strs[id +  6]), math::parseFloat(strs[id +  7]),
            math::parseFloat(strs[id +  8]), math::parseFloat(strs[id +  9]),
            math::parseFloat(strs[id + 10]), math::parseFloat(strs[id + 11]),
            math::parseFloat(strs[id + 12]), math::parseFloat(strs[id + 13]),
            math::parseFloat(strs[id + 14]), math::parseFloat(strs[id + 15]));

    skeleton->GetNodeByName(joints[i])->SetInverseBindTransform(mat);
  }

  TiXmlElement *vertWeightsXml = skinXml->FirstChildElement("vertex_weights");

  inputXml = vertWeightsXml->FirstChildElement("input");
  unsigned int jOffset = 0;
  unsigned int wOffset = 0;
  std::string weightsURL;
  while (inputXml)
  {
    std::string semantic = inputXml->Attribute("semantic");
    std::string source = inputXml->Attribute("source");
    int offset;
    inputXml->Attribute("offset", &offset);

    if (semantic == "JOINT")
      jOffset = offset;
    else
      if (semantic == "WEIGHT")
      {
        weightsURL = source;
        wOffset = offset;
      }
    inputXml = inputXml->NextSiblingElement("input");
  }

  TiXmlElement *weightsXml = this->GetElementId("source", weightsURL);

  std::string wString = weightsXml->FirstChildElement("float_array")->GetText();
  std::vector<std::string> wStrs;
  boost::split(wStrs, wString, boost::is_any_of("   "));

  std::vector<float> weights;
  for (unsigned int i = 0; i < wStrs.size(); i++)
    weights.push_back(math::parseFloat(wStrs[i]));

  std::string cString = vertWeightsXml->FirstChildElement("vcount")->GetText();
  std::string vString = vertWeightsXml->FirstChildElement("v")->GetText();
  std::vector<std::string> vCountStrs;
  std::vector<std::string> vStrs;

  boost::split(vCountStrs, cString, boost::is_any_of("   "));
  boost::split(vStrs, vString, boost::is_any_of("   "));

  std::vector<unsigned int> vCount;
  std::vector<unsigned int> v;

  for (unsigned int i = 0; i < vCountStrs.size(); i++)
    vCount.push_back(math::parseInt(vCountStrs[i]));

  for (unsigned int i = 0; i < vStrs.size(); i++)
    v.push_back(math::parseInt(vStrs[i]));

  skeleton->SetNumVertAttached(vCount.size());

  unsigned int vIndex = 0;
  for (unsigned int i = 0; i < vCount.size(); i++)
  {
    for (unsigned int j = 0; j < vCount[i]; j++)
    {
      skeleton->AddVertNodeWeight(i, joints[v[vIndex + jOffset]],
                                    weights[v[vIndex + wOffset]]);
      vIndex += (jOffset + wOffset + 1);
    }
  }

  TiXmlElement *geomXml = this->GetElementId("geometry", geomURL);
  this->LoadGeometry(geomXml, _transform, _mesh);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadAnimations(TiXmlElement *_xml, Skeleton *_skel)
{
  TiXmlElement *childXml = _xml->FirstChildElement("animation");
  if (childXml->FirstChildElement("animation"))
  {
    while (childXml)
    {
      this->LoadAnimationSet(childXml, _skel);
      childXml->NextSiblingElement("animation");
    }
  }
  else
    this->LoadAnimationSet(_xml, _skel);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadAnimationSet(TiXmlElement *_xml, Skeleton *_skel)
{
  std::stringstream animName;
  if (_xml->Attribute("name"))
    animName << _xml->Attribute("name");
  else
    if (_xml->Attribute("id"))
      animName << _xml->Attribute("id");
    else
      animName << "animation" << (_skel->GetNumAnimations() + 1);

  RawSkeletonAnim animation;

  TiXmlElement *animXml = _xml->FirstChildElement("animation");
  while (animXml)
  {
    TiXmlElement *chanXml = animXml->FirstChildElement("channel");

    while (chanXml)
    {
      std::string sourceURL = chanXml->Attribute("source");
      std::string targetStr = chanXml->Attribute("target");

      std::string targetBone = targetStr.substr(0, targetStr.find('/'));
      char sep = '0';
      if (targetStr.find('(') != std::string::npos)
        sep = '(';
      else
        if (targetStr.find('.') != std::string::npos)
          sep = '.';

      std::string targetTrans;
      if (sep == '0')
        targetTrans = targetStr.substr(targetStr.find('/') + 1);
      else
        targetTrans = targetStr.substr(targetStr.find('/') + 1,
                          targetStr.find(sep) - targetStr.find('/') - 1);

      std::string idxStr = targetStr.substr(targetStr.find(sep) + 1);
      int idx1 = -1;
      int idx2 = -1;

      if (sep == '.')
        idx1 = (idxStr == "X") ? 0 : ((idxStr == "Y") ? 1 : ((idxStr == "Z")
            ? 2 : ((idxStr == "ANGLE") ? 3 : -1)));
      else
        if (sep == '(')
        {
          std::string idx1Str = idxStr.substr(0, 1);
          idx1 = math::parseInt(idx1Str);
          if (idxStr.length() > 4)
          {
            std::string idx2Str = idxStr.substr(3, 1);
            idx2 = math::parseInt(idx2Str);
          }
        }

      TiXmlElement *frameTimesXml = NULL;
      TiXmlElement *frameTransXml = NULL;

      TiXmlElement *sampXml = this->GetElementId("sampler", sourceURL);
      TiXmlElement *inputXml = sampXml->FirstChildElement("input");
      while (inputXml)
      {
        std::string semantic = inputXml->Attribute("semantic");
        if (semantic == "INPUT")
          frameTimesXml = this->GetElementId("source",
                              inputXml->Attribute("source"));
        else
          if (semantic == "OUTPUT")
            frameTransXml = this->GetElementId("source",
                              inputXml->Attribute("source"));
        /// FIXME interpolation semantic?

        inputXml = inputXml->NextSiblingElement("input");
      }
      TiXmlElement *timeArray = frameTimesXml->FirstChildElement("float_array");
      std::string timeStr = timeArray->GetText();
      std::vector<std::string> timeStrs;
      boost::split(timeStrs, timeStr, boost::is_any_of("   "));

      std::vector<double> times;
      for (unsigned int i = 0; i < timeStrs.size(); i++)
        times.push_back(math::parseFloat(timeStrs[i]));

      TiXmlElement *output = frameTransXml->FirstChildElement("float_array");
      std::string outputStr = output->GetText();
      std::vector<std::string> outputStrs;
      boost::split(outputStrs, outputStr, boost::is_any_of("   "));

      std::vector<double> values;
      for (unsigned int i = 0; i < outputStrs.size(); i++)
        values.push_back(math::parseFloat(outputStrs[i]));

      TiXmlElement *accessor =
        frameTransXml->FirstChildElement("technique_common");
      accessor = accessor->FirstChildElement("accessor");

      unsigned int stride = math::parseInt(accessor->Attribute("stride"));

      for (unsigned int i = 0; i < times.size(); i++)
      {
        if (animation[targetBone].find(times[i]) == animation[targetBone].end())
          animation[targetBone][times[i]] =
                      _skel->GetNodeById(targetBone)->GetTransforms();

        std::vector<NodeTransform> *frame = &animation[targetBone][times[i]];

        for (unsigned int j = 0; j < (*frame).size(); j++)
        {
          NodeTransform *nt = &((*frame)[j]);
          if (nt->GetSID() == targetTrans)
          {
            if (idx1 != -1)
            {
              int index = (idx2 == -1) ? idx1 : (idx1 * 4) + idx2;
              nt->SetComponent(index, values[i]);
            }
            else
              for (unsigned int k = 0; k < stride; k++)
                nt->SetComponent(k, values[(i*stride) + k]);
          }
        }
      }

      chanXml = chanXml->NextSiblingElement("channel");
    }

    animXml = animXml->NextSiblingElement("animation");
  }

  SkeletonAnimation *anim = new SkeletonAnimation(animName.str());

  for (RawSkeletonAnim::iterator iter = animation.begin();
        iter != animation.end(); ++iter)
    for (RawNodeAnim::iterator niter = iter->second.begin();
          niter != iter->second.end(); ++niter)
    {
      math::Matrix4 transform(math::Matrix4::IDENTITY);
      for (unsigned int i = 0; i < niter->second.size(); i++)
      {
        niter->second[i].RecalculateMatrix();
        transform = transform * niter->second[i]();
      }
      anim->AddKeyFrame(iter->first, niter->first, transform);
    }

  _skel->AddAnimation(anim);
}

/////////////////////////////////////////////////
SkeletonNode* ColladaLoader::LoadSkeletonNodes(TiXmlElement *_xml,
      SkeletonNode *_parent)
{
  std::string name;
  if (_xml->Attribute("sid"))
    name = _xml->Attribute("sid");
  else
    name = _xml->Attribute("name");

  SkeletonNode* node = new SkeletonNode(_parent, name, _xml->Attribute("id"));

  if (std::string(_xml->Attribute("type")) == std::string("NODE"))
    node->SetType(SkeletonNode::NODE);

  this->SetSkeletonNodeTransform(_xml, node);

  TiXmlElement *childXml = _xml->FirstChildElement("node");
  while (childXml)
  {
    this->LoadSkeletonNodes(childXml, node);
    childXml = childXml->NextSiblingElement("node");
  }
  return node;
}

/////////////////////////////////////////////////
void ColladaLoader::SetSkeletonNodeTransform(TiXmlElement *_elem,
      SkeletonNode *_node)
{
  math::Matrix4 transform(math::Matrix4::IDENTITY);

  if (_elem->FirstChildElement("matrix"))
  {
    std::string matrixStr = _elem->FirstChildElement("matrix")->GetText();
    std::istringstream iss(matrixStr);
    std::vector<double> values(16);
    for (unsigned int i = 0; i < 16; i++)
      iss >> values[i];
    transform.Set(values[0], values[1], values[2], values[3],
        values[4], values[5], values[6], values[7],
        values[8], values[9], values[10], values[11],
        values[12], values[13], values[14], values[15]);

    NodeTransform nt(transform);
    nt.SetSourceValues(transform);
    if (_elem->FirstChildElement("matrix")->Attribute("sid"))
      nt.SetSID(_elem->FirstChildElement("matrix")->Attribute("sid"));
    _node->AddRawTransform(nt);
  }
  else
  {
    if (_elem->FirstChildElement("translate"))
    {
      std::string transStr = _elem->FirstChildElement("translate")->GetText();
      math::Vector3 translate;
      translate = boost::lexical_cast<math::Vector3>(transStr);
      // translate *= this->meter;
      transform.SetTranslate(translate);

      NodeTransform nt(transform);
      if (_elem->FirstChildElement("translate")->Attribute("sid"))
        nt.SetSID(_elem->FirstChildElement("translate")->Attribute("sid"));
      nt.SetType(NodeTransform::TRANSLATE);
      nt.SetSourceValues(translate);
      _node->AddRawTransform(nt);
    }

    TiXmlElement *rotateXml = _elem->FirstChildElement("rotate");
    while (rotateXml)
    {
      math::Matrix3 mat;
      math::Vector3 axis;
      double angle;

      std::string rotateStr = rotateXml->GetText();
      std::istringstream iss(rotateStr);

      iss >> axis.x >> axis.y >> axis.z;
      iss >> angle;
      mat.SetFromAxis(axis, GZ_DTOR(angle));

      math::Matrix4 mat4(math::Matrix4::IDENTITY);
      mat4 = mat;
      NodeTransform nt(mat4);
      if (rotateXml->Attribute("sid"))
        nt.SetSID(rotateXml->Attribute("sid"));
      nt.SetType(NodeTransform::ROTATE);
      nt.SetSourceValues(axis, angle);
      _node->AddRawTransform(nt);

      transform = transform * mat;

      rotateXml = rotateXml->NextSiblingElement("rotate");
    }

    if (_elem->FirstChildElement("scale"))
    {
      std::string scaleStr = _elem->FirstChildElement("scale")->GetText();
      math::Vector3 scale;
      scale = boost::lexical_cast<math::Vector3>(scaleStr);
      math::Matrix4 scaleMat;
      scaleMat.SetScale(scale);

      NodeTransform nt(scaleMat);
      if (_elem->FirstChildElement("matrix")->Attribute("sid"))
        nt.SetSID(_elem->FirstChildElement("matrix")->Attribute("sid"));
      nt.SetType(NodeTransform::SCALE);
      nt.SetSourceValues(scale);
      _node->AddRawTransform(nt);

      transform = transform * scaleMat;
    }
  }

  _node->SetTransform(transform);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadGeometry(TiXmlElement *_xml,
                                 const math::Matrix4 &_transform, Mesh *_mesh)
{
  TiXmlElement *meshXml = _xml->FirstChildElement("mesh");
  TiXmlElement *childXml;

  if (!meshXml)
    return;

  childXml = meshXml->FirstChildElement("triangles");
  while (childXml)
  {
    this->LoadTriangles(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("triangles");
  }

  childXml = meshXml->FirstChildElement("polylist");
  while (childXml)
  {
    this->LoadPolylist(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("polylist");
  }

  childXml = meshXml->FirstChildElement("lines");
  while (childXml)
  {
    this->LoadLines(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("lines");
  }
}

/////////////////////////////////////////////////
TiXmlElement *ColladaLoader::GetElementId(const std::string &_name,
                                          const std::string &_id)
{
  return this->GetElementId(this->colladaXml, _name, _id);
}

/////////////////////////////////////////////////
TiXmlElement *ColladaLoader::GetElementId(TiXmlElement *_parent,
    const std::string &_name,
    const std::string &_id)
{
  std::string id = _id;
  if (id.length() > 0 && id[0] == '#')
    id.erase(0, 1);

  if ((id.empty() && _parent->Value() == _name) ||
      (_parent->Attribute("id") && _parent->Attribute("id") == id) ||
      (_parent->Attribute("sid") && _parent->Attribute("sid") == id))
  {
    return _parent;
  }

  TiXmlElement *elem = _parent->FirstChildElement();
  while (elem)
  {
    TiXmlElement *result = this->GetElementId(elem, _name, _id);
    if (result)
    {
      return result;
    }

    elem = elem->NextSiblingElement();
  }

  return NULL;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadVertices(const std::string &_id,
    const math::Matrix4 &_transform,
    std::vector<math::Vector3> &_verts,
    std::vector<math::Vector3> &_norms)
{
  TiXmlElement *verticesXml = this->GetElementId(this->colladaXml,
                                                 "vertices", _id);

  if (!verticesXml)
  {
    gzerr << "Unable to find vertices[" << _id << "] in collada file\n";
    return;
  }

  TiXmlElement *inputXml = verticesXml->FirstChildElement("input");
  while (inputXml)
  {
    std::string semantic = inputXml->Attribute("semantic");
    std::string sourceStr = inputXml->Attribute("source");
    if (semantic == "NORMAL")
    {
      this->LoadNormals(sourceStr, _transform, _norms);
    }
    else if (semantic == "POSITION")
    {
      this->LoadPositions(sourceStr, _transform, _verts);
    }

    inputXml = inputXml->NextSiblingElement("input");
  }
}

/////////////////////////////////////////////////
void ColladaLoader::LoadPositions(const std::string &_id,
    const math::Matrix4 &_transform,
    std::vector<math::Vector3> &_values)
{
  TiXmlElement *sourceXml = this->GetElementId("source", _id);
  TiXmlElement *floatArrayXml = sourceXml->FirstChildElement("float_array");
  if (!floatArrayXml)
  {
    gzerr << "Vertex source missing float_array element\n";
    return;
  }
  std::string valueStr = floatArrayXml->GetText();

  std::vector<std::string> strs;
  std::vector<std::string>::iterator iter, end;
  boost::split(strs, valueStr, boost::is_any_of("   "));

  end = strs.end();
  for (iter = strs.begin(); iter != end; iter += 3)
  {
    math::Vector3 vec(math::parseFloat(*iter), math::parseFloat(*(iter+1)),
        math::parseFloat(*(iter+2)));
    vec = _transform * vec;
    _values.push_back(vec);
  }
}

/////////////////////////////////////////////////
void ColladaLoader::LoadNormals(const std::string &_id,
    const math::Matrix4 &_transform,
    std::vector<math::Vector3> &_values)
{
  math::Matrix4 rotMat = _transform;
  rotMat.SetTranslate(math::Vector3::Zero);

  TiXmlElement *normalsXml = this->GetElementId("source", _id);
  if (!normalsXml)
  {
    gzerr << "Unable to find normals[" << _id << "] in collada file\n";
    return;
  }

  TiXmlElement *floatArrayXml = normalsXml->FirstChildElement("float_array");
  if (!floatArrayXml)
  {
    gzwarn << "Normal source missing float_array element\n";
    return;
  }

  std::string valueStr = floatArrayXml->GetText();
  std::istringstream iss(valueStr);
  do
  {
    math::Vector3 vec;
    iss >> vec.x >> vec.y >> vec.z;
    if (iss)
    {
      vec = rotMat * vec;
      vec.Normalize();
      _values.push_back(vec);
    }
  } while (iss);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadTexCoords(const std::string &_id,
                                  std::vector<math::Vector2d> &_values)
{
  int stride = 0;
  int texCount = 0;
  int totCount = 0;

  // Get the source element for the texture coordinates.
  TiXmlElement *xml = this->GetElementId("source", _id);
  if (!xml)
  {
    gzerr << "Unable to find tex coords[" << _id << "] in collada file\n";
    return;
  }

  // Get the array of float values. These are the raw values for the texture
  // coordinates.
  TiXmlElement *floatArrayXml = xml->FirstChildElement("float_array");
  if (!floatArrayXml)
  {
    gzerr << "Normal source missing float_array element\n";
    return;
  }

  // The technique_common holds an <accessor> element that indicates how to
  // parse the float array.
  xml = xml->FirstChildElement("technique_common");
  if (!xml)
  {
    gzerr << "Unable to find technique_common element for texture "
          << "coordinates with id[" << _id << "]\n";
    return;
  }

  // Get the accessor XML element.
  xml = xml->FirstChildElement("accessor");
  if (!xml)
  {
    gzerr << "Unable to find <accessor> as a child of <technique_common> "
          << "for texture coordinates with id[" << _id << "]\n";
    return;
  }

  // Read in the total number of texture coordinate values
  if (floatArrayXml->Attribute("count"))
    totCount = boost::lexical_cast<int>(floatArrayXml->Attribute("count"));
  else
  {
    gzerr << "<float_array> has no count attribute in texture coordinate "
          << "element with id[" << _id << "]\n";
    return;
  }

  // Read in the stride for the texture coordinate values. The stride
  // indicates the number of values in the float array the comprise
  // a complete texture coordinate.
  if (xml->Attribute("stride"))
    stride = boost::lexical_cast<int>(xml->Attribute("stride"));
  else
  {
    gzerr << "<accessor> has no stride attribute in texture coordinate element "
          << "with id[" << _id << "]\n";
    return;
  }

  // Read in the count of texture coordinates.
  if (xml->Attribute("count"))
    texCount = boost::lexical_cast<int>(xml->Attribute("count"));
  else
  {
    gzerr << "<accessor> has no count attribute in texture coordinate element "
          << "with id[" << _id << "]\n";
    return;
  }

  // \TODO This is a good a GZ_ASSERT
  // The total number of texture values should equal the stride multiplied
  // by the number of texture coordinates.
  if (texCount * stride != totCount)
  {
    gzerr << "Error reading texture coordinates. Coordinate counts in element "
             "with id[" << _id << "] do not add up correctly\n";
    return;
  }

  // Read the raw texture values, and split them on spaces.
  std::string valueStr = floatArrayXml->GetText();
  std::vector<std::string> values;
  boost::split(values, valueStr, boost::is_any_of(" "));

  // Read in all the texture coordinates.
  for (int i = 0; i < totCount; i += stride)
  {
    // We only handle 2D texture coordinates right now.
    _values.push_back(math::Vector2d(boost::lexical_cast<double>(values[i]),
          1.0 - boost::lexical_cast<double>(values[i+1])));
  }
}

/////////////////////////////////////////////////
Material *ColladaLoader::LoadMaterial(const std::string &_name)
{
  TiXmlElement *matXml = this->GetElementId("material", _name);
  if (!matXml || !matXml->FirstChildElement("instance_effect"))
    return NULL;

  Material *mat = new Material();
  std::string effectName =
    matXml->FirstChildElement("instance_effect")->Attribute("url");
  TiXmlElement *effectXml = this->GetElementId("effect", effectName);

  TiXmlElement *commonXml = effectXml->FirstChildElement("profile_COMMON");
  if (commonXml)
  {
    TiXmlElement *techniqueXml = commonXml->FirstChildElement("technique");
    TiXmlElement *lambertXml = techniqueXml->FirstChildElement("lambert");

    TiXmlElement *phongXml = techniqueXml->FirstChildElement("phong");
    TiXmlElement *blinnXml = techniqueXml->FirstChildElement("blinn");
    if (lambertXml)
    {
      this->LoadColorOrTexture(lambertXml, "ambient", mat);
      this->LoadColorOrTexture(lambertXml, "emission", mat);
      this->LoadColorOrTexture(lambertXml, "diffuse", mat);
      if (lambertXml->FirstChildElement("transparency"))
      {
        mat->SetTransparency(
            this->LoadFloat(lambertXml->FirstChildElement("transparency")));
      }

      if (lambertXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = lambertXml->FirstChildElement("transparent");
        this->LoadTransparent(transXml, mat);
      }
    }
    else if (phongXml)
    {
      this->LoadColorOrTexture(phongXml, "ambient", mat);
      this->LoadColorOrTexture(phongXml, "emission", mat);
      this->LoadColorOrTexture(phongXml, "specular", mat);
      this->LoadColorOrTexture(phongXml, "diffuse", mat);
      if (phongXml->FirstChildElement("shininess"))
        mat->SetShininess(
            this->LoadFloat(phongXml->FirstChildElement("shininess")));

      if (phongXml->FirstChildElement("transparency"))
        mat->SetTransparency(
            this->LoadFloat(phongXml->FirstChildElement("transparency")));
      if (phongXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = phongXml->FirstChildElement("transparent");
        this->LoadTransparent(transXml, mat);
      }
    }
    else if (blinnXml)
    {
      this->LoadColorOrTexture(blinnXml, "ambient", mat);
      this->LoadColorOrTexture(blinnXml, "emission", mat);
      this->LoadColorOrTexture(blinnXml, "specular", mat);
      this->LoadColorOrTexture(blinnXml, "diffuse", mat);
      if (blinnXml->FirstChildElement("shininess"))
        mat->SetShininess(
            this->LoadFloat(blinnXml->FirstChildElement("shininess")));

      if (blinnXml->FirstChildElement("transparency"))
        mat->SetTransparency(
            this->LoadFloat(blinnXml->FirstChildElement("transparency")));
      if (blinnXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = blinnXml->FirstChildElement("transparent");
        this->LoadTransparent(transXml, mat);
      }
    }
  }

  TiXmlElement *glslXml = effectXml->FirstChildElement("profile_GLSL");
  if (glslXml)
    gzerr << "profile_GLSL unsupported\n";

  TiXmlElement *cgXml = effectXml->FirstChildElement("profile_CG");
  if (cgXml)
    gzerr << "profile_CG unsupported\n";
  return mat;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadColorOrTexture(TiXmlElement *_elem,
    const std::string &_type, Material *_mat)
{
  if (!_elem || !_elem->FirstChildElement(_type))
    return;

  TiXmlElement *typeElem = _elem->FirstChildElement(_type);

  if (typeElem->FirstChildElement("color"))
  {
    std::string colorStr = typeElem->FirstChildElement("color")->GetText();
    Color color = boost::lexical_cast<Color>(colorStr);
    if (_type == "diffuse")
      _mat->SetDiffuse(color);
    else if (_type == "ambient")
      _mat->SetAmbient(color);
    else if (_type == "emission")
      _mat->SetEmissive(color);
  }
  else if (typeElem->FirstChildElement("texture"))
  {
    _mat->SetLighting(true);
    TiXmlElement *imageXml = NULL;
    std::string textureName =
      typeElem->FirstChildElement("texture")->Attribute("texture");
    TiXmlElement *textureXml = this->GetElementId("newparam", textureName);
    if (textureXml)
    {
      if (std::string(textureXml->Value()) == "image")
      {
        imageXml = textureXml;
      }
      else
      {
        TiXmlElement *sampler = textureXml->FirstChildElement("sampler2D");
        if (sampler)
        {
          std::string sourceName =
            sampler->FirstChildElement("source")->GetText();
          TiXmlElement *sourceXml = this->GetElementId("newparam", sourceName);
          if (sourceXml)
          {
            TiXmlElement *surfaceXml = sourceXml->FirstChildElement("surface");
            if (surfaceXml && surfaceXml->FirstChildElement("init_from"))
            {
              imageXml = this->GetElementId("image",
                  surfaceXml->FirstChildElement("init_from")->GetText());
            }
          }
        }
      }
    }
    else
    {
      imageXml = this->GetElementId("image", textureName);
    }

    if (imageXml && imageXml->FirstChildElement("init_from"))
    {
      std::string imgFile =
        imageXml->FirstChildElement("init_from")->GetText();
      _mat->SetTextureImage(imgFile, this->path);
    }
  }
}

/////////////////////////////////////////////////
void ColladaLoader::LoadPolylist(TiXmlElement *_polylistXml,
    const math::Matrix4 &_transform,
    Mesh *_mesh)
{
  // This function parses polylist types in collada into
  // a set of triangle meshes.  The assumption is that
  // each polylist polygon is convex, and we do decomposion
  // by anchoring each triangle about vertex 0 or each polygon
  SubMesh *subMesh = new SubMesh;
  subMesh->SetName(this->currentNodeName);
  bool combinedVertNorms = false;

  subMesh->SetPrimitiveType(SubMesh::TRIANGLES);

  if (_polylistXml->Attribute("material"))
  {
    std::map<std::string, std::string>::iterator iter;
    std::string matStr = _polylistXml->Attribute("material");

    iter = this->materialMap.find(matStr);
    if (iter != this->materialMap.end())
      matStr = iter->second;

    int matIndex = _mesh->AddMaterial(this->LoadMaterial(matStr));
    if (matIndex < 0)
      gzwarn << "Unable to add material[" << matStr << "]\n";
    else
      subMesh->SetMaterialIndex(matIndex);
  }

  TiXmlElement *polylistInputXml = _polylistXml->FirstChildElement("input");

  std::vector<math::Vector3> verts;
  std::vector<math::Vector3> norms;
  std::vector<math::Vector2d> texcoords;

  math::Matrix4 bindShapeMat(math::Matrix4::IDENTITY);
  if (_mesh->HasSkeleton())
    bindShapeMat = _mesh->GetSkeleton()->GetBindShapeTransform();

  // read input elements
  std::map<std::string, int> inputs;
  while (polylistInputXml)
  {
    std::string semantic = polylistInputXml->Attribute("semantic");
    std::string source = polylistInputXml->Attribute("source");
    std::string offset = polylistInputXml->Attribute("offset");
    if (semantic == "VERTEX")
    {
      unsigned int count = norms.size();
      this->LoadVertices(source, _transform, verts, norms);
      if (norms.size() > count)
        combinedVertNorms = true;
    }
    else if (semantic == "NORMAL")
    {
      this->LoadNormals(source, _transform, norms);
      combinedVertNorms = false;
    }
    else if (semantic == "TEXCOORD")
      this->LoadTexCoords(source, texcoords);

    inputs[semantic] = math::parseInt(offset);

    polylistInputXml = polylistInputXml->NextSiblingElement("input");
  }

  // read vcount
  // break poly into triangles
  // if vcount >= 4, anchor around 0 (note this is bad for concave elements)
  //   e.g. if vcount = 4, break into triangle 1: [0,1,2], triangle 2: [0,2,3]
  std::vector<std::string> vcountStrs;
  TiXmlElement *vcountXml = _polylistXml->FirstChildElement("vcount");
  std::string vcountStr = vcountXml->GetText();
  boost::split(vcountStrs, vcountStr, boost::is_any_of("   "));
  std::vector<int> vcounts;
  for (unsigned int j = 0; j < vcountStrs.size(); ++j)
    vcounts.push_back(math::parseInt(vcountStrs[j]));

  // read p
  TiXmlElement *pXml = _polylistXml->FirstChildElement("p");
  std::string pStr = pXml->GetText();

  std::vector<math::Vector3> vertNorms(verts.size());
  std::vector<int> vertNormsCounts(verts.size());
  std::fill(vertNormsCounts.begin(), vertNormsCounts.end(), 0);

  int *values = new int[inputs.size()];
  std::map<std::string, int>::iterator end = inputs.end();
  std::map<std::string, int>::iterator iter;
  math::Vector2d vec;

  std::vector<std::string> strs;
  boost::split(strs, pStr, boost::is_any_of("   "));
  std::vector<std::string>::iterator strs_iter = strs.begin();
  for (unsigned int l = 0; l < vcounts.size(); ++l)
  {
    // put us at the beginning of the polygon list
    if (l > 0) strs_iter += inputs.size()*vcounts[l-1];

    for (unsigned int k = 2; k < (unsigned int)vcounts[l]; ++k)
    {
      // if vcounts[l] = 5, then read 0,1,2, then 0,2,3, 0,3,4,...
      // here k = the last number in the series
      // j is the triangle loop
      for (unsigned int j = 0; j < 3; ++j)
      {
        // break polygon into triangles
        unsigned int triangle_index;

        if (j == 0)
          triangle_index = 0;
        if (j == 1)
          triangle_index = (k-1)*inputs.size();
        if (j == 2)
          triangle_index = (k)*inputs.size();

        for (unsigned int i = 0; i < inputs.size(); i++)
        {
          values[i] = math::parseInt(strs_iter[triangle_index+i]);
          /*gzerr << "debug parsing "
                << " poly-i[" << l
                << "] tri-end-index[" << k
                << "] tri-vertex-i[" << j
                << "] triangle[" << triangle_index
                << "] input[" << i
                << "] value[" << values[i]
                << "]\n"; */
        }


        for (iter = inputs.begin(); iter != end; ++iter)
        {
          if (iter->first == "VERTEX")
          {
            subMesh->AddVertex(bindShapeMat * verts[values[iter->second]]);
            subMesh->AddIndex(subMesh->GetVertexCount()-1);
            if (combinedVertNorms)
              subMesh->AddNormal(norms[values[iter->second]]);
            if (_mesh->HasSkeleton())
            {
              Skeleton *skel = _mesh->GetSkeleton();
              for (unsigned int i = 0;
                  i < skel->GetNumVertNodeWeights(values[iter->second]); i++)
              {
                std::pair<std::string, double> node_weight =
                              skel->GetVertNodeWeight(values[iter->second], i);
                SkeletonNode *node =
                  _mesh->GetSkeleton()->GetNodeByName(node_weight.first);
                subMesh->AddNodeAssignment(subMesh->GetVertexCount()-1,
                            node->GetHandle(), node_weight.second);
              }
            }
          }
          else if (iter->first == "NORMAL")
          {
            subMesh->AddNormal(norms[values[iter->second]]);
          }
          else if (iter->first == "TEXCOORD")
          {
            subMesh->AddTexCoord(texcoords[values[iter->second]].x,
                texcoords[values[iter->second]].y);
          }
          // else
          // gzerr << "Unhandled semantic[" << iter->first << "]\n";
        }
      }
    }
  }
  delete [] values;

  _mesh->AddSubMesh(subMesh);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadTriangles(TiXmlElement *_trianglesXml,
                                  const math::Matrix4 &_transform,
                                  Mesh *_mesh)
{
  SubMesh *subMesh = new SubMesh;
  subMesh->SetName(this->currentNodeName);
  bool combinedVertNorms = false;

  subMesh->SetPrimitiveType(SubMesh::TRIANGLES);

  if (_trianglesXml->Attribute("material"))
  {
    std::map<std::string, std::string>::iterator iter;
    std::string matStr = _trianglesXml->Attribute("material");

    iter = this->materialMap.find(matStr);
    if (iter != this->materialMap.end())
      matStr = iter->second;

    int matIndex = _mesh->AddMaterial(this->LoadMaterial(matStr));
    if (matIndex < 0)
      gzwarn << "Unable to add material[" << matStr << "]\n";
    else
      subMesh->SetMaterialIndex(matIndex);
  }

  TiXmlElement *trianglesInputXml = _trianglesXml->FirstChildElement("input");

  std::vector<math::Vector3> verts;
  std::vector<math::Vector3> norms;
  std::vector<math::Vector2d> texcoords;

  // A list of all the input values.
  std::list<std::pair<std::string, int> > inputs;
  while (trianglesInputXml)
  {
    std::string semantic = trianglesInputXml->Attribute("semantic");
    std::string source = trianglesInputXml->Attribute("source");
    std::string offset = trianglesInputXml->Attribute("offset");
    if (semantic == "VERTEX")
    {
      unsigned int count = norms.size();
      this->LoadVertices(source, _transform, verts, norms);
      if (norms.size() > count)
        combinedVertNorms = true;
    }
    else if (semantic == "NORMAL")
    {
      this->LoadNormals(source, _transform, norms);
      combinedVertNorms = false;
    }
    else if (semantic == "TEXCOORD")
      this->LoadTexCoords(source, texcoords);

    inputs.push_back(std::make_pair(semantic, math::parseInt(offset)));

    trianglesInputXml = trianglesInputXml->NextSiblingElement("input");
  }

  TiXmlElement *pXml = _trianglesXml->FirstChildElement("p");
  if (!pXml || !pXml->GetText())
  {
    gzerr << "Collada file[" << this->filename
          << "] is invalid. Loading what we can...\n";
    return;
  }
  std::string pStr = pXml->GetText();

  std::vector<math::Vector3> vertNorms(verts.size());
  std::vector<int> vertNormsCounts(verts.size());
  std::fill(vertNormsCounts.begin(), vertNormsCounts.end(), 0);

  int *values = new int[inputs.size()];
  std::list<std::pair<std::string, int> >::iterator end = inputs.end();
  std::list<std::pair<std::string, int> >::iterator iter;
  math::Vector2d vec;

  std::vector<std::string> strs;
  boost::split(strs, pStr, boost::is_any_of("   "));

  for (unsigned int j = 0; j < strs.size(); j += inputs.size())
  {
    for (unsigned int i = 0; i < inputs.size(); i++)
      values[i] = math::parseInt(strs[j+i]);

    bool already = false;
    for (iter = inputs.begin(); iter != end; ++iter)
    {
      if ((*iter).first == "VERTEX")
      {
        subMesh->AddVertex(verts[values[(*iter).second]]);
        subMesh->AddIndex(subMesh->GetVertexCount()-1);
        if (combinedVertNorms)
          subMesh->AddNormal(norms[values[(*iter).second]]);
        if (_mesh->HasSkeleton())
        {
          Skeleton *skel = _mesh->GetSkeleton();
          for (unsigned int i = 0;
                  i < skel->GetNumVertNodeWeights(values[(*iter).second]); i++)
          {
            std::pair<std::string, double> node_weight =
              skel->GetVertNodeWeight(values[(*iter).second], i);
            SkeletonNode *node =
                _mesh->GetSkeleton()->GetNodeByName(node_weight.first);
            subMesh->AddNodeAssignment(subMesh->GetVertexCount()-1,
                            node->GetHandle(), node_weight.second);
          }
        }
      }
      else if ((*iter).first == "NORMAL")
      {
        subMesh->AddNormal(norms[values[(*iter).second]]);
      }
      else if ((*iter).first == "TEXCOORD" && !already)
      {
        already = true;
        subMesh->AddTexCoord(texcoords[values[(*iter).second]].x,
            texcoords[values[(*iter).second]].y);
      }
      // else
      // gzerr << "Unhandled semantic[" << (*iter).first << "]\n";
    }
  }
  delete [] values;

  _mesh->AddSubMesh(subMesh);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadLines(TiXmlElement *_xml,
    const math::Matrix4 &_transform,
    Mesh *_mesh)
{
  SubMesh *subMesh = new SubMesh;
  subMesh->SetName(this->currentNodeName);
  subMesh->SetPrimitiveType(SubMesh::LINES);

  TiXmlElement *inputXml = _xml->FirstChildElement("input");
  // std::string semantic = inputXml->Attribute("semantic");
  std::string source = inputXml->Attribute("source");

  std::vector<math::Vector3> verts;
  std::vector<math::Vector3> norms;
  this->LoadVertices(source, _transform, verts, norms);

  TiXmlElement *pXml = _xml->FirstChildElement("p");
  std::string pStr = pXml->GetText();
  std::istringstream iss(pStr);

  do
  {
    int a, b;
    iss >> a >> b;

    if (!iss)
      break;
    subMesh->AddVertex(verts[a]);
    subMesh->AddIndex(subMesh->GetVertexCount() - 1);
    subMesh->AddVertex(verts[b]);
    subMesh->AddIndex(subMesh->GetVertexCount() - 1);
  } while (iss);

  _mesh->AddSubMesh(subMesh);
}

/////////////////////////////////////////////////
float ColladaLoader::LoadFloat(TiXmlElement *_elem)
{
  float value = 0;

  if (_elem->FirstChildElement("float"))
  {
    value = math::parseFloat(_elem->FirstChildElement("float")->GetText());
  }

  return value;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadTransparent(TiXmlElement *_elem, Material *_mat)
{
  const char *opaqueCStr = _elem->Attribute("opaque");
  if (!opaqueCStr)
  {
    // gzerr << "No Opaque set\n";
    return;
  }

  // TODO: Handle transparent textures
  if (_elem->FirstChildElement("color"))
  {
    const char *colorCStr = _elem->FirstChildElement("color")->GetText();
    if (!colorCStr)
    {
      gzerr << "No color string\n";
      return;
    }

    std::string opaqueStr = opaqueCStr;
    std::string colorStr = colorCStr;
    Color color = boost::lexical_cast<Color>(colorStr);

    double srcFactor = 0;
    double dstFactor = 0;

    if (opaqueStr == "RGB_ZERO")
    {
      srcFactor = color.r * _mat->GetTransparency();
      dstFactor = 1.0 - color.r * _mat->GetTransparency();
    }
    else if (opaqueStr == "A_ONE")
    {
      srcFactor = 1.0 - color.a * _mat->GetTransparency();
      dstFactor = color.a * _mat->GetTransparency();
    }

    _mat->SetBlendFactors(srcFactor, dstFactor);
  }
}
