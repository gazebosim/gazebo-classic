/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <vector>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Road2d.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief A road segment
    class RoadSegment
    {
      /// \brief Load the road segment from message data.
      /// \param[in] _msg The robot data.
      public: void Load(msgs::Road _msg);

      /// \brief Name of the road segment.
      public: std::string name;

      /// \brief Points that make up the middle of the road.
      public: std::vector<ignition::math::Vector3d> points;

      /// \brief Width of the road.
      public: double width;

      /// \brief Texture of the road
      public: std::string texture;
    };

    /// \brief Private data for the Road2d class.
    class Road2dPrivate : public VisualPrivate
    {
      /// \brief All the road segments.
      public: std::vector<RoadSegment> segments;
    };
  }
}

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
Road2d::Road2d()
  : Visual(*new Road2dPrivate, "roads", rendering::get_scene())
{
}

/////////////////////////////////////////////////
Road2d::Road2d(const std::string &_name, VisualPtr _parent)
  : Visual(*new Road2dPrivate, _name, _parent)
{
  Road2dPrivate *dPtr =
      reinterpret_cast<Road2dPrivate *>(this->dataPtr);

  dPtr->type = VT_VISUAL;
}

/////////////////////////////////////////////////
Road2d::~Road2d()
{
  Road2dPrivate *dPtr =
      reinterpret_cast<Road2dPrivate *>(this->dataPtr);

  dPtr->segments.clear();
}

/////////////////////////////////////////////////
void Road2d::Load(VisualPtr /*_parent*/)
{
  // This function is deprecated. Remove in gazebo9
}

//////////////////////////////////////////////////
void Road2d::Load(msgs::Road _msg)
{
  this->Load();

  Road2dPrivate *dPtr =
      reinterpret_cast<Road2dPrivate *>(this->dataPtr);

  RoadSegment segment;
  segment.Load(_msg);

  Ogre::MovableObject *obj = dynamic_cast<Ogre::MovableObject *>
      (this->GetSceneNode()->getCreator()->createEntity(
      segment.name, segment.name));
  obj->setRenderQueueGroup(obj->getRenderQueueGroup()+1);
  this->AttachObject(obj);
  dPtr->segments.push_back(segment);

  // make the road visual not selectable
  this->SetVisibilityFlags(GZ_VISIBILITY_ALL & (~GZ_VISIBILITY_SELECTABLE));
}

/////////////////////////////////////////////////
void RoadSegment::Load(msgs::Road _msg)
{
  this->width = _msg.width();

  for (int i = 0; i < _msg.point_size(); ++i)
  {
    this->points.push_back(msgs::ConvertIgn(_msg.point(i)));
  }

  this->name = _msg.name();

  /// Create the mesh
  Ogre::MeshPtr mesh =
      Ogre::MeshManager::getSingleton().createManual(this->name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


  Ogre::SubMesh *subMesh = mesh->createSubMesh();
  subMesh->useSharedVertices = false;
  subMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
  subMesh->vertexData = new Ogre::VertexData();
  Ogre::VertexData *vertexData = subMesh->vertexData;
  Ogre::VertexDeclaration *vertexDecl = vertexData->vertexDeclaration;

  size_t currOffset = 0;

  // Positions
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // Normals
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // two dimensional ->roadObjtexture coordinates
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                         Ogre::VES_TEXTURE_COORDINATES, 0);

  // allocate the vertex buffer
  vertexData->vertexCount = this->points.size() * 2;
  Ogre::HardwareVertexBufferSharedPtr vBuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        vertexDecl->getVertexSize(0), vertexData->vertexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);

  Ogre::VertexBufferBinding *binding = vertexData->vertexBufferBinding;
  binding->setBinding(0, vBuf);
  float *vertices = static_cast<float*>(
      vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // index buffer
  subMesh->indexData->indexCount = vertexData->vertexCount;
  subMesh->indexData->indexBuffer =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_32BIT,
      subMesh->indexData->indexCount,
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
      false);
  Ogre::HardwareIndexBufferSharedPtr iBuf = subMesh->indexData->indexBuffer;
  uint32_t *indices = static_cast<uint32_t *>(
      iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  ignition::math::Vector3d pA, pB, tangent;

  math::Box bounds;
  bounds.min.Set(GZ_DBL_MAX, GZ_DBL_MAX, GZ_DBL_MAX);
  bounds.max.Set(GZ_DBL_MIN, GZ_DBL_MIN, GZ_DBL_MIN);

  // length for each texture tile, same as road width as texture is square
  // (if texture size should change or made custom in a future version
  // there needs to be code to handle this)
  double texMaxLen = this->width;

  // current road length
  double curLen = 0.0;

  // Generate the triangles for the road
  for (unsigned int i = 0; i < this->points.size(); ++i)
  {
    double factor = 1.0;

    // update current road length
    if (i > 0)
    {
      curLen += this->points[i].Distance(this->points[i-1]);
    }

    // assign texture coordinate as percentage of texture tile size
    // and let ogre/opengl handle the texture wrapping
    float texCoord = curLen/texMaxLen;

    // Start point is a special case
    if (i == 0)
    {
      tangent = (this->points[i+1] - this->points[i]).Normalize();
    }
    // End point is a special case
    else if (i == this->points.size() - 1)
    {
      tangent = (this->points[i] - this->points[i-1]).Normalize();
    }
    // Every other point in the road
    else
    {
      math::Vector3 v1 = (this->points[i+1] - this->points[i]).Normalize();
      math::Vector3 v0 = (this->points[i] - this->points[i-1]).Normalize();
      double dot = v0.Dot(v1 * -1);
      tangent = (v1+v0).Normalize().Ign();

      // Check to see if the points are not colinear
      // If not colinear, then the road needs to be widended for the turns
      if (!ignition::math::equal(fabs(dot), 1.0))
      {
        factor = 1.0 / sin(acos(dot) * 0.5);
      }
    }

    // The tangent is used to calculate the two verteces to either side of
    // the point. The vertices define the triangle mesh of the road
    double theta = atan2(tangent.X(), -tangent.Y());

    pA = pB = this->points[i];
    double w = (this->width * factor) * 0.5;

    pA.X() += cos(theta) * w;
    pA.Y() += sin(theta) * w;

    pB.X() -= cos(theta) * w;
    pB.Y() -= sin(theta) * w;

    bounds.min.SetToMin(pA);
    bounds.min.SetToMin(pB);

    bounds.max.SetToMax(pA);
    bounds.max.SetToMax(pB);

    // Position
    *vertices++ = pA.X();
    *vertices++ = pA.Y();
    *vertices++ = pA.Z();

    // Normal
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = 1;

    // UV Coord
    *vertices++ = 0;
    *vertices++ = texCoord;

    // Position
    *vertices++ = pB.X();
    *vertices++ = pB.Y();
    *vertices++ = pB.Z();

    // Normal
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = 1;

    // UV Coord
    *vertices++ = 1;
    *vertices++ = texCoord;
  }

  // Add all the indices
  for (unsigned int i = 0; i < subMesh->indexData->indexCount; ++i)
    *indices++ = i;

  // unlock
  vBuf->unlock();
  iBuf->unlock();

  mesh->_setBounds(Ogre::AxisAlignedBox(Conversions::Convert(bounds.min.Ign()),
                   Conversions::Convert(bounds.max.Ign())));

  if (_msg.has_material())
  {
    if (_msg.material().has_script())
    {
      for (int i = 0; i < _msg.material().script().uri_size(); ++i)
      {
        std::string matUri = _msg.material().script().uri(i);
        if (!matUri.empty())
          RenderEngine::Instance()->AddResourcePath(matUri);
      }

      std::string matName = _msg.material().script().name();

      if (!matName.empty())
        subMesh->setMaterialName(matName);
     }
  }
  else
  {
    subMesh->setMaterialName("Gazebo/Road");
  }

  mesh->load();
}
