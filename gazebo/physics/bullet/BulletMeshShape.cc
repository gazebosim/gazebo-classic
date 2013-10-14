/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <hacd/hacdHACD.h>

#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletMeshShape.hh"

using namespace gazebo;
using namespace physics;

void HACDCallback(const char *_msg, double /*_progress*/,
    double /*concavity*/, size_t /*nVertices*/)
{
  std::cout << _msg;
}

//////////////////////////////////////////////////
BulletMeshShape::BulletMeshShape(CollisionPtr _parent)
  : MeshShape(_parent)
{
}


//////////////////////////////////////////////////
BulletMeshShape::~BulletMeshShape()
{
}

//////////////////////////////////////////////////
void BulletMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletMeshShape::Init()
{
  MeshShape::Init();

  BulletCollisionPtr bParent =
    boost::static_pointer_cast<BulletCollision>(this->collisionParent);

  float *vertices = NULL;
  int *indices = NULL;

  btTriangleMesh *bulletTriMesh = new btTriangleMesh();

  unsigned int numVertices = this->mesh->GetVertexCount();
  unsigned int numIndices = this->mesh->GetIndexCount();

  // Get all the vertex and index data
  this->mesh->FillArrays(&vertices, &indices);

  // Scale the vertex data
  for (unsigned int j = 0;  j < numVertices; j++)
  {
    vertices[j*3+0] = vertices[j*3+0] *
      this->sdf->Get<math::Vector3>("scale").x;
    vertices[j*3+1] = vertices[j*3+1] *
      this->sdf->Get<math::Vector3>("scale").y;
    vertices[j*3+2] = vertices[j*3+2] *
      this->sdf->Get<math::Vector3>("scale").z;
  }

  std::vector<HACD::Vec3<HACD::Real> > points;
  std::vector<HACD::Vec3<int32_t> > triangles;

  // Create the Bullet trimesh
  for (unsigned int j = 0; j < numIndices; j += 3)
  {
    points.push_back(HACD::Vec3<HACD::Real>(vertices[indices[j]*3+0],
          vertices[indices[j]*3+1],
          vertices[indices[j]*3+2]));

    points.push_back(HACD::Vec3<HACD::Real>(vertices[indices[j+1]*3+0],
          vertices[indices[j+1]*3+1],
          vertices[indices[j+1]*3+2]));

    points.push_back(HACD::Vec3<HACD::Real>(vertices[indices[j+2]*3+0],
          vertices[indices[j+2]*3+1],
          vertices[indices[j+2]*3+2]));

    triangles.push_back(HACD::Vec3<int32_t>(
          points.size()-3, points.size()-2, points.size()-1));
  }

  size_t clusters = 4;
  size_t concavity = 200;
  // bool invert = false;
  bool addExtraDistPoints = false;
  bool addFacesPoints = false;
  double ccConnectDist = 30;
  size_t targetNTrianglesDecimatedMesh = 0;

  HACD::HeapManager *heapManager = HACD::createHeapManager(65536*(1000));
  HACD::HACD *const myHACD = HACD::CreateHACD(heapManager);

  myHACD->SetPoints(&points[0]);
  myHACD->SetNPoints(points.size());
  myHACD->SetTriangles(&triangles[0]);
  myHACD->SetNTriangles(triangles.size());
  myHACD->SetCompacityWeight(0.001);
  myHACD->SetVolumeWeight(0.0);

  // if two connected components are seperated by distance < ccConnectDist
  // then create a virtual edge between them so the can be merged during
  // the simplification process
  myHACD->SetConnectDist(ccConnectDist);

  // minimum number of clusters
  myHACD->SetNClusters(clusters);

  // max of 100 vertices per convex-hull
  myHACD->SetNVerticesPerCH(100);

  // maximum concavity
  myHACD->SetConcavity(concavity);

  // threshold to detect small clusters
  myHACD->SetSmallClusterThreshold(0.25);

  // Number triangles in the decimated mesh
  myHACD->SetNTargetTrianglesDecimatedMesh(targetNTrianglesDecimatedMesh);

  // Get debug output during processing
  // myHACD->SetCallBack(&HACDCallback);

  myHACD->SetAddExtraDistPoints(addExtraDistPoints);
  myHACD->SetAddFacesPoints(addFacesPoints);

  // Do the decomposition.
  myHACD->Compute();

  // Use a compound shape to store all the convex meshes.
  btCompoundShape *compoundShape = new btCompoundShape();

  for (size_t c = 0; c < clusters; ++c)
  {
    // Debug output
    // std::cout << std::endl << "Convex-Hull " << c << std::endl;

    size_t numPoints = myHACD->GetNPointsCH(c);
    size_t numTriangles = myHACD->GetNTrianglesCH(c);

    HACD::Vec3<HACD::Real> *pointsCH = new HACD::Vec3<HACD::Real>[numPoints];
    HACD::Vec3<int32_t> *trianglesCH = new HACD::Vec3<int32_t>[numTriangles];
    myHACD->GetCH(c, pointsCH, trianglesCH);

    // Debug output
    // std::cout << "Points " << numPoints << std::endl;
    // for(size_t v = 0; v < numPoints; ++v)
    // {
    //   std::cout << v << "\t"
    //     << pointsCH[v].X() << "\t"
    //     << pointsCH[v].Y() << "\t"
    //     << pointsCH[v].Z() << std::endl;
    // }
    // std::cout << "Triangles " << numTriangles << std::endl;

    for (size_t f = 0; f < numTriangles; ++f)
    {
      btVector3 bv0(
          pointsCH[trianglesCH[f].X()].X(),
          pointsCH[trianglesCH[f].X()].Y(),
          pointsCH[trianglesCH[f].X()].Z());

      btVector3 bv1(
          pointsCH[trianglesCH[f].Y()].X(),
          pointsCH[trianglesCH[f].Y()].Y(),
          pointsCH[trianglesCH[f].Y()].Z());

      btVector3 bv2(
          pointsCH[trianglesCH[f].Z()].X(),
          pointsCH[trianglesCH[f].Z()].Y(),
          pointsCH[trianglesCH[f].Z()].Z());

      bulletTriMesh->addTriangle(bv0, bv1, bv2);
    }

    // Create a new convex shape.
    btConvexShape *convexShape = new btConvexTriangleMeshShape(
        bulletTriMesh, true);
    convexShape->setMargin(0.001f);

    // Add the convex shape.
    compoundShape->addChildShape(
        BulletTypes::ConvertPose(math::Pose(0, 0, 0, 0, 0, 0)), convexShape);

    delete [] pointsCH;
    delete [] trianglesCH;
  }

  // Set the collision shape.
  bParent->SetCollisionShape(compoundShape);

  HACD::DestroyHACD(myHACD);
  HACD::releaseHeapManager(heapManager);

  delete [] vertices;
  delete [] indices;
}
