/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <sdf/sdf.hh>
#include "gazebo/math/Pose.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Material.hh"

std::vector<std::string> params;

// To Render high quality: povray +H480 +W640 +R5 +AM2 +A0.3 +Q11 <filename>

// Convert radians to degrees
#define RTOD(r) ((r) * 180 / M_PI)

// Convert degrees to radians
#define DTOR(d) ((d) * M_PI / 180)

void help()
{
  printf("help");
}

bool parse(int argc, char **argv)
{
  if (argc == 1 || std::string(argv[1]) == "help")
  {
    help();
    return false;
  }

  // Get parameters from command line
  for (int i = 1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back(p);
  }

  // Get parameters from stdin
  if (!isatty(fileno(stdin)))
  {
    char str[1024];
    while (!feof(stdin))
    {
      if (fgets(str, 1024, stdin)== NULL)
        break;

      if (feof(stdin))
        break;
      std::string p = str;
      boost::trim(p);
      params.push_back(p);
    }
  }

  return true;
}

gazebo::math::Vector3 Convert(const gazebo::math::Vector3 &_vec)
{
  gazebo::math::Vector3 result;
  gazebo::math::Quaternion rot1(0, M_PI*.5, 0);
  gazebo::math::Quaternion rot2(0, -M_PI*.5, 0);

  result = rot1.RotateVector(_vec);
  result = rot2.RotateVector(result);

  return result;
}

void ProcessMesh(sdf::ElementPtr _elem, const gazebo::math::Pose _pose)
{
  const gazebo::common::Mesh *mesh;

  mesh = gazebo::common::MeshManager::Instance()->Load(
      _elem->GetValueString("filename"));

  const_cast<gazebo::common::Mesh*>(mesh)->RecalculateNormals();


  for (unsigned int i = 0; i < mesh->GetSubMeshCount(); i++)
  {
    const gazebo::common::SubMesh *subMesh = mesh->GetSubMesh(i);
    printf("mesh2 {\n");
    printf("  vertex_vectors {\n");
    printf("    %d, \n    ", subMesh->GetVertexCount());
    for (unsigned int v = 0; v < subMesh->GetVertexCount(); v++)
    {
      gazebo::math::Vector3 vert = subMesh->GetVertex(v);
      // vert = _pose.CoordPositionAdd(vert);
      printf("<%f, %f, %f>, ", vert.x, vert.y, vert.z);
    }
    printf("  }\n");

    printf("  normal_vectors {\n");
    printf("    %d, \n    ", subMesh->GetNormalCount());
    for (unsigned int n = 0; n < subMesh->GetNormalCount(); n++)
    {
      gazebo::math::Vector3 norm = subMesh->GetNormal(n);
      printf("<%f, %f, %f>, ", norm.x, norm.y, norm.z);
    }
    printf("  }\n");

    printf("  uv_vectors {\n");
    printf("    %d, \n", subMesh->GetTexCoordCount());
    for (unsigned int j = 0; j < subMesh->GetTexCoordCount(); j++)
    {
      printf("    <%f, %f>, \n", subMesh->GetTexCoord(j).x,
        1.0 - subMesh->GetTexCoord(j).y);
    }
    printf("  }\n");

    const gazebo::common::Material *mat = mesh->GetMaterial(
        subMesh->GetMaterialIndex());
    if (mat)
    {
      printf("  texture_list {\n");
      printf("    1, \n");
      printf("  texture {\n");
      if (!mat->GetTextureImage().empty())
      {
        printf("    uv_mapping pigment { image_map ");
        printf("{ tiff \"%s\" } }\n",
            mat->GetTextureImage().c_str());
      }
      else
      {
        printf("    pigment { color rgb <%f, %f, %f> }\n",
            mat->GetDiffuse().r, mat->GetDiffuse().g, mat->GetDiffuse().b);
      }

      printf("    finish {\n");
      printf("      ambient color rgb <%f, %f, %f>\n",
          mat->GetAmbient().r, mat->GetAmbient().g, mat->GetAmbient().b);
      printf("      specular %f\n", 1.0);
      printf("    }\n");

      printf("  }\n");
      printf("  }\n");
    }

    printf("  face_indices {\n");
    printf("    %d, \n", subMesh->GetIndexCount() / 3);
    for (unsigned int j = 0; j < subMesh->GetIndexCount(); j+= 3)
    {
      if (mat)
      {
        printf("    <%d, %d, %d>, 0\n", subMesh->GetIndex(j),
            subMesh->GetIndex(j+1), subMesh->GetIndex(j+2));
      }
      else
      {
        printf("    <%d, %d, %d>\n", subMesh->GetIndex(j),
            subMesh->GetIndex(j+1), subMesh->GetIndex(j+2));
      }
    }
    printf("  }\n");

    printf("  normal_indices {\n");
    printf("    %d, \n", subMesh->GetIndexCount() / 3);
    for (unsigned int j = 0; j < subMesh->GetIndexCount(); j+= 3)
    {
      printf("    <%d, %d, %d>, \n", subMesh->GetIndex(j),
        subMesh->GetIndex(j+1), subMesh->GetIndex(j+2));
    }
    printf("  }\n");
/*
    printf("  uv_indices {\n");
    printf("    %d, \n", subMesh->GetIndexCount() / 3);
    for (unsigned int j = 0; j < subMesh->GetIndexCount(); j+= 3)
    {
      printf("    <%d, %d, %d>, \n", subMesh->GetIndex(j),
        subMesh->GetIndex(j+1), subMesh->GetIndex(j+2));
    }
    printf("  }\n");
    */

    gazebo::math::Vector3 rpy = _pose.rot.GetAsEuler();
    printf("  translate <%f, %f, %f>\n", _pose.pos.x, _pose.pos.y, _pose.pos.z);
    printf("  rotate <%f, %f, %f>\n", RTOD(rpy.x), RTOD(rpy.y), RTOD(rpy.z));

    printf("}\n");
  }
}

void ProcessLight(sdf::ElementPtr _elem)
{
  gazebo::math::Pose pose;
  gazebo::common::Color diffuse, specular;

  pose = _elem->GetOrCreateElement("origin")->GetValuePose("pose");
  diffuse = _elem->GetValueColor("diffuse");
  specular = _elem->GetValueColor("specular");
  // double fadeDist =
  // _elem->GetElement("attenuation")->GetValueDouble("range");
  // double constant =
  // _elem->GetElement("attenuation")->GetValueDouble("constant");
  // double linear = _elem->GetElement("attenuation")->GetValueDouble("linear");
  // double quadratic =
  // _elem->GetElement("attenuation")->GetValueDouble("quadratic");

  printf("light_source {\n");
  printf("  <%f, %f, %f>, rgb <%f, %f, %f>\n",
      pose.pos.x, pose.pos.y, pose.pos.z,
      diffuse.r, diffuse.g, diffuse.b);

  std::string type = _elem->GetValueString("type");
  if (type == "point")
  {
    // printf("  pointlight\n");
  }
  else if (type == "directional")
  {
    printf("  parallel\n");
  }
  else if (type == "spot")
  {
    double innerAngle, outerAngle, falloff;
    innerAngle = _elem->GetElement("spot")->GetValueDouble("inner_angle");
    outerAngle = _elem->GetElement("spot")->GetValueDouble("outer_angle");
    falloff = _elem->GetElement("spot")->GetValueDouble("falloff");
    printf("  spotlight\n");
    printf("  radius %f\n", RTOD(innerAngle));
    printf("  falloff %f\n", RTOD(outerAngle));
    printf("  tightness %f\n", falloff);
  }

  if (_elem->HasElement("direction"))
  {
    gazebo::math::Vector3 dir =
      _elem->GetElement("direction")->GetValueVector3("xyz");
    gazebo::math::Plane plane(gazebo::math::Vector3(0, 0, 1));

    double d = plane.Distance(pose.pos, dir);
    double t;
    t = atan2(dir.x, dir.z*-1);
    double x = sin(t) * d;
    t = atan2(dir.y, dir.z*-1);
    double y = sin(t) * d;
    printf("  point_at <%f, %f, 0.0>\n", x, y);
  }

  printf("}\n");
}

void ProcessScene(sdf::ElementPtr _elem)
{
  gazebo::common::Color color;
  if (_elem->HasElement("background"))
  {
    color = _elem->GetValueColor("background");
    printf("background { rgb <%f, %f, %f> }\n", color.r, color.g, color.b);
  }

  if (_elem->HasElement("ambient"))
  {
    color = _elem->GetValueColor("ambient");
    // printf("global_settings { ambient_light rgb <%f, %f, %f> }\n",
        // color.R(), color.G(), color.B());
  }

  // int count = 35;
  int count = 1600;

  // int recursionLimit = 3;
  int recursionLimit = 20;

  // float errorBound = 1.8;
  float errorBound = 1.0;

  // Note: Extreme quality
  printf("global_settings { radiosity{\n");
  printf("  count %d\n", count);
  printf("  recursion_limit %d\n", recursionLimit);
  printf("  error_bound %f\n", errorBound);

  printf("} }\n");
}

void ProcessGeometry(sdf::ElementPtr _elem, const gazebo::math::Pose &_pose)
{
  if (_elem->HasElement("plane"))
  {
    sdf::ElementPtr planeElem = _elem->GetElement("plane");
    gazebo::math::Vector3 normal = planeElem->GetValueVector3("normal");
    printf("plane {\n");
    printf("  <%f, %f, %f>, 0\n", normal.x, normal.y, normal.z);
    printf("  texture {pigment { color Yellow } }\n");
    printf("}\n");
  }
  else if (_elem->HasElement("box"))
  {
    sdf::ElementPtr boxElem = _elem->GetElement("box");
    gazebo::math::Vector3 size = boxElem->GetValueVector3("size");
    printf("box {\n");
    gazebo::math::Vector3 corner1 = _pose.pos - (size/2.0);
    gazebo::math::Vector3 corner2 = _pose.pos + (size/2.0);
    corner1 = _pose.rot.RotateVector(corner1);
    corner2 = _pose.rot.RotateVector(corner2);
    printf(" <%f, %f, %f, >, <%f, %f, %f>\n", corner1.x, corner1.y, corner1.z,
                                            corner2.x, corner2.y, corner2.z);
    printf("}\n");
  }
  else if (_elem->HasElement("cylinder"))
  {
    sdf::ElementPtr cylinderElem = _elem->GetElement("cylinder");
    double radius = cylinderElem->GetValueDouble("radius");
    double length = cylinderElem->GetValueDouble("length");
    gazebo::math::Vector3 capPoint = _pose.pos;
    capPoint.z += length;
    capPoint = _pose.rot.RotateVector(capPoint);
    printf("cylinder {\n");
    printf("  <%f, %f, %f>, <%f, %f, %f>, %f\n",
        _pose.pos.x, _pose.pos.y, _pose.pos.z,
        capPoint.x, capPoint.y, capPoint.z, radius);
    printf("}\n");
  }
  else if (_elem->HasElement("sphere"))
  {
    sdf::ElementPtr sphereElem = _elem->GetElement("sphere");
    double radius = sphereElem->GetValueDouble("radius");

    printf("sphere {\n");
    printf("  <%f, %f, %f> %f\n",
        _pose.pos.x, _pose.pos.y, _pose.pos.z, radius);
    printf("}\n");
  }
  else if (_elem->HasElement("mesh"))
  {
    ProcessMesh(_elem->GetElement("mesh"), _pose);
  }
}


int main(int argc, char **argv)
{
  if (!parse(argc, argv))
    return 0;

  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF);
  if (!sdf::init(sdf))
  {
    gzerr << "Unable to initialize sdf\n";
    return false;
  }

  if (!sdf::readFile(params[0], sdf))
  {
    gzerr << "Unable to read sdf file[" << params[0] << "]\n";
    return false;
  }

  printf("#include \"colors.inc\"\n");
  printf("#include \"shapes.inc\"\n");
  printf("camera {\n");
  printf("  location <-2, 0, 2>\n");
  printf("  look_at <0, 0, 1>\n");
  printf("  sky <0, 0, 1>\n");
  printf("  direction <1, 0, 0>\n");
  printf("  up <0, 0, -1>\n");
  printf("  right <0, 1, 0>\n");
  printf("}\n");

  sdf::ElementPtr root = sdf->root;

  gazebo::math::Pose modelPose, linkPose, visualPose;

  sdf::ElementPtr worldElem = root->GetElement("world");
  while (worldElem)
  {
    if (worldElem->HasElement("scene"))
      ProcessScene(worldElem->GetElement("scene"));

    if (worldElem->HasElement("light"))
    {
      sdf::ElementPtr lightElem = worldElem->GetElement("light");
      while (lightElem)
      {
        ProcessLight(lightElem);
        lightElem = lightElem->GetNextElement("light");
      }
    }

    sdf::ElementPtr modelElem = worldElem->GetElement("model");
    while (modelElem)
    {
      modelPose = modelElem->GetOrCreateElement("origin")->GetValuePose("pose");

      sdf::ElementPtr linkElem = modelElem->GetElement("link");
      while (linkElem)
      {
        linkPose = linkElem->GetOrCreateElement("origin")->GetValuePose("pose");

        if (linkElem->HasElement("visual"))
        {
          sdf::ElementPtr visualElem = linkElem->GetElement("visual");
          while (visualElem)
          {
            visualPose =
              visualElem->GetOrCreateElement("origin")->GetValuePose("pose");
            // visualPose = (visualPose + linkPose) + modelPose;
            visualPose = modelPose + (linkPose + visualPose);
            // visualPose.pos = modelPose.pos + linkPose.pos + visualPose.pos;
            // visualPose.rot = visualPose.rot * linkPose.rot * modelPose.rot;
            sdf::ElementPtr geomElem = visualElem->GetElement("geometry");
            ProcessGeometry(geomElem, visualPose);

            visualElem = visualElem->GetNextElement("visual");
          }
        }
        linkElem = linkElem->GetNextElement("link");
      }
      modelElem = modelElem->GetNextElement("model");
    }
    worldElem = worldElem->GetNextElement("world");
  }
}
