/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <vector>
#include <gts.h>

#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/common/GTSMeshUtils.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
static void FillVertex(GtsPoint *_p, gpointer *_data)
{
  // create a Gazebo vertex from GTS_POINT and add it to the submesh
  SubMesh *subMesh = reinterpret_cast<SubMesh *>(_data[0]);
  GHashTable* vIndex = reinterpret_cast<GHashTable *>(_data[2]);
  subMesh->AddVertex(GTS_POINT(_p)->x, GTS_POINT(_p)->y, GTS_POINT(_p)->z);
  // fill the hash table which will later be used for adding indices to the
  // submesh in the FillFace function.
  g_hash_table_insert(vIndex, _p,
      GUINT_TO_POINTER((*(reinterpret_cast<guint *>(_data[1])))++));
}

//////////////////////////////////////////////////
static void FillFace(GtsTriangle *_t, gpointer *_data)
{
  SubMesh *subMesh = reinterpret_cast<SubMesh *>(_data[0]);
  GHashTable *vIndex = reinterpret_cast<GHashTable *>(_data[2]);
  int *x = reinterpret_cast<int*>(_data[3]);
  GtsVertex *v1, *v2, *v3;
  gts_triangle_vertices(_t, &v1, &v2, &v3);
  if (*x == 0)
  {
    subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v1))+*x);
    subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v3))+*x);
    subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v2))+*x);
  }
  else
  {
    subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v1))+*x);
    subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v2))+*x);
    subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v3))+*x);
  }
}

//////////////////////////////////////////////////////////////////////////
static void AddConstraint(GtsConstraint *_c, GtsSurface *_s)
{
  gts_delaunay_add_constraint(_s, _c);
}

//////////////////////////////////////////////////
bool GTSMeshUtils::CreateExtrudedPolyline(
    const std::vector<math::Vector2d> &_vertices,
    const double &_height, SubMesh *_subMesh)
{
  if (_vertices.size() < 3)
  {
    gzerr << "Unable to create an extruded polyline mesh with "
      << "less than 3 vertices\n";
    return false;
  }

  if (!_subMesh)
    _subMesh = new SubMesh();

  int i, k;

  int numSides = _vertices.size();

  // Euler's Formula: numFaces = numEdges - numVertices + 2
  //                           = numSides + 2
  // # of SideFaces = numFaces - (upper face + lower face)
  //                = numFaces - 2
  //                = numSides


  GSList *l, *verticesList = NULL;
  double z;
  for (k = 0; k < 2; ++k)
  {
    GtsSurface *surface;
    GtsVertex *v1, *v2, *v3;
    GtsFifo *edgeList;
    edgeList = gts_fifo_new();
    verticesList = NULL;

    if (k == 0)
      z = 0.0;
    else
      z = _height;

    // List the vertices and edges
    for (i = 0; i < numSides; ++i)
    {
      verticesList = g_slist_append(verticesList,
          gts_vertex_new(gts_vertex_class(),
            _vertices[i].x, _vertices[i].y, z));
      if (i != 0)
      {
        gts_fifo_push(edgeList,
            gts_edge_new(GTS_EDGE_CLASS(gts_constraint_class()),
              reinterpret_cast<GtsVertex *>
              (g_slist_nth_data(verticesList, i)),
              reinterpret_cast<GtsVertex *>
              (g_slist_nth_data(verticesList, i-1))));
      }
    }

    gts_fifo_push(edgeList,
        gts_edge_new(GTS_EDGE_CLASS(gts_constraint_class()),
          reinterpret_cast<GtsVertex *>
          (g_slist_nth_data(verticesList, i-1)),
          reinterpret_cast<GtsVertex *>
          (g_slist_nth_data(verticesList, 0))));

    GtsTriangle *tri = gts_triangle_enclosing(
        gts_triangle_class(), verticesList, 100.);
    gts_triangle_vertices(tri, &v1, &v2, &v3);

    surface = gts_surface_new(gts_surface_class(),
        gts_face_class(),
        gts_edge_class(),
        gts_vertex_class());

    gts_surface_add_face(surface,
        gts_face_new(gts_face_class(),
          tri->e1, tri->e2, tri->e3));

    l = verticesList;
    while (l)
    {
      GtsVertex *v_in = reinterpret_cast<GtsVertex *>(l->data);
      GtsVertex *v_out = gts_delaunay_add_vertex(surface, v_in, NULL);
      if (v_out != NULL)
      {
        gts_vertex_replace(v_in, v_out);
      }
      l = l->next;
    }

    // add constraints
    gts_fifo_foreach(edgeList, (GtsFunc) AddConstraint, surface);

    // delete the enclosing triangle
    gts_allow_floating_vertices = TRUE;
    gts_object_destroy(GTS_OBJECT(v1));
    gts_object_destroy(GTS_OBJECT(v2));
    gts_object_destroy(GTS_OBJECT(v3));
    gts_allow_floating_vertices = FALSE;

    // Remove edges on the boundary which are not constraints
    gts_delaunay_remove_hull(surface);

    // fill the submesh with data generated by GTS
    unsigned int n2 = 0, m = numSides*k;
    gpointer data[4];
    GHashTable *vIndex = g_hash_table_new(NULL, NULL);

    data[0] = _subMesh;
    data[1] = &n2;
    data[2] = vIndex;
    data[3] = &m;
    gts_surface_foreach_vertex(surface, (GtsFunc) FillVertex, data);
    n2 = 0;
    gts_surface_foreach_face(surface, (GtsFunc) FillFace, data);

    g_hash_table_destroy(vIndex);
    gts_object_destroy(GTS_OBJECT(surface));
    gts_fifo_destroy(edgeList);
  }

  return true;
}
