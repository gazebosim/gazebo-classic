/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
  GtsVertex *v1, *v2, *v3;
  gts_triangle_vertices(_t, &v1, &v2, &v3);
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v1)));
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v3)));
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup(vIndex, v2)));
}

//////////////////////////////////////////////////////////////////////////
static void AddConstraint(GtsConstraint *_c, GtsSurface *_s)
{
  gts_delaunay_add_constraint(_s, _c);
}

//////////////////////////////////////////////////
static void Intersection(GtsEdge *_c, gpointer *_data)
{
  double x = *reinterpret_cast<double *>(_data[0]);
  double y = *reinterpret_cast<double *>(_data[1]);
  int *intersection = reinterpret_cast<int *>(_data[2]);

  GtsVertex *v1, *v2;
  v1 = _c->segment.v1;
  v2 = _c->segment.v2;

  double x1 = v1->p.x;
  double x2 = v2->p.x;
  double y1 = v1->p.y;
  double y2 = v2->p.y;

  double xmin = std::min(x1, x2);
  double xmax = (x1 + x2) - xmin;
  double ymin = std::min(y1, y2);
  double ymax = (y1 + y2) - ymin;
  double xBound = xmax+1;

  if (y < ymax && y >= ymin)
  {
    double xdiff1, ydiff1, xdiff2, ydiff2;
    xdiff1 = x2 - x1;
    ydiff1 = y2 - y1;
    xdiff2 = xBound - x;
    ydiff2 = 0;

    double s, t;
    s = (-ydiff1 * (x1 - x) + xdiff1 * (y1 - y)) /
        (-xdiff2 * ydiff1 + xdiff1 * ydiff2);
    t = (xdiff2 * (y1 - y) - ydiff2 * (x1 - x)) /
        (-xdiff2 * ydiff1 + xdiff1 * ydiff2);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
      (*intersection)++;
  }
}

//////////////////////////////////////////////////
bool TriangleIsHole(GtsTriangle *_t, GtsFifo *_edgeList)
{
  GtsEdge *e1, *e2, *e3;
  GtsVertex *v1, *v2, *v3;

  gts_triangle_vertices_edges(_t, NULL, &v1, &v2, &v3, &e1, &e2, &e3);

  double xCenter = (v1->p.x + v2->p.x + v3->p.x) / 3.0;
  double yCenter = (v1->p.y + v2->p.y + v3->p.y) / 3.0;

  int intersections = 0;
  gpointer data[3];
  data[0] = &xCenter;
  data[1] = &yCenter;
  data[2] = &intersections;

  gts_fifo_foreach(_edgeList, (GtsFunc) Intersection, data);

  if (intersections % 2)
    return false;
  else
    return true;
}

//////////////////////////////////////////////////
GtsSurface *GTSMeshUtils::DelaunayTriangulation(
    const std::vector<std::vector<math::Vector2d> > &_path)
{
  GSList *l, *verticesList = NULL;
  double z = 0;

  GtsSurface *surface;
  GtsVertex *v1, *v2, *v3;
  GtsFifo *edgeList;
  edgeList = gts_fifo_new();
  verticesList = NULL;

  unsigned int e = 0;
  for (unsigned p = 0; p < _path.size(); ++p)
  {
    unsigned int startEdgeIndex = e;
    unsigned int numSides = _path[p].size();
    // List the vertices and edges
    for (unsigned int i = 0; i < numSides; ++i)
    {
      verticesList = g_slist_append(verticesList,
          gts_vertex_new(gts_vertex_class(),
            _path[p][i].x, _path[p][i].y, z));
      if (i != 0)
      {
        gts_fifo_push(edgeList,
            gts_edge_new(GTS_EDGE_CLASS(gts_constraint_class()),
            reinterpret_cast<GtsVertex *>
            (g_slist_nth_data(verticesList, e-1)),
            reinterpret_cast<GtsVertex *>
            (g_slist_nth_data(verticesList, e))));
      }
      e++;
    }

    gts_fifo_push(edgeList,
        gts_edge_new(GTS_EDGE_CLASS(gts_constraint_class()),
        reinterpret_cast<GtsVertex *>
        (g_slist_nth_data(verticesList, e-1)),
        reinterpret_cast<GtsVertex *>
        (g_slist_nth_data(verticesList, startEdgeIndex))));
  }

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
  gts_allow_floating_vertices = true;
  gts_object_destroy(GTS_OBJECT(v1));
  gts_object_destroy(GTS_OBJECT(v2));
  gts_object_destroy(GTS_OBJECT(v3));
  gts_allow_floating_vertices = false;

  // Remove edges on the boundary which are not constraints
  gts_delaunay_remove_hull(surface);

  // remove holes - only needed if there is more than one path
  if (_path.size() > 1)
  {
    gts_surface_foreach_face_remove(surface, (GtsFunc) TriangleIsHole,
        edgeList);
  }

  gts_fifo_destroy(edgeList);

  return surface;
}

//////////////////////////////////////////////////
bool GTSMeshUtils::DelaunayTriangulation(
    const std::vector<std::vector<math::Vector2d> > &_path, SubMesh *_subMesh)
{
  if (_path.empty())
  {
    gzerr << "Unable to create an extruded outline mesh with "
      << "no paths\n";
    return false;
  }

  if (!_subMesh)
    _subMesh = new SubMesh();

  GtsSurface *surface = GTSMeshUtils::DelaunayTriangulation(_path);

  // fill the submesh with data generated by GTS
  unsigned int n2 = 0;
  gpointer data[3];
  GHashTable *vIndex = g_hash_table_new(NULL, NULL);

  data[0] = _subMesh;
  data[1] = &n2;
  data[2] = vIndex;
  gts_surface_foreach_vertex(surface, (GtsFunc) FillVertex, data);
  n2 = 0;
  gts_surface_foreach_face(surface, (GtsFunc) FillFace, data);

  g_hash_table_destroy(vIndex);
  gts_object_destroy(GTS_OBJECT(surface));
  return true;
}
