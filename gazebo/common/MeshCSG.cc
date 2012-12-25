/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <string>
#include "gts.h"

#include "gazebo/common/Common.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/MeshCSG.hh"
#include "gazebo/common/MeshManager.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
MeshCSG::MeshCSG()
{
}

//////////////////////////////////////////////////
MeshCSG::~MeshCSG()
{
}

//////////////////////////////////////////////////
static void MergeVertices(GPtrArray * _vertices, double _epsilon)
{
  GPtrArray *array;
  GNode *kdtree;
  GtsVertex **verticesData = reinterpret_cast<GtsVertex **>(_vertices->pdata);
  array = g_ptr_array_new();
  for (unsigned int i = 0; i < _vertices->len; ++i)
    g_ptr_array_add (array, verticesData[i]);
  kdtree = gts_kdtree_new(array, NULL);
  g_ptr_array_free (array, true);

  for (unsigned int i = 0; i < _vertices->len; i++)
  {
    GtsVertex *v = reinterpret_cast<GtsVertex *>(verticesData[i]);

    if (!GTS_OBJECT (v)->reserved)
    {
      // Do something only if v is active
      GtsBBox *bbox;
      GSList *selected, *j;

      // build bounding box
      bbox = gts_bbox_new (gts_bbox_class(),
			   v,
			   GTS_POINT(v)->x - _epsilon,
			   GTS_POINT(v)->y - _epsilon,
			   GTS_POINT(v)->z - _epsilon,
			   GTS_POINT(v)->x + _epsilon,
			   GTS_POINT(v)->y + _epsilon,
			   GTS_POINT(v)->z + _epsilon);

      // select vertices which are inside bbox using kdtree
      j = selected = gts_kdtree_range (kdtree, bbox, NULL);
      while (j)
      {
	      GtsVertex *sv = reinterpret_cast<GtsVertex *>(j->data);
        // mark sv as inactive
	      if (sv != v && !GTS_OBJECT(sv)->reserved)
	        GTS_OBJECT(sv)->reserved = v;
	      j = j->next;
      }
      g_slist_free(selected);
      gts_object_destroy(GTS_OBJECT(bbox));
    }
  }

  gts_kdtree_destroy(kdtree);

  // destroy inactive vertices

  // we want to control vertex destruction
  gts_allow_floating_vertices = true;

  for (unsigned int i = 0; i < _vertices->len; ++i)
  {
    GtsVertex * v = reinterpret_cast<GtsVertex *>(verticesData[i]);
    // v is inactive
    if (GTS_OBJECT(v)->reserved)
    {
      verticesData[i] =
          reinterpret_cast<GtsVertex *>(GTS_OBJECT(v)->reserved);
      gts_object_destroy(GTS_OBJECT(v));
    }
  }
  gts_allow_floating_vertices = false;
}

//////////////////////////////////////////////////
static void FillVertex(GtsPoint *_p, gpointer *_data)
{
  /*(*GTS_OBJECT (p)->klass->write) (GTS_OBJECT (p), (FILE *) data[0]);
  if (!GTS_POINT_CLASS (GTS_OBJECT (p)->klass)->binary)
    fputc ('\n', (FILE *) data[0]);*/

  SubMesh *subMesh = reinterpret_cast<SubMesh *>(_data[0]);
  GHashTable* vIndex = reinterpret_cast<GHashTable *>(_data[2]);
  subMesh->AddVertex(GTS_POINT(_p)->x, GTS_POINT(_p)->y, GTS_POINT (_p)->z);
  g_hash_table_insert(vIndex, _p, GUINT_TO_POINTER (++(*((guint *)_data[1]))));
}

//////////////////////////////////////////////////
static void FillEdge(GtsSegment *_s, gpointer *_data)
{
/*  fprintf ((FILE *) data[0], "%u %u",
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[2], s->v1)),
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[2], s->v2)));
  if (GTS_OBJECT (s)->klass->write)
    (*GTS_OBJECT (s)->klass->write) (GTS_OBJECT (s), (FILE *) data[0]);
  fputc ('\n', (FILE *) data[0]);*/
  GHashTable* eIndex = reinterpret_cast<GHashTable *>(_data[3]);
  g_hash_table_insert (eIndex, _s, GUINT_TO_POINTER (++(*((guint *)_data[1]))));
}

//////////////////////////////////////////////////
static void FillFace(GtsTriangle *_t, gpointer *_data)
{
/*  fprintf (data[0], "%u %u %u",
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[3], t->e1)),
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[3], t->e2)),
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[3], t->e3)));
  if (GTS_OBJECT (t)->klass->write)
    (*GTS_OBJECT (t)->klass->write) (GTS_OBJECT (t), data[0]);
  fputc ('\n', data[0]);*/
  SubMesh *subMesh = reinterpret_cast<SubMesh *>(_data[0]);
  GHashTable* eIndex = reinterpret_cast<GHashTable *>(_data[3]);
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup (eIndex, _t->e1)));
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup (eIndex, _t->e2)));
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup (eIndex, _t->e3)));
}

//////////////////////////////////////////////////
Mesh *MeshCSG::CreateBoolean(const Mesh *_m1,
    const Mesh *_m2, int _operation)
{
  GtsSurface *s1, *s2, *s3;
  GtsSurfaceInter *si;
  GNode *tree1, *tree2;

  bool closed = true;
  bool is_open1 = false;
  bool is_open2 = false;

  s1 = gts_surface_new(gts_surface_class(), gts_face_class(), gts_edge_class(),
      gts_vertex_class());
  s2 = gts_surface_new(gts_surface_class(), gts_face_class(), gts_edge_class(),
      gts_vertex_class());
  s3 = gts_surface_new(gts_surface_class(), gts_face_class(), gts_edge_class(),
      gts_vertex_class());

  this->ConvertMeshToGTS(_m1, s1);
  this->ConvertMeshToGTS(_m2, s2);

  // build bounding box tree for first surface
  tree1 = gts_bb_tree_surface (s1);
  is_open1 = gts_surface_volume (s1) < 0. ? true : false;

  // build bounding box tree for second surface
  tree2 = gts_bb_tree_surface (s2);
  is_open2 = gts_surface_volume (s2) < 0. ? true : false;

  si = gts_surface_inter_new (gts_surface_inter_class (), s1, s2, tree1, tree2,
      is_open1, is_open2);
  assert(gts_surface_inter_check (si, &closed));
  if (!closed)
  {
    gzerr << "the intersection of " << _m1->GetName() << " and "
        << _m2->GetName() << " is not a closed curve\n";
    return NULL;
  }

  /*FILE *output1 = fopen("output3.gts", "w");
  gts_surface_write(s1, output1);
  fclose(output1);

  FILE *output2 = fopen("output4.gts", "w");
  gts_surface_write(s2, output2);
  fclose(output2);*/

  if (_operation == MeshCSG::UNION)
  {
    gts_surface_inter_boolean (si, s3, GTS_1_OUT_2);
    gts_surface_inter_boolean (si, s3, GTS_2_OUT_1);
  }
  else if (_operation == MeshCSG::INTERSECTION)
  {
    gts_surface_inter_boolean (si, s3, GTS_1_IN_2);
    gts_surface_inter_boolean (si, s3, GTS_2_IN_1);
  }
  else if (_operation == MeshCSG::DIFFERENCE)
  {
    gts_surface_inter_boolean (si, s3, GTS_1_OUT_2);
    gts_surface_inter_boolean (si, s3, GTS_2_IN_1);
    gts_surface_foreach_face (si->s2, (GtsFunc) gts_triangle_revert, NULL);
    gts_surface_foreach_face (s2, (GtsFunc) gts_triangle_revert, NULL);
  }

  FILE *output = fopen("output.gts", "w");
  gts_surface_write(s3, output);
  fclose(output);

  // create the boolean mesh
  Mesh *mesh = new Mesh();
  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // fill the submesh with data generated by GTS
  unsigned int n;
  gpointer data[4];
  GHashTable *vIndex = g_hash_table_new(NULL, NULL);
  GHashTable *eIndex = g_hash_table_new(NULL, NULL);

  data[0] = subMesh;
  data[1] = &n;
  data[2] = vIndex;
  data[3] = eIndex;

  n = 0;
  gts_surface_foreach_vertex(s3, (GtsFunc)FillVertex, data);
  n = 0;
  gts_surface_foreach_edge(s3, (GtsFunc)FillEdge, data);
  gts_surface_foreach_face(s3, (GtsFunc)FillFace, data);
  g_hash_table_destroy(vIndex);
  g_hash_table_destroy(eIndex);

  mesh->RecalculateNormals();

  // destroy surfaces
  gts_object_destroy(GTS_OBJECT(s1));
  gts_object_destroy(GTS_OBJECT(s2));
  gts_object_destroy(GTS_OBJECT(s3));
  gts_object_destroy(GTS_OBJECT(si));

  // destroy bounding box trees (including bounding boxes)
  gts_bb_tree_destroy(tree1, true);
  gts_bb_tree_destroy(tree2, true);
  return mesh;
}

//////////////////////////////////////////////////
void MeshCSG::ConvertMeshToGTS(const Mesh *_mesh, GtsSurface *_surface)
{
  if (!_surface)
  {
    gzerr << "Surface is NULL ";
//    _surface = gts_surface_new(gts_surface_class(), gts_face_class(),
//        gts_edge_class(), gts_vertex_class());
    return;
  }

  GPtrArray *vertices = g_ptr_array_new();

  for (unsigned int i = 0; i < _mesh->GetSubMeshCount(); ++i)
  {
    const SubMesh *subMesh = _mesh->GetSubMesh(i);
    unsigned int indexCount = subMesh->GetIndexCount();
    if (subMesh->GetVertexCount() <= 2)
      continue;

    for (unsigned int j = 0; j < subMesh->GetVertexCount(); ++j)
    {
      math::Vector3 vertex = subMesh->GetVertex(j);
      g_ptr_array_add (vertices, gts_vertex_new(gts_vertex_class(), vertex.x,
          vertex.y, vertex.z));
    }

    // merge duplicate vertices, otherwise gts produces undesirable results
    MergeVertices(vertices, 0.01);

    GtsVertex **verticesData =
        reinterpret_cast<GtsVertex **>(vertices->pdata);
    for (unsigned int j = 0; j < indexCount/3; ++j)
    {
      GtsEdge *e1 = GTS_EDGE(gts_vertices_are_connected(
          verticesData[subMesh->GetIndex(3*j)],
          verticesData[subMesh->GetIndex(3*j+1)]));
      GtsEdge *e2 = GTS_EDGE(gts_vertices_are_connected(
          verticesData[subMesh->GetIndex(3*j+1)],
          verticesData[subMesh->GetIndex(3*j+2)]));
      GtsEdge *e3 = GTS_EDGE(gts_vertices_are_connected(
          verticesData[subMesh->GetIndex(3*j+2)],
          verticesData[subMesh->GetIndex(3*j)]));
      if (e1 == NULL && verticesData[subMesh->GetIndex(3*j)]
          != verticesData[subMesh->GetIndex(3*j+1)])
      {
        e1 = gts_edge_new(_surface->edge_class,
            verticesData[subMesh->GetIndex(3*j)],
            verticesData[subMesh->GetIndex(3*j+1)]);
      }
      if (e2 == NULL && verticesData[subMesh->GetIndex(3*j+1)]
          != verticesData[subMesh->GetIndex(3*j+2)])
      {
        e2 = gts_edge_new(_surface->edge_class,
            verticesData[subMesh->GetIndex(3*j+1)],
            verticesData[subMesh->GetIndex(3*j+2)]);
      }
      if (e3 == NULL && verticesData[subMesh->GetIndex(3*j+2)]
          != verticesData[subMesh->GetIndex(3*j)])
      {
        e3 = gts_edge_new(_surface->edge_class,
            verticesData[subMesh->GetIndex(3*j+2)],
            verticesData[subMesh->GetIndex(3*j)]);
      }
      if (e1 != NULL && e2 != NULL && e3 != NULL)
      {
        gts_surface_add_face(_surface, gts_face_new(_surface->face_class, e1,
            e2, e3));
      }
      else
      {
        gzwarn << _mesh->GetName() << ": Ignoring degenerate facet!";
      }
    }
  }
}
