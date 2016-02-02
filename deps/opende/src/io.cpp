/*
Copyright (c) 2013 Claude Lacoursiere

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.

reference:
   hdf5-based BPMD database: https://grasp.robotics.cs.rpi.edu/bpmd/
*/


#include "gazebo/gazebo_config.h"
#ifdef HDF5_INSTRUMENT
#include <iostream>
#include <string>
#include <string.h>
#include <stdlib.h>
#include "io.h"
#include "ioh5.h"
#include "objects.h"
#include "joints/joint.h"
#include "joints/contact.h"
#include <ode/h5dump.h>

using namespace H5;
using namespace h5;

// This does not use STL because the need for iterators is almost nil.

// Utilities
// Put identical values in all elements of a vector
template <typename R, typename V>
void set_vector(R *x, const V& v, size_t n, size_t stride = 1)
{
  for (size_t i=0; i<n; ++i){
    x[ i * stride ] = v;
  }
}

/// this wrapper saves space and typing
void copy(dReal *dest, const dReal * source, size_t n, size_t offset)
{
  for (size_t i = 0; i<n; ++i) {
    dest[i + n*offset] = source[i];
  }
}

/// this wrapper saves space and typing
void copy2(dReal *dest, const dReal * s1, const dReal * s2, size_t block, size_t offset)
{
  for (size_t i = 0; i<block; ++i) {
    dest[i +   2 * block * offset] = s1[i];
    dest[i +   3 * block * offset] = s2[i];
  }
}

// ODE specific utilities.
// ODE's matrix format for 3x3 matrices has leading
// dimension 4 and we need this to be 3 instead.
dReal* copy_dMatrix3_33(dReal * dest, dReal * source)
{
  memcpy(dest ,    source,    3*sizeof(dReal));
  memcpy(dest+3,  source+4,   3*sizeof(dReal));
  memcpy(dest+6,  source+8,   3*sizeof(dReal));
  return dest;
}

// Must compute world inertia here since that is not cached in the body
// data structure, but computed as needed in the solvers.

dReal * get_world_inertia(dReal * dest, dxBody * b, size_t offset)
{
  dMatrix3  M0;
  dMatrix3  tmp;
  dMultiply2_333 (tmp, b->mass.I, b->posr.R);
  dMultiply0_333 (M0,   b->posr.R, tmp);
  // Now, dMatrix3 is a 12D array so we need to extract the first 3
  // elements of each row
  return copy_dMatrix3_33(dest + offset*9, (dReal *)M0);
}

/// maybe unnecessarily verbose since this
/// is an internal function.
void h5dump::allocate_bodies(){

  int n = nbodies;

  if ( nbodies >  0  ) {
    x = new dReal[3*n];   // positions
    set_vector(x, 0, 3*n);
    q = new dReal[4*n];   // quaternions
    set_vector(q, 0,4*n);   // quaternions

    v = new dReal[6*n];   // velocites, linear *and* angular
    set_vector(v, 0, 6*n);   // velocites, linear *and* angular

    f = new dReal[6*n];   // forces, translational and rotational
    set_vector(f, 0, 6*n);

    m = new dReal[n];     // mass
    set_vector(m, 0, n);

    M = new dReal[9*n];   // inertia tensor in world frame
    /// this will be repacked as a 3 x (3n) matrix during dump
    set_vector(M, 0, 9*n);

    id = new int[n];            // tags for rigid bodies to match with
                                // matrices
    state = new int[n];         // 1 if body is active, -1 otherwise.

    body_data = new void*[n];
    memset(body_data, 0, n*sizeof(void*));
  }
}

void h5dump::free_bodies(){

  if ( nbodies >  0 ) {
    if (x) delete [] x;
    if (q) delete [] q;
    if (v) delete [] v;
    if (f) delete [] f;
    if (m) delete [] m;
    if (M) delete [] M;
    if (body_data) delete [] body_data;
    nbodies = 0;
  }
}

void h5dump::allocate_constraints()
{
  // now allocate and initialize

  if ( nrows > 0 ) {
    rows      = new size_t[nrows];
    memset(rows, 0, nrows*sizeof(size_t));

    g      = new dReal[nrows];
    set_vector(g, 0, nrows);

    gt     = new dReal[nrows];
    set_vector(gt, 0, nrows);

    bounds  = new dReal[2*nrows];
    set_vector(bounds  , -INFINITY, nrows, 2);
    set_vector(bounds+1,  INFINITY, nrows, 2);

    constraint_pairs = new int[2*nrows];
    set_vector(constraint_pairs, -1, 2*nrows);

    G = new dReal[nrows*ldg];
    set_vector(G, 0, nrows*ldg);
  }
}

void h5dump::free_constraints()
{
  if ( nrows > 0 ) {
    if ( g ) delete [] g;
    if ( gt ) delete [] gt;
    if ( bounds ) delete [] bounds;
    if (constraint_pairs ) delete [] constraint_pairs;
    if ( G ) delete [] G;
    nrows = 0 ;
  }
}

void h5dump::allocate_contacts(){
  size_t   n = ncontacts;
  if ( n >  0 ) {
    points  = new dReal[n*3];
    set_vector(points, 0, n*3);

    normals = new dReal[n*3];
    set_vector(normals, 0, n*3);

    // change it to gaps
    gaps = new dReal[n];
    set_vector(gaps, 0, n);

    mu_s    = new dReal[2*n];
    set_vector(mu_s, 0, 2*n);

    mu_k    = new dReal[3*n];
    set_vector(mu_k, 0, 3*n);

    contact_pairs = new int[n*2];
    set_vector(contact_pairs, 0, n*2);
  }
}

void h5dump::free_contacts()
{
  if ( ncontacts >   0) {
    if ( points ) delete [] points;
    if ( normals ) delete [] normals;
    if ( gaps ) delete [] gaps;
    if ( mu_s ) delete [] mu_s;
    if ( mu_k ) delete [] mu_k;
    if ( contact_pairs ) delete [] contact_pairs;
    ncontacts = 0  ;
  }
}

/// concatenate all the rigid body data into a packed array
void h5dump::pack_bodies()
{
  int n = nbodies = world->nb;
  if ( n > 0 ) {
    allocate_bodies();
    int i = 0;
    for (dxBody* b = world->firstbody; b != 0 ; b = (dxBody *)b->next) {
      // positions stored as just CM coordinates
      copy(x, dBodyGetPosition(b), 3, i );

      // Rotations as quaternions
      copy(q, dBodyGetQuaternion(b), 4, i );

      // linear and angular velocities are concatenated
      memcpy(v+6*i, dBodyGetLinearVel(b), 3 * sizeof(dReal) );
      memcpy(v+6*i+3, dBodyGetAngularVel(b), 3 * sizeof(dReal) );

      // forces and torques are concatenated
      //copy2(f, b->facc, b->tacc, 3, i);
      memcpy(f+6*i, b->facc, 3 * sizeof(dReal) );
      memcpy(f+6*i+3, b->tacc, 3 * sizeof(dReal) );

      // the scalar mass
      m[i] =  b->mass.mass;

      // compute and store the inertia tensor
      get_world_inertia(M, b, i);

      state[i]  = ( ( b->flags & dxBodyDisabled ) == 0 ) ? 1 : -1 ;
      id[i] =  tags[(void*)b] = i+1;
      ++i;
    }
  }
  tags[NULL] = -1;
}

void h5dump::pack_constraints()
{
  nconstraints = 0;
  // first cound how many active rows we have for regular constraints,
  // i.e., not contacts
  nrows  =0;
  for (dxJoint* c = world->firstjoint; c != 0 ; c = (dxJoint  *)c->next) {
    if ( c->type() != dJointTypeContact) {
      ++nconstraints;
      dxJoint::Info1 info1;
      c->getInfo1(&info1);
      nrows += (size_t)info1.m;
    }
  }

  if ( nconstraints > 0 ) {
    allocate_constraints();
    // tmp vector for ODE
    int * junk = new int[nrows];

    // Jacobian matrix, i.e., it is a row major nrows x 12 array
    int i = 0;
    int row = 0;
    for (dxJoint* c = world->firstjoint; c != 0 ; c = (dxJoint*)c->next) {
      if ( dJointIsEnabled(c) && c->type() != dJointTypeContact) {

        // find out which bodies we have
        // for reasons unknown to me, this labeling is inverted in ODE
        for (int j = 1; j > -1 ; --j )
          constraint_pairs[i*2  -j + 1 ] = tags[c->node[j].body];

        // Get number of rows
        dxJoint::Info1 info1;
        c->getInfo1(&info1);
        // store the number of rows
        rows[i] = (size_t )info1.m;

        dxJoint::Info2 info;
        info.rowskip = ldg;       // The leading dimension of G : 12;
        info.fps     = 60;           // arbitrary value: we don't store that
        info.erp     = 1;            // arbitrary value: we don't store that

        dReal * J0  = G + ldg * row;
        dReal  cfm[3];
        info.J1l = J0;
        info.J1a = J0 + 3;
        info.J2l = J0 + 6;
        info.J2a = J0 + 9;
        info.c = g + row;
        info.cfm = (dReal *)cfm;
        info.lo = bounds + 2*row;
        info.hi = bounds + 2*row + 1;
        info.findex = junk + row;

        c->getInfo2(&info);
        row += rows[i];
        ++i;
      }
    }
    delete [] junk;
  }
}

void h5dump::pack_contacts()
{
  ncontacts  =0;
  for (dxJoint* c = world->firstjoint; c != 0 ; c = (dxJoint *)c->next) {
    ncontacts += ( c->type() == dJointTypeContact && dJointIsEnabled(c)) ;
  }
  if ( ncontacts ) {
    allocate_contacts();

    // extract data
    int k = 0;                        // running index
    for (dxJoint* c = world->firstjoint; c != 0 ; c = (dxJoint *)c->next) {
      if ( c->type() == dJointTypeContact && dJointIsEnabled(c)) {
        dContactGeom geo  = ((dxJointContact *)c)->contact.geom;
        dSurfaceParameters surf  = ((dxJointContact *)c)->contact.surface;

        // gaps is the -1*depth
        gaps[k] =    -1.0*geo.depth;
        copy(normals, geo.normal, 3, k);
        copy(points,  geo.pos,    3, k);
        mu_s[2 * k    ] =     surf.mu ;
        mu_s[2 * k + 1] =     surf.mu ;
        if ( surf.mode&dContactMu2 ) {
          mu_k[2*k] = surf.mu2;
          mu_k[2*k + 1] = surf.mu2;
        }
        else {
          mu_k[2 * k]     = surf.mu;
          mu_k[2 * k + 1] = surf.mu;
        }
        if ( surf.mode&dContactMu3 ) {
          mu_k[3*k] = surf.mu3;
          mu_k[3*k + 1] = surf.mu3;
        }
        else {
          mu_k[3 * k]     = surf.mu;
          mu_k[3 * k + 1] = surf.mu;
        }

        // find out which bodies we have
        for (int i = 1; i > -1 ; --i )
          contact_pairs[k*2  -i + 1 ] = tags[c->node[i].body];

        ++k;
      }
    }
  }
}

void h5dump::dump(bool first)
{
  pack_bodies();
  pack_constraints();
  pack_contacts();

  H5File * F = append_or_create( file );

  Group  gp = append_problem(F, "mbs_frame");
  if (first )
    //dump_string(gp, "toolkit", "This example was generated by the ODE toolkit");

  dump_scalar(gp, "timestep",    step_size);

  // bodies
  if ( nbodies >  0  ) {
    Group gb = gp.createGroup("bodies");
    // store positions and quaternions as 2D arrays with 3 and 4 columns,
    // respectively which removes possibility of confusion with the format
    // of the generalized forces and velocities
    dump_array(gb, "positions", x, nbodies, 3);
    dump_array(gb, "quaternions", q, nbodies, 4);
    // these are plain vectors which makes the manipulation of the whole
    // system easier.
    dump_array(gb, "velocities", v, 6*nbodies);
    dump_array(gb, "forces", f, 6*nbodies);
    dump_array(gb, "masses", m, nbodies);
    // world_inertia_tensors
    dump_array(gb, "world_inertia_tensors", M, 3*nbodies, 3);
    dump_array(gb, "ids", id, nbodies);
    // states is kept here to make it more flexible to recover the simulation
    dump_array(gb, "states", state, nbodies);
    gb.close();
    free_bodies();
  }

  if ( nconstraints >  0 ) {
    Group gc = gp.createGroup("constraints");
    dump_array(gc, "rows",              rows, nconstraints);
    dump_array(gc, "violations",         g,    nrows);
    dump_array(gc, "time_derivative",   gt,    nrows);
    dump_array(gc, "bounds",          bounds,    nrows, 2);
    dump_array(gc, "pairs",  constraint_pairs,   nconstraints, 2);
    dump_array(gc, "Jacobian",          G,   nrows, ldg);
    gc.close();
    free_constraints();
  }

  if ( ncontacts >  0 ) {
    Group gct = gp.createGroup("contacts");
    dump_array(gct,  "points",          points, ncontacts, 3);
    dump_array(gct,  "normals",         normals, ncontacts, 3);
    dump_array(gct,  "gaps",            gaps, ncontacts);
    dump_array(gct,  "mu_s",            mu_s, ncontacts, 2);
    dump_array(gct,  "mu_k",            mu_k, ncontacts, 3);
    dump_array(gct, "pairs",   contact_pairs,    ncontacts, 2);
    gct.close();
    free_contacts();
  }

  gp.close();
  F->close();
  delete F;
}

extern "C" {
  void h5dump_world(const char * file, dxWorld * w, dReal stepsize)
  {
    static size_t frame = 0;

    if ( true  || getenv("DUMP_HDF5_ODE")) {
      ++frame;
      size_t freq = floor(1/stepsize);
      // dump 5 times a second
      h5dump h(file, w, stepsize, frame%(freq/5) );
    }
  }

  void h5_write_errors(const char *file , dReal * errors, unsigned int m){
    H5File * F = append_or_create( H5std_string(file) );
    H5::Group  root = F->openGroup("/");
    H5::Group  g = root.openGroup(root.getObjnameByIdx(root.getNumObjs()-1));
    if ( ! g.getNumObjs() || g.getObjnameByIdx(0) != H5std_string("solver_data") ) {
      g = g.createGroup("solver_data");
      if ( ! g.getNumObjs() || g.getObjnameByIdx(0) != H5std_string("ODE_GS") ) {
        g = g. createGroup("ODE_GS");
        h5::dump_array( g, std::string("gs_errors"), errors, m );
      }
    }
    F->close();
    delete F;
    return;
  }
}

#endif // HDF5_INSTRUMENT
