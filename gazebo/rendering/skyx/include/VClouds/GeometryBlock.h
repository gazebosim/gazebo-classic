/*
--------------------------------------------------------------------------------
This source file is part of SkyX.
Visit http://www.paradise-studios.net/products/skyx/

Copyright (C) 2009-2012 Xavier Verguín González <xavyiy@gmail.com>

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
--------------------------------------------------------------------------------
*/

#ifndef _SkyX_VClouds_GeometryBlock_H_
#define _SkyX_VClouds_GeometryBlock_H_

#include "Prerequisites.h"

namespace SkyX { namespace VClouds {

  class VClouds;

  class DllExport GeometryBlock
  {
    public:
      /** Vertex struct
      */
      struct VERTEX
      {        // Position
        float x, y, z,
              // 3D Coords
              xc, yc, zc,
              // Noise coords
              u, v,
              // Opacity
              o;
      };

      /** Constructor
        @param vc VClouds pointer
        @param Height Field height (in woorld coordinates)
        @param Alpha Alpha angle
        @param Beta Beta angle
        @param Radius Total radius
        @param Phi Actimutal angle
        @param Na Number of slices in A zone
        @param Nb Number of slices in B zone
        @param Nc Number of slices in C zone
        @param A A radius
        @param B B radius
        @param C C radius
        */
      GeometryBlock(VClouds *vc,
          const float& Height, const Ogre::Radian& Alpha,
          const Ogre::Radian& Beta,
          const float& Radius, const Ogre::Radian& Phi, const int& Na,
          const int& Nb, const int& Nc, const int& A,
          const int& B, const int& C, const int& Position);

      /** Destructor
      */
      ~GeometryBlock();

      /** Create
      */
      void create();

      /** Remove
      */
      void remove();

      /** Update geometry
        @param c Camera
        @param displacement Current offset in world units per zone
        */
      void updateGeometry(Ogre::Camera* c, const Ogre::Vector3& displacement);

      /** Has been create() already called?
        @return true if created() have been already called, false if not
        */
      inline const bool& isCreated() const
      {
        return mCreated;
      }

      /** Get mesh
        @return Mesh
        */
      inline Ogre::MeshPtr getMesh()
      {
        return mMesh;
      }

      /** Get sub mesh
        @return Sub mesh
        */
      inline Ogre::SubMesh* getSubMesh()
      {
        return mSubMesh;
      }

      /** Get entity
        @return Entity
        */
      inline Ogre::Entity* getEntity()
      {
        return mEntity;
      }

      /** Get hardware vertex buffer reference
        @return Ogre::HardwareVertexBufferSharedPtr reference
        */
      inline Ogre::HardwareVertexBufferSharedPtr &getHardwareVertexBuffer()
      {
        return mVertexBuffer;
      }

      /** Get hardware index buffer reference
        @return Ogre::HardwareIndexBufferSharedPtr reference
        */
      inline Ogre::HardwareIndexBufferSharedPtr &getHardwareIndexBuffer()
      {
        return mIndexBuffer;
      }

      /** Set world offset
        @param WorldOffset World offset
        */
      inline void setWorldOffset(const Ogre::Vector2& WorldOffset)
      {
        mWorldOffset = WorldOffset;
      }

      /** Is the geometry block inside the camera frustum?
        @param c Camera
        @return true if yes, false if not
        */
      bool isInFrustum(Ogre::Camera *c) const;

    private:
      /** Build axis aligned box
        @param fd Falling distance (Positive values for falling geometry,
        negative for reverse falling geometry)
        */
      const Ogre::AxisAlignedBox _buildAABox(const float& fd) const;

      /** Calculate data size
      */
      void _calculateDataSize();

      /** Create geometry
      */
      void _createGeometry();

      /** Update geometry
      */
      void _updateGeometry();

      /** Update zone C slice
        @param n Number of slice
        */
      void _updateZoneCSlice(const int& n);

      /** Update zone B slice
        @param n Number of slice
        */
      void _updateZoneBSlice(const int& n);

      /** Update zone A slice
        @param n Number of slice
        */
      void _updateZoneASlice(const int& n);

      /** Set vertex data
        @param index Vertex index
        @param o Slice opacity
        @param p Position
        */
      void _setVertexData(const int& index, const Ogre::Vector3& p,
                          const float& o);

      /// VClouds pointer
      VClouds *mVClouds;

      /// Has been create() already called?
      bool mCreated;

      /// Ogre::Submesh pointer
      Ogre::SubMesh *mSubMesh;

      /// Ogre::Entity pointer
      Ogre::Entity *mEntity;

      /// Vertices pointer
      VERTEX *mVertices;

      /// Current number of triangles
      int mNumberOfTriangles;
      /// Vertex count
      int mVertexCount;

      /// Height
      float mHeight;
      /// Angles
      Ogre::Radian mAlpha, mBeta;

      /// Radius
      float mRadius;

      /// Acimutal angle
      Ogre::Radian mPhi;
      /// Number of slices per geometry zone
      int mNa, mNb, mNc;
      /// A, B and C radius
      float mA, mB, mC;

      /// Number of block(Position)
      int mPosition;

      /// Displacement
      Ogre::Vector3 mDisplacement;
      /// World coords offset
      Ogre::Vector2 mWorldOffset;

      /// Current rendering camera
      Ogre::Camera* mCamera;

      /// Last falling distance
      float mLastFallingDistance;

      /// Ogre::MeshPtr
      Ogre::MeshPtr mMesh;
      /// Vertex buffer
      Ogre::HardwareVertexBufferSharedPtr mVertexBuffer;
      /// Index buffer
      Ogre::HardwareIndexBufferSharedPtr  mIndexBuffer;

      /// Precomputed Cos/Sin vectors
      Ogre::Vector2 mV2Cos;
      Ogre::Vector2 mV2Sin;
      /// PI - Beta, PI - Alpha Sin
      float mBetaSin;
      float mAlphaSin;
  };
}}
#endif
