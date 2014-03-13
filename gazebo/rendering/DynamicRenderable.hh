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
#ifndef _DYNAMICRENDERABLE_HH_
#define _DYNAMICRENDERABLE_HH_

#include <string>
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class DynamicRenderable DynamicRenderable.hh rendering/rendering.hh
    /// \brief Abstract base class providing mechanisms for dynamically
    /// growing hardware buffers.
    class GAZEBO_VISIBLE DynamicRenderable : public Ogre::SimpleRenderable
    {
      /// \brief Constructor
      public: DynamicRenderable();

      /// \brief Virtual destructor
      public: virtual ~DynamicRenderable();

      /// \brief Initializes the dynamic renderable.
      /// \remarks This function should only be called once. It initializes the
      /// render operation, and calls the abstract function
      /// CreateVertexDeclaration().
      /// \param[in] _opType The type of render operation to perform.
      /// \param[in] _useIndices Specifies whether to use indices to
      /// determine the vertices to use as input.
      public: void Init(RenderOpType _opType, bool _useIndices = false);

      /// \brief Set the render operation type
      /// \param[in] _opType The type of render operation to perform.
      public: void SetOperationType(RenderOpType _opType);

      /// \brief Get the render operation type
      /// \return The render operation type.
      public: RenderOpType GetOperationType() const;

      /// \brief Implementation of Ogre::SimpleRenderable
      /// \return The bounding radius
      public: virtual Ogre::Real getBoundingRadius() const;

      /// \brief Implementation of Ogre::SimpleRenderable
      /// \param[in] _cam Pointer to the Ogre camera that views the
      /// renderable.
      /// \return The squared depth in the Camera's view
      public: virtual Ogre::Real getSquaredViewDepth(
                  const Ogre::Camera *_cam) const;

      /// \brief Get type of movable
      /// \return This returns "gazebo::DynamicRenderable"
      public: std::string GetMovableType() const;

      /// \brief Creates the vertex declaration.  @remarks Override and set
      /// mRenderOp.vertexData->vertexDeclaration here.  mRenderOp.vertexData
      /// will be created for you before this method is called.
      protected: virtual void CreateVertexDeclaration() = 0;

       /// \brief Prepares the hardware buffers for the requested vertex and
       ///        index counts.
       /// \remarks
       ///    This function must be called before locking the buffers in
       ///    fillHardwareBuffers().  It guarantees that the hardware buffers
       ///    are large enough to hold at least the requested number of
       ///    vertices and indices (if using indices).  The buffers are
       ///    possibly reallocated to achieve this.
       /// \par The vertex and index count in the render operation are set to
       ///      the values of vertexCount and indexCount respectively.
       /// \param[in] _vertexCount The number of vertices the buffer must hold.
       /// \param[in] _indexCount The number of indices the buffer must hold.
       ///        This parameter is ignored if not using indices.
      protected: void PrepareHardwareBuffers(size_t _vertexCount,
                                             size_t _indexCount);

       /// \brief Fills the hardware vertex and index buffers with data.
       /// @remarks
       ///   This function must call prepareHardwareBuffers() before locking the
       ///   buffers to ensure the they are large enough for the data to be
       ///   written. Afterwards the vertex and index buffers (if using indices)
       ///   can be locked, and data can be written to them.
      protected: virtual void FillHardwareBuffers() = 0;

      /// \brief Maximum capacity of the currently allocated vertex buffer.
      protected: size_t vertexBufferCapacity;

      /// \brief Maximum capacity of the currently allocated index buffer.
      protected: size_t indexBufferCapacity;
    };
    /// \}
  }
}
#endif
