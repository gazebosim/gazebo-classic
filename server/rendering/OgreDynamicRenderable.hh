#ifndef OGREDYNAMICRENDERABLE_HH
#define OGREDYNAMICRENDERABLE_HH

#include <OgreSimpleRenderable.h>

namespace gazebo
{

/// \addtogroup gazebo_rendering
/// \{

/// \brief Abstract base class providing mechanisms for dynamically growing hardware buffers.
class OgreDynamicRenderable : public Ogre::SimpleRenderable
{
  public: enum OperationType 
  {
    /// A list of points, 1 vertex per point
    OT_POINT_LIST = 0,//Ogre::RenderOperation::OT_POINT_LIST,

    /// A list of lines, 2 vertices per line
    OT_LINE_LIST = 1,//Ogre::RenderOperation::OT_LINE_LIST,

    /// A strip of connected lines, 1 vertex per line plus 1 start vertex
    OT_LINE_STRIP = 2,//Ogre::RenderOperation::OT_LINE_STRIP,

    /// A list of triangles, 3 vertices per triangle
    OT_TRIANGLE_LIST = 3,//Ogre::RenderOperation::OT_TRIANGLE_LIST,

    /// A strip of triangles, 3 vertices for the first triangle, and 1 per triangle after that 
    OT_TRIANGLE_STRIP = 4,//Ogre::RenderOperation::OT_TRIANGLE_STRIP,

    /// A fan of triangles, 3 vertices for the first triangle, and 1 per triangle after that
    OT_TRIANGLE_FAN = 5,//Ogre::RenderOperation::OT_TRIANGLE_FAN 
  };

  /// \brief Constructor
  public: OgreDynamicRenderable();

  /// \brief Virtual destructor
  public: virtual ~OgreDynamicRenderable();

  /// \brief Initializes the dynamic renderable.
  /// \remarks This function should only be called once. It initializes the
  /// render operation, and calls the abstract function
  /// CreateVertexDeclaration(). 
  /// \param operationType The type of render operation to perform.
  /// \param useIndices Specifies whether to use indices to determine the
  /// vertices to use as input.
  public: void Init(OperationType operationType, bool useIndices);

  /// \brief Set the render operation type
  public: void SetOperationType(OperationType opType);

  /// \brief Get the render operation type
  public: OgreDynamicRenderable::OperationType GetOperationType() const;

  /// \brief Implementation of Ogre::SimpleRenderable
  public: virtual Ogre::Real getBoundingRadius(void) const;

  /// \brief Implementation of Ogre::SimpleRenderable
  public: virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const;

  /// \brief Creates the vertex declaration.  @remarks Override and set
  /// mRenderOp.vertexData->vertexDeclaration here.  mRenderOp.vertexData
  /// will be created for you before this method is called.
  protected: virtual void CreateVertexDeclaration() = 0;

   /// \brief Prepares the hardware buffers for the requested vertex and 
   ///        index counts.
   /// \remarks 
   ///    This function must be called before locking the buffers in
   ///    fillHardwareBuffers().  It guarantees that the hardware buffers are
   ///    large enough to hold at least the requested number of vertices and
   ///    indices (if using indices).  The buffers are possibly reallocated to
   ///    achieve this.  
   /// \par The vertex and index count in the render operation are set to the
   ///      values of vertexCount and indexCount respectively.
   /// \param vertexCount The number of vertices the buffer must hold.
   /// \param indexCount The number of indices the buffer must hold.  
   ///        This parameter is ignored if not using indices.
  protected: void PrepareHardwareBuffers(size_t vertexCount, size_t indexCount);

   /// \brief Fills the hardware vertex and index buffers with data.
   /// @remarks 
   ///   This function must call prepareHardwareBuffers() before locking the 
   ///   buffers to ensure the they are large enough for the data to be 
   ///   written.  Afterwards the vertex and index buffers (if using indices) 
   ///   can be locked, and data can be written to them. 
  protected: virtual void FillHardwareBuffers() = 0;

  /// \brief Maximum capacity of the currently allocated vertex buffer.
  protected: size_t vertexBufferCapacity;

  /// \brief Maximum capacity of the currently allocated index buffer.
  protected: size_t indexBufferCapacity;

};

/// \}
}
#endif
