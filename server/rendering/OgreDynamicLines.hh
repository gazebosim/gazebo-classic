#ifndef OGREDYNAMICLINES_HH
#define OGREDYNAMICLINES_HH

#include "Vector3.hh"
#include "OgreDynamicRenderable.hh"

#include <vector>

namespace gazebo
{

class OgreDynamicLines : public OgreDynamicRenderable
{
  /*typedef Ogre::Vector3 Vector3;
  typedef Ogre::Quaternion Quaternion;
  typedef Ogre::Camera Camera;
  typedef Ogre::Real Real;
  typedef Ogre::RenderOperation::OperationType OperationType;
  */

  /// Constructor
  public: OgreDynamicLines(Ogre::RenderOperation::OperationType opType=Ogre::RenderOperation::OT_LINE_STRIP);

  /// Destructor
  public: virtual ~OgreDynamicLines();

  /// Add a point to the point list
  /// \param pt Vector3 point
  public: void AddPoint(const Vector3 &pt);

  /// Change the location of an existing point in the point list
  /// \param index Index of the point to set
  /// \param value Vector3 value to set the point to
  public: void SetPoint(unsigned int index, const Vector3 &value);

  /// Return the location of an existing point in the point list
  /// \param index Number of the point to return
  /// \return Vector3 value of the point
  public: const Vector3& GetPoint(unsigned int index) const;

  /// Return the total number of points in the point list
  /// \return Number of points
  public: unsigned int GetNumPoints() const;

  /// Remove all points from the point list
  public: void Clear();

  /// Call this to update the hardware buffer after making changes.  
  public: void Update();

  /// Set the type of operation to draw with.
  /// @param opType Can be one of 
  ///    - RenderOperation::OT_LINE_STRIP
  ///    - RenderOperation::OT_LINE_LIST
  ///    - RenderOperation::OT_POINT_LIST
  ///    - RenderOperation::OT_TRIANGLE_LIST
  ///    - RenderOperation::OT_TRIANGLE_STRIP
  ///   - RenderOperation::OT_TRIANGLE_FAN
  ///   The default is OT_LINE_STRIP.
  public: void SetOperationType(Ogre::RenderOperation::OperationType opType);

  /// Get the operation type
  /// \return The operation type
  public: Ogre::RenderOperation::OperationType GetOperationType() const;

  /// Implementation DynamicRenderable, creates a simple vertex-only decl
  protected: virtual void  CreateVertexDeclaration();

  /// Implementation DynamicRenderable, pushes point list out to hardware memory
  protected: virtual void FillHardwareBuffers();

  private: std::vector<Vector3> points;
  private: bool dirty;
};

}
#endif
