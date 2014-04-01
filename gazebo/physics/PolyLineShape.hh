#ifndef _POLYLINESHAPE_HH_
#define _POLYLINESHAPE_HH_

#include <vector>

#include "gazebo/physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class PolyLineShape PolyLineShape.hh physics/physcs.hh
    /// \brief PolyLine geometry primitive.
    class PolyLineShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit PolyLineShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~PolyLineShape();

      /// \brief Initialize the polyLine.
      public: virtual void Init();

      /// \brief Set the height of the polyLine.
      /// \param[in] _height Size of each side of the polyLine.
      public: virtual void SetHeight(const double &_height);

      /// \brief Set the scale of the polyLine.
      /// \param[in] _scale Scale of the polyLine.
      public: virtual void SetScale(const math::Vector3 &_scale);

      /// \brief Set the vertices of the polyline
      /// \param[in] _msg geometry msg containing the vertex information
      public: virtual void SetVertices(const msgs::Geometry &_msg);
     
      /// \brief Get the height of the polyLine.
      /// \return The height of each side of the polyLine.
      public: double GetHeight() const;

      /// \brief Fill in the values for a geomertry message.
      /// \param[out] _msg The geometry message to fill.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Process a geometry message.
      /// \param[in] _msg The message to set values from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);
    };
    /// \}
  }
}
#endif
