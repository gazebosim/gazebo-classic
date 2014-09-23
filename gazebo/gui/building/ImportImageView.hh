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

#ifndef _IMPORT_IMAGE_VIEW_HH_
#define _IMPORT_IMAGE_VIEW_HH_

#include <map>
#include <string>
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class EditorItem;
    class MeasureItem;
    class GridLines;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ImportImageView ImportImageView.hh
    /// \brief Control the import image view and manage contents in the scene.
    class GAZEBO_VISIBLE ImportImageView : public QGraphicsView
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent Widget.
      public: ImportImageView(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ImportImageView();

      /// \brief Set the currently selected image.
      /// \param[in] _filename Name of the image file.
      /// \param[in] _width
      /// \param[in] _height
      public: void SetImage(const std::string &_filename, int _width,
                            int _height);

      /// \brief Qt resize event received when the parent widget changes size.
      /// \param[in] _event Qt resize event
      private: void resizeEvent(QResizeEvent *_event);

      /// \brief Qt mouse move event.
      /// \param[in] _event Qt mouse event.
      private: void mouseMoveEvent(QMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      private: void mouseReleaseEvent(QMouseEvent *_event);

      /// \brief Draw a measure in the scene.
      /// \param[in] _pos Start position of the measure in pixel coordinates.
      private: void DrawMeasure(const QPoint &_pos);

      /// \brief Indicate whether or not a drawing operation is taking place.
      private: bool drawInProgress;

      /// \brief A list of gui editor events connected to this view.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Editor item currently attached to the mouse during a drawing
      /// operation.
      private: QGraphicsItem *currentMouseItem;

      /// \brief Grid lines drawn on the background of the editor.
      private: GridLines *gridLines;

      /// \brief Currently selected image
      private: QGraphicsPixmapItem *image;

      /// \brief Text to be dispayed when no image has been selected
      private: QGraphicsTextItem *noImageText;

      /// \brief Text to be dispayed when an invalid file is selected
      private: QGraphicsTextItem *invalidImageText;

      /// \brief Text to be dispayed when an invalid file is selected
      private: MeasureItem *measureItem;

      /// \brief Scale (zoom level) of the editor view.
      private: double viewScale;
    };
    /// \}
  }
}

#endif
