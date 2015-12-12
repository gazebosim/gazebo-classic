/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_BUILDING_IMPORT_IMAGE_VIEW_PRIVATE_HH_
#define _GAZEBO_BUILDING_IMPORT_IMAGE_VIEW_PRIVATE_HH_

#include <vector>

#include "gazebo/common/Events.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for ImportImageView
    class ImportImageViewPrivate
    {
      /// \brief Constructor
      public: ImportImageViewPrivate() : currentMouseItem(0) {};

      /// \brief Width of the pixmap in pixels.
      public: int pixmapWidthPx;

      /// \brief Height of the pixmap in pixels.
      public: int pixmapHeightPx;

      /// \brief Width of the image in pixels.
      public: int imageWidthPx;

      /// \brief Length of the measure on the scene in pixels.
      public: int measureScenePx;

      /// \brief Indicate whether or not a drawing operation is taking place.
      public: bool drawInProgress;

      /// \brief A list of gui editor events connected to this view.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Editor item currently attached to the mouse during a drawing.
      /// operation.
      public: QGraphicsItem *currentMouseItem;

      /// \brief Grid lines drawn on the background of the editor.
      public: GridLines *gridLines;

      /// \brief Currently selected image item.
      public: QGraphicsPixmapItem *imageItem;

      /// \brief Currently selected image pixmap.
      public: QPixmap *imagePixmap;

      /// \brief Text to be displayed when no image has been selected.
      public: QGraphicsTextItem *noImageText;

      /// \brief Text to be displayed when an invalid file is selected.
      public: QGraphicsTextItem *invalidImageText;

      /// \brief Text to be displayed when an invalid file is selected.
      public: MeasureItem *measureItem;

      /// \brief Parent widget, which is the dialog.
      public: ImportImageDialog *dialog;

      /// \brief Indicates if it is allowed to draw on the view or not.
      public: bool drawDistanceEnabled;
    };
  }
}

#endif
