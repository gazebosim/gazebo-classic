/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_EDITORVIEW_HH_
#define _GAZEBO_GUI_EDITORVIEW_HH_

#include <memory>
#include <string>

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class Color;
  }

  namespace gui
  {
    class EditorItem;
    class GrabberHandle;
    class FloorItem;

    // Forward declare private data.
    class EditorViewPrivate;

    /// \class Level EditorView.hh
    /// \brief A convenient structure for storing level information
    class GZ_GUI_VISIBLE Level
    {
      /// \brief Constructor
      public: Level() : level(0), name("level"), baseHeight(0),
              // 2.4384m == 8ft, standard room height in US
              height(2.4384),
              backgroundPixmap(NULL),
              floorItem(NULL) {}

      /// \brief Level number
      public: int level;

      /// \brief Level name
      public: std::string name;

      /// \brief Level height from ground
      public: double baseHeight;

      /// \brief Level height
      public: double height;

      /// \brief Background pixmap for a level
      public: QGraphicsPixmapItem *backgroundPixmap;

      /// \brief Level's floor item
      public: FloorItem *floorItem;
    };

    /// \addtogroup gazebo_gui
    /// \{

    /// \class EditorView EditorView.hh
    /// \brief Control the editor view and manage contents in the editor scene.
    class GZ_GUI_VISIBLE EditorView : public QGraphicsView
    {
      Q_OBJECT

      /// \enum DrawModes
      /// \brief Unique identifiers for all drawing modes within the editor.
      public: enum DrawModes {
                  /// \brief None mode
                  NONE,
                  /// \brief Wall mode
                  WALL,
                  /// \brief Window mode
                  WINDOW,
                  /// \brief Door mode
                  DOOR,
                  /// \brief Stairs mode
                  STAIRS,
                  /// \brief Color mode
                  COLOR,
                  /// \brief Texture mode
                  TEXTURE
                };

      /// \brief Constructor
      /// \param[in] _parent Parent Widget.
      public: EditorView(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~EditorView();

      /// \brief Create a 3D visual from a 2D editor item.
      public: void Create3DVisual(EditorItem *_item);

      /// \brief Delete an editor item.
      /// \param[in] _item Item to be deleted.
      public: void DeleteItem(EditorItem *_item);

      /// \brief Set the graphics view background image.
      /// \param[in] _filename Name of the image file.
      /// \param[in] _scale Image scale, in meters/pixel.
      public: void SetBackgroundImage(const std::string &_filename,
                  const double _scale);

      /// \brief Qt resize event received when the parent widget changes size.
      /// \param[in] _event Qt resize event
      private: void resizeEvent(QResizeEvent *_event);

      /// \brief Qt event received when the editor view is being scrolled.
      /// \param[in] _dx Change in X position of the scrollbar.
      /// \param[in] _dx Change in Y position of the scrollbar.
      private: void scrollContentsBy(int _dx, int _dy);

      /// \brief Qt context menu received on a mouse right click.
      /// \param[in] _event Qt context menu event.
      private: void contextMenuEvent(QContextMenuEvent *_event);

      /// \brief Qt wheel event received when the mouse wheel is being scrolled.
      /// \param[in] _event Qt wheel event.
      private: void wheelEvent(QWheelEvent *_event);

      /// \brief Qt mouse move event.
      /// \param[in] _event Qt mouse event.
      private: void mouseMoveEvent(QMouseEvent *_event);

      /// \brief Qt mouse press event.
      /// \param[in] _event Qt mouse event.
      private: void mousePressEvent(QMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      private: void mouseReleaseEvent(QMouseEvent *_event);

      /// \brief Qt mouse double click event.
      /// \param[in] _event Qt mouse event.
      private: void mouseDoubleClickEvent(QMouseEvent *_event);

      /// \brief Qt leave event.
      /// \param[in] _event Qt mouse event.
      private: void leaveEvent(QEvent *_event);

      /// \brief Qt key press event.
      /// \param[in] _event Qt key event.
      private: void keyPressEvent(QKeyEvent *_event);

      /// \brief Draw a wall in the scene.
      /// \param[in] _pos Start position of the wall in pixel coordinates.
      private: void DrawWall(const ignition::math::Vector2i &_pos);

      /// \brief Draw a window in the scene.
      /// \param[in] _pos Scene position in pixel coordinates to draw the
      /// window.
      private: void DrawWindow(const ignition::math::Vector2i &_pos);

      /// \brief Draw a door in the scene
      /// \param[in] _pos Scene position in pixel coordinates to draw the
      /// door.
      private: void DrawDoor(const ignition::math::Vector2i &_pos);

      /// \brief Draw a staircase in the scene
      /// \param[in] _pos Scene position in pixel coordinates to draw the
      /// staircase.
      private: void DrawStairs(const ignition::math::Vector2i &_pos);

      /// \brief Callback triggered when the user chooses to draw an editor item
      /// in the scene
      /// \param[in] _type Type of editor item to be created.
      private: void OnCreateEditorItem(const std::string &_type);

      /// \brief Callback triggered when the user chooses a color on the
      /// palette.
      /// \param[in] _color Selected color.
      private: void OnColorSelected(const common::Color &_color);

      /// \brief Callback triggered when the user chooses a texture on the
      /// palette.
      /// \param[in] _texture Selected texture.
      private: void OnTextureSelected(const std::string &_texture);

      /// \brief Callback received when the model has been completed and
      /// uploaded onto the server.
      private: void OnFinishModel();

      /// \brief Callback received when the model has been discarded.
      private: void OnDiscardModel();

      /// \brief Qt callback when a level is to be added to the building model.
      private slots: void OnAddLevel();

      /// \brief Qt callback when a level is to be deleted from the building
      /// model.
      private slots: void OnDeleteLevel();

      /// \brief Qt callback when changes to a building level are to be applied.
      private slots: void OnLevelApply();

      /// \brief Qt Callback for opening the building level inspector.
      private slots: void OnOpenLevelInspector();

      /// \brief Callback received when a level on a building model is to
      /// be changed.
      /// \param[in] _level The level that is currently being edited.
      private: void OnChangeLevel(const int _level);

      /// \brief Delete a level from the building model
      /// \param[in] _level Level number to delete.
      private: void DeleteLevel(const int _level);

      /// \brief Cancel the current drawing operation.
      private: void CancelDrawMode();

      /// \brief Toggle visibility of background floorplan.
      private: void OnShowFloorplan();

      /// \brief Toggle visibility of editor items.
      private: void OnShowElements();

      /// \brief Show current level items if not currently hiding.
      private: void ShowCurrentLevelItems();

      /// \brief Link grabbers so they move together.
      /// \param[in] _grabber1 First grabber to be liked.
      /// \param[in] _grabber2 Second grabber to be unliked.
      private: void LinkGrabbers(GrabberHandle *_grabber1,
          GrabberHandle *_grabber2);

      /// \brief Unlink grabbers so they don't move together anymore. If only
      /// one grabber is input, that grabber is unliked from all its current
      /// links.
      /// \param[in] _grabber1 First grabber to be unliked.
      /// \param[in] _grabber2 Second grabber to be unliked.
      private: void UnlinkGrabbers(GrabberHandle *_grabber1,
          GrabberHandle *_grabber2 = NULL);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<EditorViewPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
