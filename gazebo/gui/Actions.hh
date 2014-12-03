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

#ifndef _GUI_ACTIONS_HH_
#define _GUI_ACTIONS_HH_

#include <string>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    extern GZ_GUI_VISIBLE QAction *g_newAct;
    extern GZ_GUI_VISIBLE QAction *g_openAct;
    extern GZ_GUI_VISIBLE QAction *g_importAct;
    extern GZ_GUI_VISIBLE QAction *g_saveAct;
    extern GZ_GUI_VISIBLE QAction *g_saveAsAct;
    extern GZ_GUI_VISIBLE QAction *g_saveCfgAct;
    extern GZ_GUI_VISIBLE QAction *g_cloneAct;
    extern GZ_GUI_VISIBLE QAction *g_aboutAct;
    extern GZ_GUI_VISIBLE QAction *g_quitAct;

    extern GZ_GUI_VISIBLE QAction *g_dataLoggerAct;

    extern GZ_GUI_VISIBLE QAction *g_newModelAct;
    extern GZ_GUI_VISIBLE QAction *g_resetModelsAct;
    extern GZ_GUI_VISIBLE QAction *g_resetWorldAct;
    extern GZ_GUI_VISIBLE QAction *g_editBuildingAct;
    extern GZ_GUI_VISIBLE QAction *g_editTerrainAct;
    extern GZ_GUI_VISIBLE QAction *g_editModelAct;

    extern GZ_GUI_VISIBLE QAction *g_playAct;
    extern GZ_GUI_VISIBLE QAction *g_pauseAct;
    extern GZ_GUI_VISIBLE QAction *g_stepAct;

    extern GZ_GUI_VISIBLE QAction *g_boxCreateAct;
    extern GZ_GUI_VISIBLE QAction *g_sphereCreateAct;
    extern GZ_GUI_VISIBLE QAction *g_cylinderCreateAct;
    extern GZ_GUI_VISIBLE QAction *g_meshCreateAct;
    extern GZ_GUI_VISIBLE QAction *g_pointLghtCreateAct;
    extern GZ_GUI_VISIBLE QAction *g_spotLghtCreateAct;
    extern GZ_GUI_VISIBLE QAction *g_dirLghtCreateAct;

    extern GZ_GUI_VISIBLE QAction *g_screenshotAct;

    extern GZ_GUI_VISIBLE QAction *g_showCollisionsAct;
    extern GZ_GUI_VISIBLE QAction *g_showGridAct;
    extern GZ_GUI_VISIBLE QAction *g_showContactsAct;
    extern GZ_GUI_VISIBLE QAction *g_showJointsAct;
    extern GZ_GUI_VISIBLE QAction *g_showCOMAct;
    extern GZ_GUI_VISIBLE QAction *g_transparentAct;

    extern GZ_GUI_VISIBLE QAction *g_resetAct;
    extern GZ_GUI_VISIBLE QAction *g_fullScreenAct;
    extern GZ_GUI_VISIBLE QAction *g_fpsAct;
    extern GZ_GUI_VISIBLE QAction *g_orbitAct;

    extern GZ_GUI_VISIBLE QAction *g_arrowAct;
    extern GZ_GUI_VISIBLE QAction *g_translateAct;
    extern GZ_GUI_VISIBLE QAction *g_rotateAct;
    extern GZ_GUI_VISIBLE QAction *g_scaleAct;

    extern GZ_GUI_VISIBLE QAction *g_topicVisAct;

    extern GZ_GUI_VISIBLE QAction *g_diagnosticsAct;

    extern GZ_GUI_VISIBLE QAction *g_viewWireframeAct;

    extern GZ_GUI_VISIBLE QAction *g_viewOculusAct;

    extern GZ_GUI_VISIBLE QAction *g_copyAct;
    extern GZ_GUI_VISIBLE QAction *g_pasteAct;

    extern GZ_GUI_VISIBLE QWidgetAction *g_alignAct;
    extern GZ_GUI_VISIBLE QAction *g_snapAct;

    /// \class DeleteAction Actions.hh gui/gui.hh
    /// \brief Custom delete action.
    class GAZEBO_VISIBLE DeleteAction : public QAction
    {
      Q_OBJECT
      /// \brief Constructor
      /// \param[in] _text Name of the action.
      /// \param[in] _parent Action parent object.
      public: DeleteAction(const QString &_text, QObject *_parent)
              : QAction(_text, _parent) {}

      /// \brief Emit the delete signal. This triggers the action.
      /// \param[in] _modelName Name of the model to delete.
      public: void Signal(const std::string &_modelName)
               { emit DeleteSignal(_modelName); }

      /// \brief The custom signal which a SLOT can connect to.
      /// \param[in] _modelName The name of the model to delete.
      Q_SIGNALS: void DeleteSignal(const std::string &_modelName);
    };

    /// \brief Action used to delete a model.
    extern GZ_GUI_VISIBLE DeleteAction *g_deleteAct;
  }
}
#endif
